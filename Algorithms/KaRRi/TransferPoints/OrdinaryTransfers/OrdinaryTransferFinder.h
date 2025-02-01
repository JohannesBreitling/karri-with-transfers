

#pragma once

namespace karri {

    template<typename TransferPointFinderT, typename TransfersDropoffALSStrategyT, typename InputGraphT, typename VehCHEnvT, typename CurVehLocToPickupSearchesT>
    class OrdinaryTransferFinder {
        
    // Both vehicles can drive a detour to the transfer point, but none drives the detour ALS
    // This implies, that the pickup is BNS or ORD, the dropoff can be BNS, ORD or ALS
    // If the dropoff is ALS, we need to consider a different set of vehicles to calculate the transfer psints between
    public:

        OrdinaryTransferFinder(
            TransferPointFinderT &tpFinder,
            TransfersDropoffALSStrategyT &dropoffALSStrategy,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,
            CurVehLocToPickupSearchesT &searches,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
            const RelevantPDLocs &relORDDropoffs,
            const RelevantPDLocs &relBNSDropoffs,
            std::vector<AssignmentWithTransfer> &postponedAssignments,
            const Fleet &fleet,
            const RouteState &routeState,
            RequestState &requestState,
            CostCalculator &calc,
            std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints
        ) : tpFinder(tpFinder),
            dropoffALSStrategy(dropoffALSStrategy),
            inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            relORDDropoffs(relORDDropoffs),
            relBNSDropoffs(relBNSDropoffs),
            postponedAssignments(postponedAssignments),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc),
            transferPoints(transferPoints),
            dVehIdsALS(Subset(0)) {}
        
        void findAssignments() {
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 && relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0)
                return;

            // Reset the statistcs
            numPartialsTried = 0;
            promisingPartials = std::vector<AssignmentWithTransfer>{};

            // Construct the set of possible pickup and dropoff vehicles
            std::vector<int> pVehIds = constructPVehSet();
            std::vector<int> dVehIds = constructDVehSet();

            std::vector<Vehicle> pVehs = std::vector<Vehicle>{};
            std::vector<Vehicle> dVehs = std::vector<Vehicle>{};

            for (const int pVehId : pVehIds) {
                pVehs.push_back(fleet[pVehId]);
            }

            for (const int dVehId : dVehIds) {
                dVehs.push_back(fleet[dVehId]);
            }
            
            // Loop over all the possible vehicle combinations
            for (const auto pVeh : pVehs) {
                for (const auto dVeh : dVehs) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVeh.vehicleId == pVeh.vehicleId)
                        continue;
                    
                    // Now we have a vehicle pair to work with
                    // Find the transfer points between the two vehicles
                    calculateTransferPoints(&pVeh, &dVeh);

                    if (transferPointsSize() == 0)
                        continue;

                    // Find the assignments with the transfer points
                    findAssignmentsForVehiclePair(pVeh, dVeh);

                    if (promisingPartials.size() == 0)
                        return;
                    
                    for (const auto &asgn : promisingPartials) {
                        assert(asgn.pickup->id >= 0);
                        tryDropoffBNS(asgn);
                        tryDropoffORD(asgn);
                        tryDropoffALS(asgn);
                    }
                    
                    promisingPartials.clear();

                    if (postponedAssignments.size() == 0)
                        return;

                    // Finish the postponed assignments
                    finishAssignments(pVeh, dVeh);
                    
                    postponedAssignments.clear();
                }                
            }
        }

    private:

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        void calculateTransferPoints(const Vehicle *pVeh, const Vehicle *dVeh) {
            tpFinder.init();

            //* Use the transfer point finder to find the possible transfer points       
            tpFinder.findTransferPoints(*pVeh, *dVeh);
        }

        void findAssignmentsForVehiclePair(const Vehicle &pVeh, const Vehicle &dVeh) {

            pairedLowerBoundPT = calculateLowerBoundPairedPT(pVeh, dVeh);
            pairedLowerBoundTD = calculateLowerBoundPairedTD(pVeh, dVeh);

            const int numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(dVeh.vehicleId);

            if (!relORDPickups.hasRelevantSpotsFor(pVeh.vehicleId) && !relBNSPickups.hasRelevantSpotsFor(pVeh.vehicleId))
                return;

            if (!relBNSDropoffs.hasRelevantSpotsFor(dVeh.vehicleId) && !relORDDropoffs.hasRelevantSpotsFor(dVeh.vehicleId) && dVehIdsALS.size() == 0)
                return;

            for  (int trIdxPVeh = 0; trIdxPVeh < numStopsPVeh - 1; trIdxPVeh++) {
                for  (int trIdxDVeh = 0; trIdxDVeh < numStopsDVeh - 1; trIdxDVeh++) {
                    // For fixed transfer indices, try to find possible pickups
                    tryPickupBNS(pVeh, dVeh, trIdxPVeh, trIdxDVeh);
                    tryPickupORD(pVeh, dVeh, trIdxPVeh, trIdxDVeh);
                }
            }
        }

        
        void tryPickupORD(const Vehicle &pVeh, const Vehicle &dVeh, const int trIdxPVeh, const int trIdxDVeh) {
            if (trIdxPVeh == 0 || !relORDPickups.hasRelevantSpotsFor(pVeh.vehicleId))
                return;

            const auto transferPointsForStopPair = transferPoints[{trIdxPVeh, trIdxDVeh}];

            for (const auto &pickup : relORDPickups.relevantSpotsFor(pVeh.vehicleId)) {
                if (pickup.stopIndex > trIdxPVeh)
                    continue;
                
                // Try all possible transfer points between the stops of the two vehicles
                for (const auto tp : transferPointsForStopPair) {
                    // Build the partial assignment with the transfer point
                    PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                    if (pickupPDLoc->loc == tp.loc)
                        continue;

                    AssignmentWithTransfer asgn(pVeh, dVeh, tp, pickupPDLoc, pickup.stopIndex, pickup.distToPDLoc, pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                    asgn.pickupType = ORDINARY;
                    asgn.transferTypePVeh = ORDINARY;

                    assert(asgn.pickup->id >= 0);
                    // Try the partial assignment
                    tryPartialAssignment(asgn);
                }
            }
        }
        
        void tryPickupBNS(const Vehicle &pVeh, const Vehicle &dVeh, const int trIdxPVeh, const int trIdxDVeh) {
            if (!relBNSPickups.hasRelevantSpotsFor(pVeh.vehicleId))
                return;
            
            const auto transferPointsForStopPair = transferPoints[{trIdxPVeh, trIdxDVeh}];

            for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVeh.vehicleId)) {
                for (const auto tp : transferPointsForStopPair) {
                    // Build the partial assignment with the transfer point
                    PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                    if (pickupPDLoc->loc == tp.loc)
                        continue;

                    AssignmentWithTransfer asgn(pVeh, dVeh, tp, pickupPDLoc, pickup.stopIndex, pickup.distToPDLoc, pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                    asgn.pickupType = BEFORE_NEXT_STOP;
                    asgn.transferTypePVeh = trIdxPVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                    
                    assert(asgn.pickup->id >= 0);
                    // Try the partial assignment
                    tryPartialAssignment(asgn);
                }
            }
        }

        void tryPartialAssignment(AssignmentWithTransfer &asgn) {
            numPartialsTried++;         
            if (asgn.pickupIdx == asgn.transferIdxPVeh) {
                // Paired Assignment (pVeh)
                
                // Try the lower bound for the paired assignment
                asgn.distFromPickup = 0;
                asgn.distToTransferPVeh = pairedLowerBoundPT;
                asgn.pickupPairedLowerBoundUsed = true;
            }

            if (asgn.pickupIdx == 0) {
                // Assignment with pickup BNS
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
                }
            }

            const bool unfinished = asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed;
            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) are set
            RequestCost pickupVehCost;

            assert(asgn.pVeh && asgn.pickup);
            assert(!(asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY || asgn.distToTransferPVeh == INFTY || asgn.distFromTransferPVeh == INFTY));
            
            if (unfinished) {
                pickupVehCost = calc.calcPartialCostForPVehLowerBound<true>(asgn, requestState);
            } else {
                pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);
            }
            
            if (pickupVehCost.total >= requestState.getBestCost())
                return;
            
            promisingPartials.push_back(asgn);
        }

        void tryDropoffBNS(const AssignmentWithTransfer &asgn) {
            if (asgn.transferIdxDVeh != 0 || relBNSDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId).size() == 0)
                return;

            for (const auto &dropoff : relBNSDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId)) {
                assert(asgn.transferIdxDVeh == 0);
                assert(dropoff.stopIndex == 0);

                PDLoc *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                if (dropoffPDLoc->loc == asgn.transfer.loc)
                    continue;
                
                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoff = &requestState.dropoffs[dropoff.pdId];
                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.dropoffType = BEFORE_NEXT_STOP;

                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                assert(dropoff.distFromPDLocToNextStop > 0 || dropoff.distToPDLoc == 0);
                assert(asgn.distFromDropoff > 0 || asgn.distToDropoff == 0);
                newAssignment.dropoffIdx = dropoff.stopIndex;
                newAssignment.transferTypeDVeh = BEFORE_NEXT_STOP;

                if (newAssignment.transferIdxDVeh == newAssignment.dropoffIdx) {
                    // Transfer and dropoff are inserted in the same leg (paired)
                    newAssignment.dropoffPairedLowerBoundUsed = true;
                    newAssignment.distFromTransferDVeh = 0;
                    newAssignment.distToDropoff = pairedLowerBoundTD;
                }

                // Transfer BNS in dropoff vehicle
                if (searches.knowsDistanceTransfer(newAssignment.dVeh->vehicleId, newAssignment.transfer.loc)) {
                    newAssignment.distToTransferDVeh = searches.getDistanceTransfer(newAssignment.dVeh->vehicleId, newAssignment.transfer.loc);
                } else {
                    newAssignment.dropoffBNSLowerBoundUsed = true;
                }

                if (newAssignment.dropoff->loc == newAssignment.transfer.loc)
                    continue;

                requestState.tryAssignment(newAssignment);
            }
        }

        void tryDropoffORD(const AssignmentWithTransfer &asgn) {
            if (relORDDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId).size() == 0)
                return;

            for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId)) {
                if (dropoff.stopIndex < asgn.transferIdxDVeh)
                    continue;

                PDLoc *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                if (dropoffPDLoc->loc == asgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoff = dropoffPDLoc;
                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                assert(dropoff.distFromPDLocToNextStop > 0 || dropoff.distToPDLoc == 0);
                assert(asgn.distFromDropoff > 0 || asgn.distToDropoff == 0);
                newAssignment.dropoffIdx = dropoff.stopIndex;
                newAssignment.dropoffType = ORDINARY;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                if (newAssignment.transferIdxDVeh == newAssignment.dropoffIdx) {
                    // Transfer and dropoff are inserted in the same leg (paired)
                    newAssignment.dropoffPairedLowerBoundUsed = true;
                    newAssignment.distFromTransferDVeh = 0;
                    newAssignment.distToDropoff = pairedLowerBoundTD;
                }

                if (newAssignment.dropoff->loc == newAssignment.transfer.loc)
                    continue;

                // If the dropoff coincides with a stop, we skip the assignment if the dropoff will be inserted in a different leg
                if (dropoffIsAtStop(newAssignment.dVeh, dropoffPDLoc->loc) >= 0 && dropoff.stopIndex != dropoffIsAtStop(newAssignment.dVeh, dropoffPDLoc->loc))
                    continue; 

                requestState.tryAssignment(newAssignment);
            }
        }

        void tryDropoffALS(const AssignmentWithTransfer &asgn) {
            if (!alsDropoffVehs[asgn.dVeh->vehicleId])
                return;
            
            for (const auto &dropoff : requestState.dropoffs) {
                int distanceToDropoff = dropoffALSStrategy.getDistanceToDropoff(asgn.dVeh->vehicleId, dropoff.id);
                if (distanceToDropoff == INFTY)
                    continue;

                if (dropoff.loc == asgn.transfer.loc)
                    continue;
                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoff = &dropoff;
                newAssignment.distToDropoff = distanceToDropoff;
                newAssignment.distFromDropoff = 0;
                newAssignment.dropoffIdx = routeState.numStopsOf(asgn.dVeh->vehicleId) - 1;
                newAssignment.dropoffType = AFTER_LAST_STOP;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                if (newAssignment.dropoff->loc == newAssignment.transfer.loc)
                    continue;

                requestState.tryAssignment(newAssignment);
            }
        }

        int transferPointsSize() {
            int total = 0;
            for (const auto &it : transferPoints)
                total += it.second.size();
            
            return total;
        }

        void finishAssignments(const Vehicle &pVeh, const Vehicle &dVeh) {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate = std::vector<AssignmentWithTransfer>{};
            std::vector<AssignmentWithTransfer> currentlyCalculating = std::vector<AssignmentWithTransfer>{};
            std::vector<AssignmentWithTransfer> temp = std::vector<AssignmentWithTransfer>{};

            RequestCost total;
            // Start with the pickups with postponed bns distance
            for (auto asgn : postponedAssignments) {
                if (asgn.pickupBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                } else {
                    toCalculate.push_back(asgn);
                }
            }

            if (currentlyCalculating.size() > 0)
                searches.computeExactDistancesVia(pVeh);

            for (auto asgn : currentlyCalculating) {
                assert(searches.knowsDistance(pVeh.vehicleId, asgn.pickup->id));
                const int distance = searches.getDistance(pVeh.vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                if (!asgn.isFinished()) {
                    total = calc.calcBaseLowerBound<true, true>(asgn, requestState);
                
                    if (total.total > requestState.getBestCost())
                        continue;

                    toCalculate.push_back(asgn);
                } else {
                    requestState.tryAssignment(asgn);
                    continue;
                }
            }

            // Calculate the exact paired distance between pickup and transfer
            currentlyCalculating.clear();

            auto sources = std::vector<int>{};
            auto targets = std::vector<int>{};
            auto offsets = std::vector<int>{};

            for (auto asgn : toCalculate) {
                if (asgn.pickupPairedLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.pickup->loc));
                    int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.transfer.loc));
                    int offset = inputGraph.travelTime(asgn.transfer.loc);
                    sources.push_back(sourceRank);
                    targets.push_back(targetRank);
                    offsets.push_back(offset);
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            // Calculate the direct distances between pickup and transfer
            std::vector<int> distances;
            if (currentlyCalculating.size() > 0)
                distances = vehChQuery.runManyToMany(sources, targets, offsets);

            for (int i = 0; i < currentlyCalculating.size(); i++) {
                auto asgn = currentlyCalculating[i];
                asgn.distToTransferPVeh = distances[i];
                asgn.pickupPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                if (!asgn.isFinished()) {
                    total = calc.calcBaseLowerBound<true, true>(asgn, requestState);

                    if (total.total > requestState.getBestCost())
                        continue;
                    toCalculate.push_back(asgn);
                } else {
                    requestState.tryAssignment(asgn);
                    continue;
                }        
            }

            currentlyCalculating.clear();
            // Calculate the dropoffs with postponed bns distance
            for (auto asgn : toCalculate) {
                if (asgn.dropoffBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    asgn.dropoffBNSLowerBoundUsed = false;
                    searches.addTransferForProcessing(asgn.transfer.loc, asgn.distToTransferDVeh);
                } else {
                    temp.push_back(asgn);
                }

                if (!asgn.isFinished())
                    temp.push_back(asgn);
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactTransferDistancesVia(dVeh);

            for (auto asgn : currentlyCalculating) {
                assert(searches.knowsDistanceTransfer(dVeh.vehicleId, asgn.transfer.loc));
                const int distance = searches.getDistanceTransfer(dVeh.vehicleId, asgn.transfer.loc);
                asgn.distToTransferDVeh = distance;
                asgn.dropoffBNSLowerBoundUsed = false;

                if (!asgn.isFinished()) {
                    total = calc.calcBaseLowerBound<true, false>(asgn, requestState);

                    if (total.total > requestState.getBestCost())
                        continue;

                    toCalculate.push_back(asgn);
                } else {
                    requestState.tryAssignment(asgn);
                }   
            }

            // Calculate the exact paired distance between transfer and dropoff
            currentlyCalculating.clear();

            sources = std::vector<int>{};
            targets = std::vector<int>{};
            offsets = std::vector<int>{};

            for (auto asgn : toCalculate) {
                assert(asgn.dropoffPairedLowerBoundUsed);
                
                currentlyCalculating.push_back(asgn);
                int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.transfer.loc));
                int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.dropoff->loc));
                int offset = inputGraph.travelTime(asgn.dropoff->loc);
                sources.push_back(sourceRank);
                targets.push_back(targetRank);
                offsets.push_back(offset);
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            distances.clear();
            if (currentlyCalculating.size() > 0)
                distances = vehChQuery.runManyToMany(sources, targets, offsets);

            for (int i = 0; i < currentlyCalculating.size(); i++) {
                auto asgn = currentlyCalculating[i];
                asgn.distToDropoff = distances[i];
                asgn.dropoffPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                assert(asgn.isFinished());
                requestState.tryAssignment(asgn);
            }
        }

        std::vector<int> constructPVehSet() {
            std::vector<bool> markedVehs(fleet.size(), false);
            std::vector<int> pVehIds = std::vector<int>{};

            for (const int pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                if (markedVehs[pVehId])
                    continue;
                
                pVehIds.push_back(pVehId);
                markedVehs[pVehId] = true;
            }

            for (const int pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (markedVehs[pVehId])
                    continue;
                
                pVehIds.push_back(pVehId);
                markedVehs[pVehId] = true;
            }

            return pVehIds;
        }

        std::vector<int> constructDVehSet() {
            alsDropoffVehs = std::vector<bool>(fleet.size(), false);
            std::vector<int> dVehIds = std::vector<int>{};

            for (const int dVehId : relBNSDropoffs.getVehiclesWithRelevantPDLocs()) {
                if (alsDropoffVehs[dVehId])
                    continue;
                
                dVehIds.push_back(dVehId);
                alsDropoffVehs[dVehId] = true;
            }

            for (const int dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                if (alsDropoffVehs[dVehId])
                    continue;
                
                dVehIds.push_back(dVehId);
                alsDropoffVehs[dVehId] = true;
            }

            // Calculate the possible vehicles for dropoff als
            dVehIdsALS = dropoffALSStrategy.findDropoffsAfterLastStop();

            for (const int dVehId : dVehIdsALS) {
                dVehIds.push_back(dVehId);
            }

            return dVehIds;
        }

        int calculateLowerBoundPairedPT(const Vehicle &pVeh, const Vehicle & /*dVeh*/) {
            std::vector<int> sources = std::vector<int>{};
            std::vector<int> targets = std::vector<int>{};

            for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVeh.vehicleId)) {
                const auto pickupEdge = requestState.pickups[pickup.pdId].loc;
                const auto head = inputGraph.edgeHead(pickupEdge);
                
                sources.push_back(head);
            }

            for (const auto &pickup : relORDPickups.relevantSpotsFor(pVeh.vehicleId)) {
                const auto pickupEdge = requestState.pickups[pickup.pdId].loc;
                const auto head = inputGraph.edgeHead(pickupEdge);

                sources.push_back(head);
            }

            assert(sources.size() > 0);
            
            for (auto &it : transferPoints) {
                for (const auto tp : it.second) {
                    const auto head = inputGraph.edgeHead(tp.loc);
                    targets.push_back(head);
                }
            }

            assert(targets.size() > 0);

            const auto sourceRanks = vehCh.allRanks(sources);
            const auto targetRanks = vehCh.allRanks(targets);

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks);
            
            return lb; 
        }

        int calculateLowerBoundPairedTD(const Vehicle & /*pVeh*/, const Vehicle &dVeh) {
            std::vector<int> sources = std::vector<int>{};
            std::vector<int> targets = std::vector<int>{};

            for (const auto &dropoff : relBNSDropoffs.relevantSpotsFor(dVeh.vehicleId)) {
                const auto dropoffEdge = requestState.dropoffs[dropoff.pdId].loc;
                const auto tail = inputGraph.edgeTail(dropoffEdge);
                
                targets.push_back(tail);
            }

            for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVeh.vehicleId)) {
                const auto dropoffEdge = requestState.dropoffs[dropoff.pdId].loc;
                const auto tail = inputGraph.edgeTail(dropoffEdge);

                targets.push_back(tail);
            }

            // We dont have to consider the als dropoffs, because a paired assignemnt with dropoff als would imply that the transfer is als

            for (auto &it : transferPoints) {
                for (const auto tp : it.second) {
                    const auto head = inputGraph.edgeHead(tp.loc);
                    sources.push_back(head);
                }
            }

            const auto sourceRanks = vehCh.allRanks(sources);
            const auto targetRanks = vehCh.allRanks(targets);

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks);
            
            return lb;
        }

        // If the dropoff coincides with a stop, we return the index of the stop
        // Otherwise -1 is returned
        int dropoffIsAtStop(const Vehicle* dVeh, const int dropoffLoc) {
            for (int i = 0; i < routeState.numStopsOf(dVeh->vehicleId); i++) {
                if (routeState.stopLocationsFor(dVeh->vehicleId)[i] == dropoffLoc)
                    return i;
            }

            return -1;
        }

        int pairedLowerBoundPT = INFTY;
        int pairedLowerBoundTD = INFTY;

        std::vector<AssignmentWithTransfer> promisingPartials;

        TransferPointFinderT &tpFinder;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;

        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;
        const RelevantPDLocs &relBNSDropoffs;

        std::vector<AssignmentWithTransfer> &postponedAssignments;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        CostCalculator &calc;

        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;
        
        std::vector<bool> alsDropoffVehs;
        Subset dVehIdsALS;

        int numPartialsTried;
        
    };

}
