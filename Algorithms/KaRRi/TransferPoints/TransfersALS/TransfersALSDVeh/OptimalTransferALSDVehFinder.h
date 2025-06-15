/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************

#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename TransferALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
    class OptimalTransferALSDVehFinder {

        // The dVeh drives the detour to the transfer point
        // This implies, that the dVeh drives from its last stop to a stop of the pVeh and picks up the customer
        // The dVeh then will drive to the dropoff and go into idle mode (dropoff ALS)
        // The pickup has to be BNS or ORD
    public:
        OptimalTransferALSDVehFinder(
                InputGraphT &inputGraph,
                VehCHEnvT &vehChEnv,
                TransferALSStrategyT &strategy,
                CurVehLocToPickupSearchesT &searches,
                const RelevantPDLocs &relORDPickups,
                const RelevantPDLocs &relBNSPickups,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                InsertionAsserterT &asserter
        ) : inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),
            strategy(strategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(routeState),
            lastStopToTransfersDistances(),
            transferToDropoffDistances(),
            asserter(asserter) {}

        void findAssignments(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer) {
            Timer total;

            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                return;

            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty() &&
                relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;
            
            bestCost = INFTY;
            
            // Collect the relevant last stops
            std::vector<int> relevantLastStopLocs;
            for (const int dVehId : relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const int numStops = routeState.numStopsOf(dVehId);
                const int lastStopLoc = routeState.stopLocationsFor(dVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLoc);
            }

            // Collect the relevant transfer locs
            transferEdges.clear();
            std::vector<bool> pVehFlags(fleet.size(), false);
            for (const int pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const int numStops = routeState.numStopsOf(pVehId);
                const auto stopIds = routeState.stopIdsFor(pVehId);
                pVehFlags[pVehId] = true;

                for (int i = 1; i < numStops - 1; i++) {
                    const auto& ellipse = ellipseContainer.getEdgesInEllipse(stopIds[i]);

                    for (const auto edge : ellipse)
                        transferEdges.push_back(edge);
                }
            }

            for (const int pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (pVehFlags[pVehId])
                    continue;
                
                const int numStops = routeState.numStopsOf(pVehId);
                const auto stopIds = routeState.stopIdsFor(pVehId);

                for (int i = 1; i < numStops - 1; i++) {
                    const auto& ellipse = ellipseContainer.getEdgesInEllipse(stopIds[i]);

                    for (const auto edge : ellipse)
                        transferEdges.push_back(edge);
                }
            }
            
            std::vector<int> dropoffLocs;
            for (const auto& dropoff : requestState.dropoffs) {
                dropoffLocs.push_back(dropoff.loc);
            }

            // Calculate the distances from the relevant last stops to the transfer points and from the transfer points to the dropoffs
            lastStopToTransfersDistances.clear();
            transferToDropoffDistances.clear();

            lastStopToTransfersDistances = strategy.calculateDistancesFromLastStopToAllTransfers(relevantLastStopLocs, transferEdges);
            transferToDropoffDistances = strategy.caluclateDistancesFromAllTransfersToDropoffs(transferEdges, dropoffLocs);

            findAssignmentsWithDropoffALS(relALSDropoffs, ellipseContainer);

            KASSERT(bestCost >= INFTY || asserter.assertAssignment(bestAssignment));

            // Write the stats
            auto &stats = requestState.stats().transferALSDVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;
            stats.numPickups += requestState.numPickups();
            stats.numDropoffs += requestState.numDropoffs();
            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numTransferPoints += numTransferPoints;

            stats.searchTimeDropoffALS += searchTimeDropoffALS;
            stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            stats.searchTimeTransferToDropoff += searchTimeTransferToDropoff;
        }

        void init() {
            totalTime = 0;

            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesDropoffALS = 0;

            numPickups = 0;
            numDropoffs = 0;

            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;

            numTransferPoints = 0;

            searchTimeDropoffALS = 0;
            searchTimeLastStopToTransfer = 0;
            searchTimeTransferToDropoff = 0;
        }

    private:
        void findAssignmentsWithDropoffALS(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer) {
            // The vehicle set for the dropoff is the set of vehicles for the ALS dropoff
            // The distance from the last stop to the dropoff is a lower bound for the distance from the last stop to the dropoff via the transfer point
            const auto& dVehIds = relALSDropoffs.getVehiclesWithRelevantPDLocs();
            
            std::vector<AssignmentWithTransfer> postponedAssignments;
            for (const auto dVehId: dVehIds) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                // const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
                const int lastStopLocDVeh = stopLocationsDVeh[numStopsDVeh - 1];

                // Pickup BNS
                for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                    KASSERT(postponedAssignments.empty());
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;

                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopIdsPVeh = routeState.stopIdsFor(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    for (const auto &dropoff: requestState.dropoffs) {
                        for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVehId)) {
                            assert(pickup.stopIndex == 0);
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = 1; i < numStopsPVeh - 1; i++) { // We start at i = 1 because at the first stop the transfer is only possible if the vehcle is currently waiting for another passenger, so we neglect this case
                                const int stopId = stopIdsPVeh[i];
                                const auto& transferPoints = ellipseContainer.getEdgesInEllipse(stopId);
                                for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {

                                    const auto edge = transferPoints[tpIdx];
                                    const int tpLoc = edge.edge;
                                    const int tpOffset = inputGraph.travelTime(tpLoc);

                                    const bool transferAtLastStop = tpLoc == stopLocationsDVeh[numStopsDVeh - 1];
                                    // Construct the transfer point
                                    TransferPoint tp = TransferPoint(tpLoc, pVeh, dVeh);
                                    numTransferPoints++;

                                    tp.distancePVehToTransfer = edge.distToTail + tpOffset;
                                    tp.distancePVehFromTransfer = edge.distFromHead;
                                
                                    const int distToTransferDVeh = lastStopToTransfersDistances[lastStopLocDVeh][tpLoc];
                                    KASSERT(tpLoc == stopLocationsDVeh[numStopsDVeh - 1] || distToTransferDVeh > 0);
                                    tp.distanceDVehToTransfer = distToTransferDVeh;
                                    tp.distanceDVehFromTransfer = 0;

                                    tp.stopIdxPVeh = i;
                                    tp.stopIdxDVeh = numStopsDVeh - 1;

                                    // Build the resulting assignment
                                    AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                                    asgn.pickup = pickupPDLoc;
                                    asgn.dropoff = &dropoff;

                                    asgn.pickupIdx = pickup.stopIndex;
                                    asgn.dropoffIdx = numStopsDVeh - 1;
                                    asgn.transferIdxPVeh = i;
                                    asgn.transferIdxDVeh = numStopsDVeh - 1;

                                    asgn.distToPickup = pickup.distToPDLoc;
                                    asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                    asgn.distToDropoff = transferToDropoffDistances[dropoff.loc][tpLoc];
                                    asgn.distFromDropoff = 0;

                                    // Check transfer at stop pVeh
                                    const bool transferAtStopPVeh = tp.loc == stopLocationsPVeh[i] && asgn.pickupIdx != asgn.transferIdxPVeh;
                                    if (transferAtStopPVeh) {
                                        asgn.distToTransferPVeh = 0;
                                        
                                        const int nextLeg = asgn.transferIdxPVeh < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[asgn.transferIdxPVeh + 1] - routeState.schedDepTimesFor(pVehId)[asgn.transferIdxPVeh] : 0;
                                        asgn.distFromTransferPVeh = nextLeg;
                                    }

                                    KASSERT(asgn.pickupIdx != asgn.transferIdxPVeh); // In the pickup bns case we do not have a paired assignment
                                    asgn.distToTransferDVeh = transferAtLastStop ? 0 : distToTransferDVeh;
                                    asgn.distFromTransferDVeh = 0;
                                
                                    asgn.pickupType = BEFORE_NEXT_STOP;
                                    asgn.transferTypePVeh = ORDINARY;
                                    asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                    asgn.dropoffType = AFTER_LAST_STOP;
                                
                                    // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                    if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                        continue;

                                    if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                                        asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                                    } else {
                                        asgn.pickupBNSLowerBoundUsed = true;
                                    }

                                    // Try the potentially unfinished assignment with BNS pickup
                                    tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                                }
                            }
                        }
                    }

                    if (postponedAssignments.empty())
                        continue;

                    finishAssignmentsWithPickupBNSLowerBound(pVeh, postponedAssignments);
                }

                KASSERT(postponedAssignments.empty());

                // Pickup ORD
                for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;
                    
                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopIdsPVeh = routeState.stopIdsFor(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    for (const auto &dropoff: requestState.dropoffs) {
                        //* Calculate the distances from the stops of the pVeh to the possible dropoffs
                        for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = pickup.stopIndex + 1; i < numStopsPVeh - 1; i++) {
                                assert(pickup.stopIndex > 0);
                                const int stopId = stopIdsPVeh[i];
                                const auto& transferPoints = ellipseContainer.getEdgesInEllipse(stopId);
                                for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {
                                    const auto edge = transferPoints[tpIdx];
                                    const int tpLoc = edge.edge;
                                    const int tpOffset = inputGraph.travelTime(tpLoc);

                                    const bool transferAtLastStop = tpLoc == stopLocationsDVeh[numStopsDVeh - 1];
                                    // Construct the transfer point
                                    TransferPoint tp = TransferPoint(tpLoc, pVeh, dVeh);
                                    numTransferPoints++;
                                    
                                    tp.distancePVehToTransfer = edge.distToTail + tpOffset;
                                    tp.distancePVehFromTransfer = edge.distFromHead;
                                
                                    const int distToTransferDVeh = lastStopToTransfersDistances[lastStopLocDVeh][tpLoc];
                                    KASSERT(distToTransferDVeh > 0 || stopLocationsDVeh[numStopsDVeh - 1] == tpLoc);
                                    tp.distanceDVehToTransfer = distToTransferDVeh;
                                    tp.distanceDVehFromTransfer = 0;

                                    tp.stopIdxPVeh = i;
                                    tp.stopIdxDVeh = numStopsDVeh - 1;

                                    // Build the resulting assignment
                                    AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                                    asgn.pickup = pickupPDLoc;
                                    asgn.dropoff = &dropoff;

                                    asgn.pickupIdx = pickup.stopIndex;
                                    asgn.dropoffIdx = numStopsDVeh - 1;
                                    asgn.transferIdxPVeh = i;
                                    asgn.transferIdxDVeh = numStopsDVeh - 1;

                                    asgn.distToPickup = pickup.distToPDLoc;
                                    asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                    asgn.distToDropoff = transferToDropoffDistances[dropoff.loc][tpLoc];
                                    asgn.distFromDropoff = 0;

                                    // Check transfer at stop pVeh
                                    const bool transferAtStopPVeh = tp.loc == stopLocationsPVeh[i] && asgn.pickupIdx != asgn.transferIdxPVeh;
                                    if (transferAtStopPVeh) {
                                        asgn.distToTransferPVeh = 0;
                                        
                                        const int nextLeg = asgn.transferIdxPVeh < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[asgn.transferIdxPVeh + 1] - routeState.schedDepTimesFor(pVehId)[asgn.transferIdxPVeh] : 0;
                                        asgn.distFromTransferPVeh = nextLeg;
                                    }
                                    
                                    asgn.distToTransferDVeh = transferAtLastStop ? 0 : distToTransferDVeh;
                                    asgn.distFromTransferDVeh = 0;
                                
                                    asgn.pickupType = BEFORE_NEXT_STOP;
                                    asgn.transferTypePVeh = ORDINARY;
                                    asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                    asgn.dropoffType = AFTER_LAST_STOP;
                                
                                    // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                    if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                        continue;
                                
                                    if (asgn.pickupIdx == asgn.transferIdxPVeh) {
                                        asgn.pickupPairedLowerBoundUsed = true;
                                        asgn.distToTransferPVeh = 0;
                                        asgn.distFromPickup = 0;
                                    }

                                    // Try the potentially unfinished assignment with ORD pickup
                                    tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                                }
                            }
                        }
                    }
                }
            }

            if (postponedAssignments.empty())
                return;

            finishAssignmentsWithPickupPairedLowerBound(postponedAssignments);
        }

        // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            return (asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                   || (asgn.transferIdxPVeh < numStopsPVeh - 1 &&
                       asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1]);
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn) {
            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    numAssignmentsTriedPickupORD++;
                    break;

                default:
                    assert(false);
            }
            numAssignmentsTriedDropoffALS++;
        }

        void tryFinishedAssignment(AssignmentWithTransfer &asgn) {
            KASSERT(asgn.isFinished());

            if (canSkipAssignment(asgn))
                return;

            trackAssignmentTypeStatistic(asgn);

            Timer time;
            const auto cost = calc.calc(asgn, requestState);
            requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);

            if (cost.total < bestCost) {
                bestCost = cost.total;
                bestAssignment = asgn;
            }

            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer &asgn,
                                                std::vector<AssignmentWithTransfer>& postponedAssignments) {
            
            if (canSkipAssignment(asgn))
                return;

            Timer time;
            if (!asgn.isFinished()) {
                const auto cost = calc.calcLowerBound(asgn, requestState);
                if (cost.total <= requestState.getBestCost()) {
                    postponedAssignments.push_back(asgn);
                }
            } else {
                tryFinishedAssignment(asgn);
            }
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void finishAssignmentsWithPickupBNSLowerBound(const Vehicle *pVeh, std::vector<AssignmentWithTransfer> &postponedAssignments) {
            std::vector<AssignmentWithTransfer> toCalculate;
            std::vector<AssignmentWithTransfer> pairedAssignments;
            
            for (const auto &asgn: postponedAssignments) {
                KASSERT(asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed);
                if (asgn.pickupBNSLowerBoundUsed) {
                    searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                    toCalculate.push_back(asgn);
                    continue;
                }
                pairedAssignments.push_back(asgn);
            }

            searches.computeExactDistancesVia(*pVeh);

            std::vector<AssignmentWithTransfer> assignmentsWithTwoLowerBounds;
            for (auto& asgn: toCalculate) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;
                    
                tryPotentiallyUnfinishedAssignment(asgn, assignmentsWithTwoLowerBounds);
            }

            postponedAssignments.clear();
            for (auto& asgn : pairedAssignments) {
                KASSERT(asgn.pickupPairedLowerBoundUsed);
                postponedAssignments.push_back(asgn);
            }

            for (auto& asgn : assignmentsWithTwoLowerBounds) {
                KASSERT(asgn.pickupPairedLowerBoundUsed);
                postponedAssignments.push_back(asgn);
            }
        }

        void finishAssignmentsWithPickupPairedLowerBound(std::vector<AssignmentWithTransfer> &postponedAssignments) {
            for (auto &asgn : postponedAssignments) {
                KASSERT(asgn.pickupPairedLowerBoundUsed);
                
                // Calculate the paired distance using a point to point query
                const int pickupLoc = asgn.pickup->loc;
                const int transferLoc = asgn.transfer.loc;

                const int pickupRank = vehCh.rank(inputGraph.edgeHead(pickupLoc));
                const int transferRank = vehCh.rank(inputGraph.edgeTail(transferLoc));
                const int transferOffset = inputGraph.travelTime(transferLoc);

                vehChQuery.run(pickupRank, transferRank);
                const int pairedDistance = vehChQuery.getDistance() + transferOffset;
                asgn.distToTransferPVeh = pairedDistance;

                asgn.pickupPairedLowerBoundUsed = false;
                KASSERT(asgn.isFinished());
                tryFinishedAssignment(asgn);
            }

            postponedAssignments.clear();
        }

        InputGraphT inputGraph;

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;
        
        const CH &vehCh;
        VehCHQuery vehChQuery;

        TransferALSStrategyT &strategy;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator calc;

        std::vector<EdgeInEllipse> transferEdges;
        
        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, int>> lastStopToTransfersDistances;
        std::map<int, std::map<int, int>> transferToDropoffDistances;

        InsertionAsserterT &asserter;

        AssignmentWithTransfer bestAssignment;
        int bestCost;

        //* Statistics for the transfer als dveh assignment finder
        int64_t totalTime;

        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;

        int64_t numPickups;
        int64_t numDropoffs;

        int64_t numCandidateVehiclesDropoffALS;

        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;

        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;

        // Stats for the transfer search itself
        int64_t numTransferPoints;

        int64_t searchTimeDropoffALS;
        int64_t searchTimeLastStopToTransfer;
        int64_t searchTimeTransferToDropoff;

    };
}
