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

#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"

#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename TransferALSStrategyT, typename TransfersPickupALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
    class OptimalTransferALSPVehFinder {

        // The pVeh drives the detour to the transfer point
        // This implies, that the pVeh drives from its last stop to a stop of the dVeh and drops off the customer
        // The dVeh then will then perform the dropoff ORD or ALS
        // The pickup could be BNS, ORD or ALS
    public:

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        OptimalTransferALSPVehFinder(
                InputGraphT &inputGraph,
                VehCHEnvT &vehChEnv,
                TransferALSStrategyT &strategy,
                TransfersPickupALSStrategyT &pickupALSStrategy,
                CurVehLocToPickupSearchesT &searches,
                const RelevantPDLocs &relORDPickups,
                const RelevantPDLocs &relBNSPickups,
                const RelevantPDLocs &relORDDropoffs,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                CostCalculator &calc,
                InsertionAsserterT &asserter
        ) : inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),
            strategy(strategy),
            pickupALSStrategy(pickupALSStrategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            relORDDropoffs(relORDDropoffs),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc),
            asserter(asserter),
            pVehIdsALS(fleet.size()),
            bestCost(INFTY) {}

        void init() {
            totalTime = 0;

            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesPickupALS = 0;

            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;

            numPickups = 0;
            numDropoffs = 0;

            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedPickupALS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;

            tryAssignmentsTime = 0;

            numTransferPoints = 0;

            searchTimePickupALS = 0;
            searchTimeDropoffALS = 0;
            searchTimeLastStopToTransfer = 0;
            searchTimePickupToTransfer = 0;
        }

        void findAssignments(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer) {
            Timer total;

            bestCost = INFTY;

            // Reset the last stop distances
            lastStopToTransfersDistances.clear();
            pickupToTransfersDistances.clear();

            //* Collect the full ellipses of possible transfer points
            // Collect all stop ids of last stops of potential pickup vehicles (without als, as for pickup als we first have to search to the pickups and the from the pickups)
            std::vector<int> relevantPVehIds;
            std::vector<int> relevantDVehIds;

            std::vector<bool> pVehIdFlags(fleet.size(), false);
            std::vector<bool> dVehIdFlags(fleet.size(), false);

            std::vector<int> relevantLastStopLocs;

            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                relevantPVehIds.push_back(pVehId);
                pVehIdFlags[pVehId] = true;
                
                const auto numStops = routeState.numStopsOf(pVehId);
                const auto lastStopLoc = routeState.stopLocationsFor(pVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLoc);
            }

            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (!pVehIdFlags[pVehId]) {
                    relevantPVehIds.push_back(pVehId);
                    pVehIdFlags[pVehId] = true;

                    const auto numStops = routeState.numStopsOf(pVehId);
                    const auto lastStopLocs = routeState.stopLocationsFor(pVehId)[numStops - 1];
                    relevantLastStopLocs.push_back(lastStopLocs);
                }
            }

            if (!relevantPVehIds.empty()) {
                for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (routeState.numStopsOf(dVehId) <= 1)
                        continue;
                    
                    relevantDVehIds.push_back(dVehId);
                    dVehIdFlags[dVehId] = true;
                }

                for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (!dVehIdFlags[dVehId] && routeState.numStopsOf(dVehId) > 1) {
                        relevantDVehIds.push_back(dVehId);
                        dVehIdFlags[dVehId] = true;
                    }
                }
            }

            if (relevantDVehIds.size() == 0 || relevantPVehIds.empty())
                return;

            // Get all stop ids of the dropoff vehicles
            std::vector<int> stopIdsDVehs;
            for (const auto dVehId : relevantDVehIds) {
                const auto numStops = routeState.numStopsOf(dVehId);
                KASSERT(numStops > 1);

                const auto stopIds = routeState.stopIdsFor(dVehId);

                for (int i = 0; i < numStops - 1; i++) {
                    stopIdsDVehs.push_back(stopIds[i]);
                }
            }
            
            // Combine all potential locations from the ellipses
            // Maps the stopId to the start index of the ellipse in the transferEdges
            transferEdges.clear();
            
            for (const int stopId : stopIdsDVehs) {
                const auto& ellipse = ellipseContainer.getEdgesInEllipse(stopId);

                for (const auto edge : ellipse) {
                    transferEdges.push_back(edge);
                }
            }

            // Calculate the distances from all last stops (pickup ord, bns) to the potential transfers
            if (relevantLastStopLocs.size() > 0 && transferEdges.size() > 0) {
                lastStopToTransfersDistances = strategy.calculateDistancesFromLastStopToAllTransfers(relevantLastStopLocs, transferEdges);
            }

            pVehIdsALS = pickupALSStrategy.findPickupsAfterLastStop();
            std::vector<int> pickupLocs;
            for (const auto pickup : requestState.pickups) {
                pickupLocs.push_back(pickup.loc);
            }

            // Calculate the distances from all pickups to the potential transfers
            if (pickupLocs.size() > 0 && transferEdges.size() > 0) {
                pickupToTransfersDistances = strategy.caluclateDistancesFromPickupsToAllTransfers(pickupLocs, transferEdges);
            }

            std::vector<AssignmentWithTransfer> postponedAssignments;
            findAssignmentsWithPickupBNS(relALSDropoffs, ellipseContainer, postponedAssignments);
            findAssignmentsWithPickupORD(relALSDropoffs, ellipseContainer, postponedAssignments);
            findAssignmentsWithPickupALS(relALSDropoffs, ellipseContainer, postponedAssignments);
            
            if (!postponedAssignments.empty())
                finishAssignmentsWithDropoffPairedLowerBound(postponedAssignments);
            
            KASSERT(postponedAssignments.empty());

            KASSERT(bestCost >= INFTY || asserter.assertAssignment(bestAssignment));

            // // Write the stats
            // auto &stats = requestState.stats().transferALSPVehStats;
            // stats.totalTime = total.elapsed<std::chrono::nanoseconds>();

            // stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            // stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            // stats.numCandidateVehiclesPickupALS += numCandidateVehiclesPickupALS;
            // stats.numCandidateVehiclesDropoffORD += numCandidateVehiclesDropoffORD;
            // stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;

            // stats.numPickups += requestState.numPickups();
            // stats.numDropoffs += requestState.numDropoffs();

            // stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            // stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            // stats.numAssignmentsTriedPickupALS += numAssignmentsTriedPickupALS;
            // stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            // stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;

            // stats.tryAssignmentsTime += tryAssignmentsTime;

            // stats.numTransferPoints += numTransferPoints;

            // stats.searchTimePickupALS += searchTimePickupALS;
            // stats.searchTimeDropoffALS += searchTimeDropoffALS;
            // stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            // stats.searchTimePickupToTransfer += searchTimePickupToTransfer;
        }

    private:
        void findAssignmentsWithPickupORD(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Loop over all possible vehicles and pickups
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, postponedAssignments, ellipseContainer);
                    tryDropoffALS(pVeh, &pickup, relALSDropoffs, postponedAssignments, ellipseContainer);
                }
            }
        }

        void findAssignmentsWithPickupBNS(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            //* In this case we consider all vehicles that are able to perform the pickup BNS
            if (relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Loop over all possible vehicles and pickups
            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];
                
                for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, postponedAssignments, ellipseContainer);
                    tryDropoffALS(pVeh, &pickup, relALSDropoffs, postponedAssignments, ellipseContainer);
                }

                if (postponedAssignments.empty())
                    continue;

                finishAssignmentsWithPickupBNSLowerBound(pVeh, postponedAssignments);
            }
        }


        void findAssignmentsWithPickupALS(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EdgeEllipseContainer& ellipseContainer, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            // //* In this case we consider all vehicles that are able to perform the pickup ALS

            // Timer pickupALSTimer;
            // const auto pVehIds = pickupALSStrategy.findPickupsAfterLastStop();
            // searchTimePickupALS += pickupALSTimer.elapsed<std::chrono::nanoseconds>();

            // numCandidateVehiclesPickupALS += pVehIds.size();

            // if (pVehIds.empty())
            //     return;

            // std::vector<int> relevantDVehIds;
            // for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
            //     relevantDVehIds.push_back(dVehId);
            // }

            // for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
            //     relevantDVehIds.push_back(dVehId);
            // }

            // if (relevantDVehIds.empty())
            //     return;

            // // TODO

            // std::vector<int> pickupLocs;
            // for (const auto &p : requestState.pickups) {
            //     pickupLocs.push_back(p.loc);
            // }

            // // Calculate the distances from all pickups to all stops of relevant dropoff vehicles
            // Timer pickupToTransferSearchTimer;
            // const auto transferPoints = strategy.caluclateDistancesFromPickupsToAllTransfers(pickupLocs, transferEdges);

            // transferDistances = strategy.calculateDistancesFromAllPickupsToAllStops(pickupLocs,
            //                                                                         relevantDVehIds);
            // searchTimePickupToTransfer = pickupToTransferSearchTimer.elapsed<std::chrono::nanoseconds>();

            for (const auto pVehId: pVehIdsALS) {
                const auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: requestState.pickups) {
                    // Get the distance from the last stop of the pVeh to the pickup
                    const int distanceToPickup = pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                    // KASSERT(asserter.assertLastStopDistance(pVehId, pickup.loc) == distanceToPickup);
                    KASSERT(!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty() || !relORDDropoffs.getVehiclesWithRelevantPDLocs().empty());
                    tryDropoffORDForPickupALS(pVeh, &pickup, distanceToPickup, ellipseContainer, postponedAssignments);
                    tryDropoffALSForPickupALS(pVeh, &pickup, distanceToPickup, relALSDropoffs, ellipseContainer);
                }
            }
        }


        void tryDropoffORDForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup, const EdgeEllipseContainer& ellipseContainer, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                // const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                Timer searchTimer;

                for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        if (i >= numStopsDVeh - 1)
                            continue;
                        
                        const int stopId = stopIdsDVeh[i];
                        const auto& transferEdges = ellipseContainer.getEdgesInEllipse(stopId);

                        for (int tpIdx = 0; tpIdx < transferEdges.size(); tpIdx++) {
                            const auto edge = transferEdges[tpIdx];
                            const int transferLoc = edge.edge;
                            const int edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = pickupToTransfersDistances[pickup->loc][transferLoc];
                            // KASSERT(distancePVehToTransfer == asserter.getDistanceBetweenLocations(pickup->loc, transferLoc));

                            // Build the transfer point
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                         distancePVehToTransfer, 0, edge.distToTail + edgeOffset, edge.distFromHead);

                            numTransferPoints++;
                            // Build the assignment
                            AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                            asgn.pickupIdx = numStopsPVeh - 1;
                            asgn.dropoffIdx = dropoff.stopIndex;
                            asgn.transferIdxPVeh = numStopsPVeh - 1;
                            asgn.transferIdxDVeh = i;

                            asgn.pickup = pickup;
                            const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                            asgn.dropoff = dropoffPDLoc;

                            asgn.distToDropoff = dropoff.distToPDLoc;
                            asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;

                            asgn.pickupType = AFTER_LAST_STOP;
                            asgn.dropoffType = ORDINARY;
                            asgn.transferTypePVeh = AFTER_LAST_STOP;
                            asgn.transferTypeDVeh = ORDINARY;

                            // If the pickup or dropoff conincides with the transfer, we skip the assignment
                            if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                continue;

                            finishDistances(asgn, distancePVehToTransfer, distanceToPickup, dropoff.distToPDLoc, 0);
                            
                            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                                // Postpone assignment
                                asgn.distFromTransferDVeh = 0;
                                asgn.distToDropoff = 0;
                                asgn.dropoffPairedLowerBoundUsed = true;
                            }

                            assert(asgn.distFromPickup == 0);
                            assert(asgn.distFromTransferPVeh == 0);
                            assert(asgn.distFromDropoff > 0);

                            // Try the finished assignment with ORD dropoff
                            tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                        }                        
                    }
                }
            }
        }

        void tryDropoffALSForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup,
                                       const RelevantDropoffsAfterLastStop &relALSDropoffs, 
                                       const EdgeEllipseContainer& ellipseContainer) {
            
            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                return;                                        
            
            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().size() == 0)
                return;
            
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                // const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                Timer searchTimer;
                for (const auto dropoffEntry : relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto& dropoff = requestState.dropoffs[dropoffEntry.dropoffId];

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh - 1; i++) {
                        const int stopId = stopIdsDVeh[i];
                        const auto& transferEdges = ellipseContainer.getEdgesInEllipse(stopId);
                        
                        for (int tpIdx = 0; tpIdx < transferEdges.size(); tpIdx++) {
                            const auto edge = transferEdges[tpIdx];
                            const int transferLoc = edge.edge;
                            const int edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = pickupToTransfersDistances[pickup->loc][transferLoc];
                            // KASSERT(distancePVehToTransfer == asserter.getDistanceBetweenLocations(pickup->loc, transferLoc));
                            KASSERT(distancePVehToTransfer > 0 || transferLoc == pickup->loc);
                            
                            // Build the transfer point
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                         distancePVehToTransfer, 0, edge.distToTail + edgeOffset, edge.distFromHead);

                            numTransferPoints++;

                            // Build the assignment
                            AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                            
                            asgn.pickup = pickup;
                            asgn.dropoff = &dropoff;

                            asgn.pickupIdx = numStopsPVeh - 1;
                            asgn.dropoffIdx = numStopsDVeh - 1;
                            asgn.transferIdxPVeh = numStopsPVeh - 1;
                            asgn.transferIdxDVeh = i;
                        
                            const int distanceToDropoff = dropoffEntry.distToDropoff;
                            KASSERT(distanceToDropoff > 0 && distanceToDropoff < INFTY);

                            asgn.pickupType = AFTER_LAST_STOP;
                            asgn.dropoffType = AFTER_LAST_STOP;
                            asgn.transferTypePVeh = AFTER_LAST_STOP;
                            asgn.transferTypeDVeh = ORDINARY;

                            // If the pickup or dropoff conincides with the transfer, we skip the assignment
                            if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                continue;

                            finishDistances(asgn, distancePVehToTransfer, distanceToPickup, 0, distanceToDropoff);
                            assert(asgn.distFromPickup == 0);
                            assert(asgn.distFromTransferPVeh == 0);
                            assert(asgn.distFromDropoff == 0);

                            // Try the finished assignment with ORD dropoff
                            tryFinishedAssignment(asgn);
                        }
                    }
                }
            }
        }

        void tryDropoffORD(const Vehicle *pVeh, const RelevantPDLoc *pickup,
                           std::vector<AssignmentWithTransfer> &postponedAssignments, const EdgeEllipseContainer& ellipseContainer) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
            int distanceToPickup = pickup->distToPDLoc;
            bool bnsLowerBoundUsed = false;
            const auto lastStopLocPVeh = routeState.stopLocationsFor(pVeh->vehicleId)[numStopsPVeh - 1];

            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId,
                                                                                                  pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;

                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
                // const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];

                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    
                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        if (i >= numStopsDVeh - 1)
                            continue;

                        const int stopId = stopIdsDVeh[i];
                        const auto& transferPoints = ellipseContainer.getEdgesInEllipse(stopId);
                        
                        // Loop over all possible transfer points
                        for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {
                            // Build the transfer point
                            const auto edge = transferPoints[tpIdx];
                            const int transferLoc = edge.edge;
                            const auto edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = lastStopToTransfersDistances[lastStopLocPVeh][transferLoc];
                            // assert(distancePVehToTransfer == asserter.getDistanceBetweenLocations(lastStopLocPVeh, transferLoc));
                        
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                         distancePVehToTransfer, 0, edge.distToTail + edgeOffset, edge.distFromHead);
                            
                            // Build the assignment
                            AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                            asgn.pickupIdx = pickup->stopIndex;
                            asgn.dropoffIdx = dropoff.stopIndex;
                            asgn.transferIdxPVeh = numStopsPVeh - 1;
                            asgn.transferIdxDVeh = i;

                            asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;

                            asgn.pickup = pickupPDLoc;
                            asgn.dropoff = dropoffPDLoc;

                            assert(asgn.pickupIdx < asgn.transferIdxPVeh);

                            asgn.distToPickup = distanceToPickup;
                            asgn.distFromPickup = pickup->distFromPDLocToNextStop;
                            asgn.distToDropoff = dropoff.distToPDLoc;
                            asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;

                            asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                            asgn.dropoffType = ORDINARY;
                            asgn.transferTypePVeh = AFTER_LAST_STOP;
                            asgn.transferTypeDVeh = ORDINARY;

                            // If the pickup or dropoff conincides with the transfer, we skip the assignment
                            if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                continue;
                                
                            finishDistances(asgn, 0, distancePVehToTransfer, dropoff.distToPDLoc, 0);

                            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                                asgn.distFromTransferDVeh = 0;
                                asgn.distToDropoff = 0;
                                asgn.dropoffPairedLowerBoundUsed = true;
                            }

                            // Try the assignment with ORD dropoff
                            tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                        }
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup,
                           const RelevantDropoffsAfterLastStop& relALSDropoffs,
                           std::vector<AssignmentWithTransfer>& postponedAssignments,
                           const EdgeEllipseContainer& ellipseContainer
                        ) {
            
            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                return;
            
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const int lastStopLocPVeh = routeState.stopLocationsFor(pVeh->vehicleId)[numStopsPVeh - 1];

            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            bool bnsLowerBoundUsed = false;
            int distanceToPickup = pickup->distToPDLoc;

            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId,
                                                                                                  pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;

                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                // const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);

                for (const auto &dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto& dropoff = requestState.dropoffs[dropoffEntry.dropoffId];
                    
                    for (int i = 1; i < numStopsDVeh - 1; i++) {
                        const int stopId = stopIdsDVeh[i];
                        const auto& transferPoints = ellipseContainer.getEdgesInEllipse(stopId);

                        for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {
                            const auto edge = transferPoints[tpIdx]; 
                            const int transferLoc = edge.edge;
                            const auto edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = lastStopToTransfersDistances[lastStopLocPVeh][transferLoc];
                            // assert(distancePVehToTransfer == asserter.getDistanceBetweenLocations(lastStopLocPVeh, transferLoc));
                            
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                         distancePVehToTransfer, 0, edge.distToTail + edgeOffset, edge.distFromHead);
                            if (tp.loc == dropoff.loc)
                                continue;

                            numTransferPoints++;

                            // Build the assignment
                            AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                            asgn.pickupIdx = pickup->stopIndex;
                            asgn.dropoffIdx = numStopsDVeh - 1;
                            asgn.transferIdxPVeh = numStopsPVeh - 1;
                            asgn.transferIdxDVeh = i;

                            assert(asgn.pickupIdx < numStopsPVeh - 1);

                            asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;

                            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
                            asgn.pickup = pickupPDLoc;
                            asgn.dropoff = &dropoff;

                            asgn.distToPickup = distanceToPickup;
                            asgn.distFromPickup = pickup->distFromPDLocToNextStop;

                            const int distanceToDropoff = dropoffEntry.distToDropoff;
                            assert(distanceToDropoff > 0);

                            asgn.dropoffType = AFTER_LAST_STOP;
                            asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                            asgn.transferTypePVeh = AFTER_LAST_STOP;
                            asgn.transferTypeDVeh = ORDINARY;

                            // If the pickup or dropoff conincides with the transfer, we skip the assignment
                            if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                continue;

                            finishDistances(asgn, 0, distancePVehToTransfer, 0, distanceToDropoff);
                            assert(asgn.distFromTransferPVeh == 0);
                            assert(asgn.distFromDropoff == 0);
                            assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
                            assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

                            // Try the assignment with ALS dropoff
                            tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                        }
                    }
                }
            }
        }

        void finishDistances(AssignmentWithTransfer &asgn, const int pairedDistancePVeh, const int alsDistancePVeh,
                             const int pairedDistanceDVeh, const int alsDistanceDVeh) {
            const int pickupIdx = asgn.pickupIdx;
            const int transferIdxPVeh = asgn.transferIdxPVeh;
            const int transferIdxDVeh = asgn.transferIdxDVeh;
            const int dropoffIdx = asgn.dropoffIdx;

            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);

            const auto schedDepTimesPVeh = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimesPVeh = routeState.schedArrTimesFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesDVeh = routeState.schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimesDVeh = routeState.schedArrTimesFor(asgn.dVeh->vehicleId);

            const int legPickup =
                    pickupIdx < numStopsPVeh - 1 ? schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] : 0;
            const int legTransferPVeh = transferIdxPVeh < numStopsPVeh - 1 ? schedArrTimesPVeh[transferIdxPVeh + 1] -
                                                                             schedDepTimesPVeh[transferIdxPVeh] : 0;
            const int legTransferDVeh = transferIdxDVeh < numStopsDVeh - 1 ? schedArrTimesDVeh[transferIdxDVeh + 1] -
                                                                             schedDepTimesDVeh[transferIdxDVeh] : 0;
            const int legDropoff =
                    dropoffIdx < numStopsDVeh - 1 ? schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx]
                                                  : 0;

            const bool pickupAtStop = asgn.pickup->loc == stopLocationsPVeh[pickupIdx];
            const bool transferAtStopPVeh = asgn.transfer.loc == stopLocationsPVeh[transferIdxPVeh] && asgn.transferIdxPVeh > asgn.pickupIdx;
            const bool transferAtStopDVeh = asgn.transfer.loc == stopLocationsDVeh[transferIdxDVeh];
            const bool dropoffAtStop = asgn.dropoff->loc == stopLocationsDVeh[dropoffIdx] && asgn.dropoffIdx > asgn.transferIdxDVeh;

            const bool pairedPVeh = pickupIdx == transferIdxPVeh;
            const bool pairedDVeh = transferIdxDVeh == dropoffIdx;

            const bool pickupAfterLastStop = pickupIdx == numStopsPVeh - 1;
            const bool transferAfterLastStopPVeh = transferIdxPVeh == numStopsPVeh - 1;
            const bool transferAfterLastStopDVeh = transferIdxDVeh == numStopsDVeh - 1;
            const bool dropoffAfterLastStop = dropoffIdx == numStopsDVeh - 1;

            //* Pickup distances
            // Distance to pickup
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (!pickupAtStop && pickupAfterLastStop)
                asgn.distToPickup = alsDistancePVeh;

            // Distance from pickup
            if (pairedPVeh || pickupAfterLastStop)
                asgn.distFromPickup = 0;

            if (!pairedPVeh && pickupAtStop)
                asgn.distFromPickup = legPickup;

            // Distance to transfer pVeh
            if (pairedPVeh)
                asgn.distToTransferPVeh = pairedDistancePVeh;

            if (!pickupAfterLastStop && transferAfterLastStopPVeh)
                asgn.distToTransferPVeh = alsDistancePVeh;

            if (transferAtStopPVeh)
                asgn.distToTransferPVeh = 0;

            // Distance from transfer pVeh
            if (transferAtStopPVeh)
                asgn.distFromTransferPVeh = legTransferPVeh;

            if (transferAfterLastStopPVeh)
                asgn.distFromTransferPVeh = 0;

            //* Dropoff Distances
            // Distance to transfer dVeh
            if (transferAtStopDVeh)
                asgn.distToTransferDVeh = 0;

            if (!transferAtStopDVeh && transferAfterLastStopDVeh)
                asgn.distToTransferDVeh = alsDistanceDVeh;

            // Distance from transfer dVeh
            if (pairedDVeh || transferAfterLastStopDVeh)
                asgn.distFromTransferDVeh = 0;

            if (!pairedDVeh && transferAtStopDVeh)
                asgn.distFromTransferDVeh = legTransferDVeh;

            // Distance to dropoff
            if (pairedDVeh && !dropoffAfterLastStop)
                asgn.distToDropoff = pairedDistanceDVeh;

            if ((!transferAfterLastStopDVeh && dropoffAfterLastStop) ||
                (transferAfterLastStopDVeh && transferAtStopDVeh))
                asgn.distToDropoff = alsDistanceDVeh;

            if (dropoffAtStop)
                asgn.distToDropoff = 0;

            // Distance from dropoff
            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;

            if (dropoffAfterLastStop)
                asgn.distFromDropoff = 0;

            assert(asgn.distFromDropoff > 0 || dropoffAfterLastStop);
            assert(asgn.distFromTransferPVeh > 0 || transferAfterLastStopPVeh);
            assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
            assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

            assert(asgn.distToPickup > 0 || asgn.distFromPickup > 0 || asgn.distToTransferPVeh > 0 ||
                   asgn.distFromTransferPVeh > 0);
            assert(asgn.distToTransferDVeh > 0 || asgn.distFromTransferDVeh > 0 || asgn.distToDropoff > 0 ||
                   asgn.distFromDropoff > 0);
        }

        // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            return ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                    || (asgn.transferIdxDVeh < numStopsDVeh - 1 &&
                        asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
                    || (asgn.dropoffIdx < numStopsDVeh - 1 &&
                        asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]));
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn) {
            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    numAssignmentsTriedPickupORD++;
                    break;

                case AFTER_LAST_STOP:
                    numAssignmentsTriedPickupALS++;
                    break;
                default:
                    assert(false);
            }

            switch (asgn.dropoffType) {
                case ORDINARY:
                    numAssignmentsTriedDropoffORD++;
                    break;
                case AFTER_LAST_STOP:
                    numAssignmentsTriedDropoffALS++;
                    break;
                default:
                    assert(false);
            }
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
                bestAssignment = asgn;
                bestCost = cost.total;
            }

            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer& asgn,
                                                std::vector<AssignmentWithTransfer>& postponedAssignments) {
                                                
            if (canSkipAssignment(asgn))
                return;
            
            Timer time;
            if (!asgn.isFinished()) {
                const auto lowerBound = calc.calcLowerBound(asgn, requestState);
                if (lowerBound.total >= requestState.getBestCost())
                    return;
                
                postponedAssignments.push_back(asgn);
            } else {
                tryFinishedAssignment(asgn);
            }
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void finishAssignmentsWithPickupBNSLowerBound(const Vehicle *pVeh, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            std::vector<AssignmentWithTransfer> assignmentsLeft;
            std::vector<AssignmentWithTransfer> toCalculate;

            for (const auto &asgn: postponedAssignments) {
                assert(asgn.pickupBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed);
                if (!asgn.pickupBNSLowerBoundUsed) {
                    assignmentsLeft.push_back(asgn);
                    continue;
                }

                toCalculate.push_back(asgn);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            std::vector<AssignmentWithTransfer> assignmentsWithTwoLowerBounds;
            for (auto &asgn: toCalculate) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                tryPotentiallyUnfinishedAssignment(asgn, assignmentsWithTwoLowerBounds);
            }
            
            postponedAssignments.clear();
            
            for (auto& asgn : assignmentsLeft) {
                assert(asgn.dropoffPairedLowerBoundUsed);
                postponedAssignments.push_back(asgn);
            }

            for (auto& asgn : assignmentsWithTwoLowerBounds) {
                assert(asgn.dropoffPairedLowerBoundUsed);
                postponedAssignments.push_back(asgn);
            }
        }

        void finishAssignmentsWithDropoffPairedLowerBound(std::vector<AssignmentWithTransfer>& postponedAssignments) {
            for (auto& asgn: postponedAssignments) {
                KASSERT(asgn.dropoffPairedLowerBoundUsed);
                // Check against the new best cost
                const auto lowerBound = calc.calcLowerBound(asgn, requestState);
                if (lowerBound.total >= requestState.getBestCost())
                    continue;
                
                // Calculate the paired distance
                const int transferLoc = asgn.transfer.loc;
                const int dropoffLoc = asgn.dropoff->loc;

                const int transferRank = vehCh.rank(inputGraph.edgeHead(transferLoc));
                const int dropoffRank  = vehCh.rank(inputGraph.edgeTail(dropoffLoc));
                
                const int offset = inputGraph.travelTime(dropoffLoc);

                vehChQuery.run(transferRank, dropoffRank);
                asgn.distToDropoff = vehChQuery.getDistance() + offset;
                asgn.dropoffPairedLowerBoundUsed = false;

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
        TransfersPickupALSStrategyT &pickupALSStrategy;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator &calc;
        InsertionAsserterT &asserter;

        std::vector<EdgeInEllipse> transferEdges;
        
        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, int>> lastStopToTransfersDistances;
        std::map<int, std::map<int, int>> pickupToTransfersDistances;

        Subset pVehIdsALS;

        AssignmentWithTransfer bestAssignment;
        int bestCost;


        //* Statistics for the transfer als pveh assignment finder
        int64_t totalTime;

        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;
        int64_t numCandidateVehiclesPickupALS;

        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;

        int64_t numPickups;
        int64_t numDropoffs;

        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;
        int64_t numAssignmentsTriedPickupALS;

        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;

        // Stats for the transfer search itself
        int64_t numTransferPoints;

        // Search from last stop to all stops
        int64_t searchTimePickupALS;
        int64_t searchTimeDropoffALS;
        int64_t searchTimePickupToTransfer;
        int64_t searchTimeLastStopToTransfer;
    };
}
