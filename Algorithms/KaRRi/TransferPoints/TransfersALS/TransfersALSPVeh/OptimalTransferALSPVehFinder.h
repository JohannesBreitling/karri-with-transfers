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
            relPVehToInternalIdx(fleet.size(), INVALID_INDEX),
            dVehStopsFlags(fleet.size()),
            isEdgeRel(inputGraph.numEdges()),
            relEdgesToInternalIdx(inputGraph.numEdges()),
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

            numTransferPoints = 0;
        }

        template<typename EllipsesT>
        void findAssignments(const RelevantDropoffsAfterLastStop &relALSDropoffs, const EllipsesT &ellipseContainer) {
            Timer total;
            Timer innerTimer;

            bestCost = INFTY;
            bestAssignment = AssignmentWithTransfer();

            //* Collect the full ellipses of possible transfer points
            // Collect all stop ids of last stops of potential pickup vehicles (without als, as for pickup als we first have to search to the pickups and the from the pickups)

            relPVehToInternalIdx.clear(); // Reset all back to INVALID_INDEX

            std::vector<int> relevantLastStopLocs;

            int numRelPVehs = 0;
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                relPVehToInternalIdx[pVehId] = numRelPVehs++;

                const auto numStops = routeState.numStopsOf(pVehId);
                const auto lastStopLoc = routeState.stopLocationsFor(pVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLoc);
            }

            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (relPVehToInternalIdx[pVehId] != INVALID_INDEX)
                    continue;
                relPVehToInternalIdx[pVehId] = numRelPVehs++;

                const auto numStops = routeState.numStopsOf(pVehId);
                const auto lastStopLocs = routeState.stopLocationsFor(pVehId)[numStops - 1];
                relevantLastStopLocs.push_back(lastStopLocs);
            }

            if (relevantLastStopLocs.empty())
                return;

            if (dVehStopsFlags.size() <= routeState.getMaxStopId())
                dVehStopsFlags.resize(routeState.getMaxStopId() + 1);
            dVehStopsFlags.reset();
            allTransferEdges.clear();
            minDVehCostForTransferEdge.clear();
            isEdgeRel.reset();
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto numStops = routeState.numStopsOf(dVehId);
                if (numStops <= 1)
                    continue;

                const auto &stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < numStops - 1; i++) {
                    const auto stopId = stopIds[i];
                    if (dVehStopsFlags.isSet(stopId))
                        continue;
                    dVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    const int vehWaitTimeFromTransferToEndOfRoute =
                            time_utils::getTotalVehWaitTimeInInterval(dVehId, i, numStops - 1, routeState);
                    for (const auto &e: ellipse) {
                        if (!isEdgeRel.isSet(e.edge)) {
                            relEdgesToInternalIdx[e.edge] = static_cast<int>(allTransferEdges.size());
                            isEdgeRel.set(e.edge);
                            allTransferEdges.push_back(e);
                            minDVehCostForTransferEdge.push_back(INFTY);
                        }
                        auto &minCost = minDVehCostForTransferEdge[relEdgesToInternalIdx[e.edge]];
                        minCost = std::min(minCost,
                                           computeMinDVehCostForTransferEdge(e, dVehId, i, numStops - 1,
                                                                             vehWaitTimeFromTransferToEndOfRoute));
                    }
                }
            }

            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto numStops = routeState.numStopsOf(dVehId);
                if (numStops <= 1)
                    continue;

                const auto &rel = relORDDropoffs.relevantSpotsFor(dVehId);
                const int latestRelevantStopIdx = std::min(numStops - 2, rel[rel.size() - 1].stopIndex);
                const int earliestRelevantStopIdx = rel[0].stopIndex;
                const auto &stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i <= latestRelevantStopIdx; ++i) {
                    const auto stopId = stopIds[i];
                    if (dVehStopsFlags.isSet(stopId))
                        continue;
                    dVehStopsFlags.set(stopId);
                    const auto &ellipse = ellipseContainer.getEdgesInEllipse(stopId);
                    const int vehWaitTimeFromTransferToEndOfRoute =
                            time_utils::getTotalVehWaitTimeInInterval(dVehId, i, numStops - 1, routeState);
                    for (const auto &e: ellipse) {
                        if (!isEdgeRel.isSet(e.edge)) {
                            relEdgesToInternalIdx[e.edge] = static_cast<int>(allTransferEdges.size());
                            isEdgeRel.set(e.edge);
                            allTransferEdges.push_back(e);
                            minDVehCostForTransferEdge.push_back(INFTY);
                        }
                        auto &minCost = minDVehCostForTransferEdge[relEdgesToInternalIdx[e.edge]];
                        minCost = std::min(minCost,
                                           computeMinDVehCostForTransferEdge(e, dVehId, i, earliestRelevantStopIdx,
                                                                             vehWaitTimeFromTransferToEndOfRoute));
                    }
                }
            }

            if (allTransferEdges.empty())
                return;

            const auto initTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Calculate the distances from all last stops (pickup ord, bns) to the potential transfers
            innerTimer.restart();
            const auto lastStopToTransfersDistances = strategy.calculateDistancesFromLastStopToAllTransfers(
                    relevantLastStopLocs, allTransferEdges);
            const int64_t searchTimeLastStopToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            innerTimer.restart();
            const auto pVehIdsALS = pickupALSStrategy.findPickupsAfterLastStop();
            const auto searchTimePickupALS = innerTimer.elapsed<std::chrono::nanoseconds>();

            std::vector<int> pickupLocs;
            for (const auto &pickup: requestState.pickups) {
                pickupLocs.push_back(pickup.loc);
            }

            std::vector<int> dropoffLocs;
            for (const auto &dropoff: requestState.dropoffs) {
                dropoffLocs.push_back(dropoff.loc);
            }

            // Calculate the distances from all pickups to the potential transfers
            innerTimer.restart();
            // pickupToTransfersDistances[i][j] stores the distance from i-th pickup to the j-th edge in transferEdges
            const auto pickupToTransfersDistances = strategy.calculateDistancesFromPickupsToAllTransfers(pickupLocs,
                                                                                                         allTransferEdges);
            const int64_t searchTimePickupToTransfer = innerTimer.elapsed<std::chrono::nanoseconds>();

            // Calculate the distances from all transfers to the dropoffs
            innerTimer.restart();
            // transferToDropoffDistances[i][j] stores the distance from the j-th edge in transferEdges to the i-th dropoff
            const auto transfersToDropoffsDistances = strategy.calculateDistancesFromAllTransfersToDropoffs(
                    allTransferEdges, dropoffLocs);
            const auto searchTimeTransferToDropoff = innerTimer.elapsed<std::chrono::nanoseconds>();

            numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupALS += pVehIdsALS.size();
            numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();

            innerTimer.restart();
            findAssignmentsWithPickupBNS(relALSDropoffs, lastStopToTransfersDistances, transfersToDropoffsDistances,
                                         ellipseContainer);
            findAssignmentsWithPickupORD(relALSDropoffs, lastStopToTransfersDistances, transfersToDropoffsDistances,
                                         ellipseContainer);
            findAssignmentsWithPickupALS(pVehIdsALS, relALSDropoffs, pickupToTransfersDistances,
                                         transfersToDropoffsDistances, ellipseContainer);
            const auto tryAssignmentsTime = innerTimer.elapsed<std::chrono::nanoseconds>();

            KASSERT(bestCost >= INFTY || asserter.assertAssignment(bestAssignment));

            // Write the stats
            auto &stats = requestState.stats().transferALSPVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.initTime = initTime;

            stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            stats.numCandidateVehiclesPickupALS += numCandidateVehiclesPickupALS;
            stats.numCandidateVehiclesDropoffORD += numCandidateVehiclesDropoffORD;
            stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;

            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedPickupALS += numAssignmentsTriedPickupALS;
            stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;

            stats.tryAssignmentsTime += tryAssignmentsTime;

            stats.numTransferPoints += numTransferPoints;

            stats.searchTimePickupALS += searchTimePickupALS;
            stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            stats.searchTimePickupToTransfer += searchTimePickupToTransfer;
            stats.searchTimeTransferToDropoff += searchTimeTransferToDropoff;
        }

    private:

        int computeMinDVehCostForTransferEdge(const EdgeInEllipse &e, const int vehId,
                                              const int transferIdx,
                                              const int earliestDropoffIdx,
                                              const int vehWaitTimeFromTransferToEndOfRoute) const {

            using namespace time_utils;
            const int minTransferDetour =
                    e.distToTail + inputGraph.travelTime(e.edge) + InputConfig::getInstance().stopTime +
                    e.distFromHead -
                    calcLengthOfLegStartingAt(transferIdx, vehId, routeState);
            KASSERT(minTransferDetour >= 0);
            const int minResDetour = std::max(0, minTransferDetour - vehWaitTimeFromTransferToEndOfRoute);
            int minTripTime = transferIdx == earliestDropoffIdx ? 0 : e.distFromHead;
            int minAddedTripTime = 0;
            if (transferIdx < earliestDropoffIdx) {
                const int minArrStopBeforeDropoff =
                        routeState.schedArrTimesFor(vehId)[earliestDropoffIdx] + minResDetour;
                const int maxDepStopAfterTransfer =
                        routeState.schedDepTimesFor(vehId)[transferIdx + 1] + minTransferDetour;
                minTripTime += std::max(0, minArrStopBeforeDropoff - maxDepStopAfterTransfer);
                minAddedTripTime += calcAddedTripTimeInInterval(vehId, transferIdx, earliestDropoffIdx,
                                                                minTransferDetour, routeState);
            }

            using F = CostCalculator::CostFunction;
            return F::calcVehicleCost(minResDetour) + F::calcTripCost(minTripTime, requestState) +
                   F::calcChangeInTripCostsOfExistingPassengers(minAddedTripTime);

        }


        void findAssignmentsWithPickupORD(const RelevantDropoffsAfterLastStop &relALSDropoffs,
                                          const auto &lastStopToTransfersDistances,
                                          const auto &transfersToDropoffsDistances,
                                          const EdgeEllipseContainer &ellipseContainer) {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Loop over all possible vehicles and pickups
            std::vector<AssignmentWithTransfer> placeholder;
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const auto *pVeh = &fleet[pVehId];
                const auto &thisLastStopToTransfersDistances = lastStopToTransfersDistances.getDistancesFor(
                        relPVehToInternalIdx[pVehId]);

                for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, placeholder, thisLastStopToTransfersDistances,
                                  transfersToDropoffsDistances, ellipseContainer);
                    tryDropoffALS(pVeh, &pickup, relALSDropoffs, placeholder, thisLastStopToTransfersDistances,
                                  ellipseContainer);
                }
            }
            KASSERT(placeholder.empty());
        }

        void findAssignmentsWithPickupBNS(const RelevantDropoffsAfterLastStop &relALSDropoffs,
                                          const auto &lastStopToTransfersDistances,
                                          const auto &transfersToDropoffsDistances,
                                          const EdgeEllipseContainer &ellipseContainer) {
            //* In this case we consider all vehicles that are able to perform the pickup BNS
            if (relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Loop over all possible vehicles and pickups
            std::vector<AssignmentWithTransfer> postponedAssignments;
            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                postponedAssignments.clear();
                auto *pVeh = &fleet[pVehId];
                const auto &thisLastStopToTransfersDistances = lastStopToTransfersDistances.getDistancesFor(
                        relPVehToInternalIdx[pVehId]);

                for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, postponedAssignments, thisLastStopToTransfersDistances,
                                  transfersToDropoffsDistances, ellipseContainer);
                    tryDropoffALS(pVeh, &pickup, relALSDropoffs, postponedAssignments, thisLastStopToTransfersDistances,
                                  ellipseContainer);
                }

                finishAssignmentsWithPickupBNSLowerBound(pVeh, postponedAssignments);
            }
        }


        void findAssignmentsWithPickupALS(const Subset &pVehIdsALS,
                                          const RelevantDropoffsAfterLastStop &relALSDropoffs,
                                          const auto &pickupToTransfersDistances,
                                          const auto &transfersToDropoffsDistances,
                                          const EdgeEllipseContainer &ellipseContainer) {

            using F = CostCalculator::CostFunction;
            std::vector<int> minCostFromDepAtPickup(requestState.numPickups(), INFTY);
            for (const auto &pickup: requestState.pickups) {
                const auto &thisPickupToTransfersDistances = pickupToTransfersDistances.getDistancesFor(pickup.id);
                auto &minCost = minCostFromDepAtPickup[pickup.id];
                for (const auto &e: allTransferEdges) {
                    const int distToTransfer = thisPickupToTransfersDistances[relEdgesToInternalIdx[e.edge]];
                    const int minCostTransfer = minDVehCostForTransferEdge[relEdgesToInternalIdx[e.edge]] +
                                                F::calcVehicleCost(distToTransfer) +
                                                F::calcTripCost(distToTransfer, requestState);
                    minCost = std::min(minCost, minCostTransfer);
                }
            }

            for (const auto pVehId: pVehIdsALS) {
                const auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: requestState.pickups) {
                    // Get the distance from the last stop of the pVeh to the pickup
                    const int distanceToPickup = pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                    if (distanceToPickup >= INFTY)
                        continue; // Pickup is not reachable

                    const int minDistToTransfer = pickupToTransfersDistances.getMinDistanceFor(pickup.id);
                    if (minDistToTransfer >= INFTY)
                        continue; // No transfer is reachable from this pickup

                    // Prune if lower bound for cost of pickup vehicle with this pickup is worse than best known cost
                    using namespace time_utils;

                    const auto stopIdx = routeState.numStopsOf(pVehId) - 1;
                    const auto depAtPickup = getActualDepTimeAtPickup(pVehId, stopIdx, distanceToPickup,
                                                                      pickup, requestState, routeState);
                    const auto vehTimeTillDepAtPickup =
                            depAtPickup - getVehDepTimeAtStopForRequest(pVehId, stopIdx, requestState, routeState);
                    const auto psgTimeTillDepAtPickup = depAtPickup - requestState.originalRequest.requestTime;
                    const int lowerBoundCost =
                            minCostFromDepAtPickup[pickup.id] + F::calcVehicleCost(vehTimeTillDepAtPickup) +
                            F::calcTripCost(psgTimeTillDepAtPickup, requestState);
                    if (lowerBoundCost > requestState.getBestCost()) {
                        continue;
                    }

                    // KASSERT(asserter.assertLastStopDistance(pVehId, pickup.loc) == distanceToPickup);
                    KASSERT(!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty() ||
                            !relORDDropoffs.getVehiclesWithRelevantPDLocs().empty());
                    const auto &thisPickupToTransfersDistances = pickupToTransfersDistances.getDistancesFor(pickup.id);
                    KASSERT(minDistToTransfer == *std::min_element(thisPickupToTransfersDistances.begin(),
                                                                   thisPickupToTransfersDistances.end()));


                    tryDropoffORDForPickupALS(pVeh, &pickup, distanceToPickup, thisPickupToTransfersDistances,
                                              transfersToDropoffsDistances, ellipseContainer);
                    tryDropoffALSForPickupALS(pVeh, &pickup, distanceToPickup, thisPickupToTransfersDistances,
                                              relALSDropoffs, ellipseContainer);
                }
            }
        }


        void tryDropoffORDForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup,
                                       const auto &thisPickupToTransfersDistances,
                                       const auto &transfersToDropoffsDistances,
                                       const EdgeEllipseContainer &ellipseContainer) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                // const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
                const auto schedDepTimesDVeh = routeState.schedDepTimesFor(dVehId);
                const auto schedArrTimesDVeh = routeState.schedArrTimesFor(dVehId);
                const auto occupanciesDVeh = routeState.occupanciesFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                for (const auto &relDropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    if (relDropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    const auto *dropoff = &requestState.dropoffs[relDropoff.pdId];
                    const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                            relDropoff.pdId);

                    // Compute part of cost lower bound that depends only on the dropoff
                    using namespace time_utils;
                    const bool dropoffAtExistingStop = isDropoffAtExistingStop(dVehId, INVALID_INDEX, relDropoff.stopIndex,
                                                                               dropoff->loc, routeState);
                    const int minInitialDropoffDetour =
                            calcInitialDropoffDetour(dVehId, relDropoff.stopIndex, relDropoff.distToPDLoc,
                                                     relDropoff.distFromPDLocToNextStop,
                                                     dropoffAtExistingStop, routeState);
                    const int minDropoffCostWithoutTrip =
                            calc.calcMinKnownDropoffSideCostWithoutTripTime(*dVeh, relDropoff.stopIndex,
                                                                            minInitialDropoffDetour,
                                                                            dropoff->walkingDist, requestState);

                    for (int i = relDropoff.stopIndex; i > 0; i--) {
                        if (i >= numStopsDVeh - 1)
                            continue;

                        // If occupancy exceeds capacity at this leg, we do not have to consider this or earlier stops
                        // of the route, as this leg needs to be traversed.
                        if (occupanciesDVeh[i] + requestState.originalRequest.numRiders > dVeh->capacity)
                            break;

                        // Compute lower bound for costs with transfer between i and i + 1, considering trip time
                        // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
                        // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
                        const auto minTripTimeToStopBeforeDropoff =
                                schedArrTimesDVeh[relDropoff.stopIndex] - schedDepTimesDVeh[i + 1];
                        const int minTripTimeToDropoff =
                                std::max(requestState.minDirectPDDist, minTripTimeToStopBeforeDropoff +
                                        relDropoff.distToPDLoc) + dropoff->walkingDist;
                        const int minCostFromHere =
                                CostCalculator::CostFunction::calcTripCost(minTripTimeToDropoff, requestState) +
                                minDropoffCostWithoutTrip;
                        if (minCostFromHere > requestState.getBestCost())
                            break;

                        const int stopId = stopIdsDVeh[i];
                        const auto &transferEdges = ellipseContainer.getEdgesInEllipse(stopId);

                        for (int tpIdx = 0; tpIdx < transferEdges.size(); tpIdx++) {
                            const auto edge = transferEdges[tpIdx];
                            const int transferLoc = edge.edge;
                            KASSERT(isEdgeRel.isSet(transferLoc));
                            const int edgeOffset = inputGraph.travelTime(transferLoc);

                            if (minDVehCostForTransferEdge[relEdgesToInternalIdx[transferLoc]] >=
                                requestState.getBestCost())
                                continue;

                            const int distancePVehToTransfer = thisPickupToTransfersDistances[relEdgesToInternalIdx[transferLoc]];
                            // KASSERT(distancePVehToTransfer == asserter.getDistanceBetweenLocations(pickup->loc, transferLoc));

                            // If the pickup or dropoff conincides with the transfer, we skip the assignment
                            if (pickup->loc == transferLoc || transferLoc == dropoff->loc)
                                continue;

                            // Build the transfer point
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                             distancePVehToTransfer, 0, edge.distToTail + edgeOffset,
                                                             edge.distFromHead);

                            numTransferPoints++;
                            // Build the assignment
                            AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                            asgn.pickupIdx = numStopsPVeh - 1;
                            asgn.dropoffIdx = relDropoff.stopIndex;
                            asgn.transferIdxPVeh = numStopsPVeh - 1;
                            asgn.transferIdxDVeh = i;

                            asgn.pickup = pickup;
                            asgn.dropoff = dropoff;

                            asgn.distToDropoff = relDropoff.distToPDLoc;
                            asgn.distFromDropoff = relDropoff.distFromPDLocToNextStop;

                            asgn.pickupType = AFTER_LAST_STOP;
                            asgn.dropoffType = ORDINARY;
                            asgn.transferTypePVeh = AFTER_LAST_STOP;
                            asgn.transferTypeDVeh = ORDINARY;


                            finishDistances(asgn, distancePVehToTransfer, distanceToPickup, relDropoff.distToPDLoc, 0);

                            // If transfer and dropoff are paired, vehicle drives directly from transfer to dropoff.
                            // Set according distances.
                            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                                asgn.distFromTransferDVeh = 0;
                                asgn.distToDropoff = transfersToThisDropoffDistances[relEdgesToInternalIdx[transferLoc]];
                            }

                            assert(asgn.distFromPickup == 0);
                            assert(asgn.distFromTransferPVeh == 0);
                            assert(asgn.distFromDropoff > 0);

                            // Try the finished assignment with ORD dropoff
                            tryFinishedAssignment(asgn);
                        }
                    }
                }
            }
        }

        void tryDropoffALSForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup,
                                       const auto &thisPickupToTransfersDistances,
                                       const RelevantDropoffsAfterLastStop &relALSDropoffs,
                                       const EdgeEllipseContainer &ellipseContainer) {

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
                const auto schedDepTimesDVeh = routeState.schedDepTimesFor(dVehId);
                const auto occupanciesDVeh = routeState.occupanciesFor(dVehId);
                const auto schedArrTimeAtLastStopDVeh = routeState.schedArrTimesFor(dVehId)[numStopsDVeh - 1];
                const auto lastStopLoc = routeState.stopLocationsFor(dVehId)[numStopsDVeh - 1];

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                for (const auto dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto &dropoff = requestState.dropoffs[dropoffEntry.dropoffId];

                    using namespace time_utils;
                    const bool dropoffAtExistingStop = dropoff.loc == lastStopLoc;
                    const int minInitialDropoffDetour = calcInitialDropoffDetour(dVehId, numStopsDVeh - 1,
                                                                                 numStopsDVeh - 1, 0,
                                                                                 dropoffAtExistingStop, routeState);
                    const int minDropoffCostWithoutTrip =
                            calc.calcMinKnownDropoffSideCostWithoutTripTime(*dVeh, numStopsDVeh - 1,
                                                                            minInitialDropoffDetour,
                                                                            dropoff.walkingDist, requestState);

                    // Try all possible transfer points
                    for (int i = numStopsDVeh - 2; i >= 1; --i) {

                        // If occupancy exceeds capacity at this leg, we do not have to consider this or earlier stops
                        // of the route, as this leg needs to be traversed.
                        if (occupanciesDVeh[i] + requestState.originalRequest.numRiders > dVeh->capacity)
                            break;

                        // Compute lower bound for costs with transfer between i and i + 1, considering trip time
                        // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
                        // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
                        const auto minTripTimeToLastStop = schedArrTimeAtLastStopDVeh - schedDepTimesDVeh[i + 1];
                        const int minTripTimeToDropoff = std::max(requestState.minDirectPDDist,
                                                                  minTripTimeToLastStop + dropoffEntry.distToDropoff) +
                                                         dropoff.walkingDist;
                        const int minCostFromHere =
                                CostCalculator::CostFunction::calcTripCost(minTripTimeToDropoff, requestState) +
                                minDropoffCostWithoutTrip;
                        if (minCostFromHere > requestState.getBestCost())
                            break;

                        const int stopId = stopIdsDVeh[i];
                        const auto &transferEdges = ellipseContainer.getEdgesInEllipse(stopId);

                        for (int tpIdx = 0; tpIdx < transferEdges.size(); tpIdx++) {
                            const auto edge = transferEdges[tpIdx];
                            const int transferLoc = edge.edge;
                            KASSERT(isEdgeRel.isSet(transferLoc));

                            if (minDVehCostForTransferEdge[relEdgesToInternalIdx[transferLoc]] >=
                                requestState.getBestCost())
                                continue;

                            const int edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = thisPickupToTransfersDistances[relEdgesToInternalIdx[transferLoc]];
                            // KASSERT(distancePVehToTransfer == asserter.getDistanceBetweenLocations(pickup->loc, transferLoc));
                            KASSERT(distancePVehToTransfer > 0 || transferLoc == pickup->loc);

                            // Build the transfer point
                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                             distancePVehToTransfer, 0, edge.distToTail + edgeOffset,
                                                             edge.distFromHead);

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
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const auto &thisLastStopToTransfersDistances,
                           const auto &transfersToDropoffsDistances,
                           const EdgeEllipseContainer &ellipseContainer) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
            int distanceToPickup = pickup->distToPDLoc;
            bool bnsLowerBoundUsed = false;

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
                const auto schedDepTimesDVeh = routeState.schedDepTimesFor(dVehId);
                const auto schedArrTimesDVeh = routeState.schedArrTimesFor(dVehId);
                const auto occupanciesDVeh = routeState.occupanciesFor(dVehId);

                for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                    const auto &transfersToThisDropoffDistances = transfersToDropoffsDistances.getDistancesFor(
                            dropoff.pdId);

                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    // Compute part of cost lower bound that depends only on the dropoff
                    using namespace time_utils;
                    const bool dropoffAtExistingStop = isDropoffAtExistingStop(dVehId, INVALID_INDEX, dropoff.stopIndex,
                                                                               dropoffPDLoc->loc, routeState);
                    const int minInitialDropoffDetour =
                            calcInitialDropoffDetour(dVehId, dropoff.stopIndex, dropoff.distToPDLoc,
                                                     dropoff.distFromPDLocToNextStop,
                                                     dropoffAtExistingStop, routeState);
                    const int minDropoffCostWithoutTrip =
                            calc.calcMinKnownDropoffSideCostWithoutTripTime(*dVeh, dropoff.stopIndex,
                                                                            minInitialDropoffDetour,
                                                                            dropoffPDLoc->walkingDist, requestState);

                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        if (i >= numStopsDVeh - 1)
                            continue;

                        // If occupancy exceeds capacity at this leg, we do not have to consider this or earlier stops
                        // of the route, as this leg needs to be traversed.
                        if (occupanciesDVeh[i] + requestState.originalRequest.numRiders > dVeh->capacity)
                            break;

                        // Compute lower bound for costs with transfer between i and i + 1, considering trip time
                        // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
                        // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
                        const auto minTripTimeToStopBeforeDropoff =
                                schedArrTimesDVeh[dropoff.stopIndex] - schedDepTimesDVeh[i + 1];
                        const int minTripTimeToDropoff =
                                std::max(requestState.minDirectPDDist, minTripTimeToStopBeforeDropoff +
                                                                       dropoff.distToPDLoc) + dropoffPDLoc->walkingDist;
                        const int minCostFromHere =
                                CostCalculator::CostFunction::calcTripCost(minTripTimeToDropoff, requestState) +
                                        minDropoffCostWithoutTrip;
                        if (minCostFromHere > requestState.getBestCost())
                            break;

                        const int stopId = stopIdsDVeh[i];
                        const auto &transferPoints = ellipseContainer.getEdgesInEllipse(stopId);

                        // Loop over all possible transfer points
                        for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {
                            // Build the transfer point
                            const auto edge = transferPoints[tpIdx];
                            const int transferLoc = edge.edge;
                            KASSERT(isEdgeRel.isSet(transferLoc));
                            const auto edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = thisLastStopToTransfersDistances[relEdgesToInternalIdx[transferLoc]];
                            // assert(distancePVehToTransfer == asserter.getDistanceBetweenLocations(lastStopLocPVeh, transferLoc));

                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                             distancePVehToTransfer, 0, edge.distToTail + edgeOffset,
                                                             edge.distFromHead);

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

                            // If the pickup or dropoff coincides with the transfer, we skip the assignment
                            if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                continue;

                            finishDistances(asgn, 0, distancePVehToTransfer, dropoff.distToPDLoc, 0);

                            // If transfer and dropoff are paired, vehicle drives directly from transfer to dropoff.
                            // Set according distances.
                            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                                asgn.distFromTransferDVeh = 0;
                                asgn.distToDropoff = transfersToThisDropoffDistances[relEdgesToInternalIdx[transferLoc]];
                            }

                            // Try the assignment with ORD dropoff
                            tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                        }
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup,
                           const RelevantDropoffsAfterLastStop &relALSDropoffs,
                           std::vector<AssignmentWithTransfer> &postponedAssignments,
                           const auto &thisLastStopToTransfersDistances,
                           const EdgeEllipseContainer &ellipseContainer
        ) {

            if (relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                return;

            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

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
                const auto schedDepTimesDVeh = routeState.schedDepTimesFor(dVehId);
                const auto occupanciesDVeh = routeState.occupanciesFor(dVehId);
                const auto schedArrTimeAtLastStopDVeh = routeState.schedArrTimesFor(dVehId)[numStopsDVeh - 1];
                const auto lastStopLoc = routeState.stopLocationsFor(dVehId)[numStopsDVeh - 1];

                for (const auto &dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto &dropoff = requestState.dropoffs[dropoffEntry.dropoffId];

                    using namespace time_utils;
                    const bool dropoffAtExistingStop = dropoff.loc == lastStopLoc;
                    const int minInitialDropoffDetour = calcInitialDropoffDetour(dVehId, numStopsDVeh - 1,
                                                                                 numStopsDVeh - 1, 0,
                                                                                 dropoffAtExistingStop, routeState);
                    const int minDropoffCostWithoutTrip =
                            calc.calcMinKnownDropoffSideCostWithoutTripTime(*dVeh, numStopsDVeh - 1,
                                                                            minInitialDropoffDetour,
                                                                            dropoff.walkingDist, requestState);

                    for (int i = numStopsDVeh - 2; i >= 1; --i) {

                        // If occupancy exceeds capacity at this leg, we do not have to consider this or earlier stops
                        // of the route, as this leg needs to be traversed.
                        if (occupanciesDVeh[i] + requestState.originalRequest.numRiders > dVeh->capacity)
                            break;

                        // Compute lower bound for costs with transfer between i and i + 1, considering trip time
                        // from stop i + 1 to the dropoff and detour to the dropoff. This lower bound also holds for
                        // any stop before i so if it is worse than the best known cost, we can stop for this vehicle.
                        const auto minTripTimeToLastStop = schedArrTimeAtLastStopDVeh - schedDepTimesDVeh[i + 1];
                        const int minTripTimeToDropoff = std::max(requestState.minDirectPDDist,
                                                                  minTripTimeToLastStop + dropoffEntry.distToDropoff) +
                                                         dropoff.walkingDist;
                        const int minCostFromHere =
                                CostCalculator::CostFunction::calcTripCost(minTripTimeToDropoff, requestState) +
                                        minDropoffCostWithoutTrip;
                        if (minCostFromHere > requestState.getBestCost())
                            break;

                        const int stopId = stopIdsDVeh[i];
                        const auto &transferPoints = ellipseContainer.getEdgesInEllipse(stopId);

                        for (int tpIdx = 0; tpIdx < transferPoints.size(); tpIdx++) {
                            const auto edge = transferPoints[tpIdx];
                            const int transferLoc = edge.edge;
                            KASSERT(isEdgeRel.isSet(transferLoc));
                            const auto edgeOffset = inputGraph.travelTime(transferLoc);
                            const int distancePVehToTransfer = thisLastStopToTransfersDistances[relEdgesToInternalIdx[transferLoc]];
                            // assert(distancePVehToTransfer == asserter.getDistanceBetweenLocations(lastStopLocPVeh, transferLoc));

                            TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                             distancePVehToTransfer, 0, edge.distToTail + edgeOffset,
                                                             edge.distFromHead);
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

                            // These assertions do not work due to edges with travel time 0
//                            assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
//                            assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

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
            const bool transferAtStopPVeh =
                    asgn.transfer.loc == stopLocationsPVeh[transferIdxPVeh] && asgn.transferIdxPVeh > asgn.pickupIdx;
            const bool transferAtStopDVeh = asgn.transfer.loc == stopLocationsDVeh[transferIdxDVeh];
            const bool dropoffAtStop =
                    asgn.dropoff->loc == stopLocationsDVeh[dropoffIdx] && asgn.dropoffIdx > asgn.transferIdxDVeh;

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

            if (!pairedDVeh && transferAtStopDVeh) {
                KASSERT(legTransferDVeh > 0);
                asgn.distFromTransferDVeh = legTransferDVeh;
            }

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

            // These assertions do not work due to edges with travel time 0
//            KASSERT(asgn.distFromDropoff > 0 || dropoffAfterLastStop);
//            KASSERT(asgn.distFromTransferPVeh > 0 || transferAfterLastStopPVeh);
//            KASSERT(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
//            KASSERT(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);
//            KASSERT(asgn.distToPickup > 0 || asgn.distFromPickup > 0 || asgn.distToTransferPVeh > 0 ||
//                   asgn.distFromTransferPVeh > 0);
//            KASSERT(asgn.distToTransferDVeh > 0 || asgn.distFromTransferDVeh > 0 || asgn.distToDropoff > 0 ||
//                   asgn.distFromDropoff > 0);
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
            const auto cost = calc.calc(asgn, requestState);
            requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            if (cost.total < bestCost) {
                bestAssignment = asgn;
                bestCost = cost.total;
            }
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer &asgn,
                                                std::vector<AssignmentWithTransfer> &postponedAssignments) {

            if (canSkipAssignment(asgn))
                return;

            if (!asgn.isFinished()) {
                const auto lowerBound = calc.calcLowerBound(asgn, requestState);
                if (lowerBound.total >= requestState.getBestCost())
                    return;

                postponedAssignments.push_back(asgn);
            } else {
                tryFinishedAssignment(asgn);
            }
        }

        void finishAssignmentsWithPickupBNSLowerBound(const Vehicle *pVeh,
                                                      std::vector<AssignmentWithTransfer> &postponedAssignments) {

            if (postponedAssignments.empty())
                return;

            for (const auto &asgn: postponedAssignments) {
                KASSERT(asgn.pickupBNSLowerBoundUsed && !asgn.dropoffPairedLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            for (auto &asgn: postponedAssignments) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                KASSERT(asgn.isFinished());
                tryFinishedAssignment(asgn);
            }
        }

        InputGraphT inputGraph;

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const CH &vehCh;

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

        std::vector<EdgeInEllipse> allTransferEdges;

        // Lower bound on part of cost of assignment using transfer edge for the dropoff vehicle including detour and
        // trip time starting from transfer.
        std::vector<int> minDVehCostForTransferEdge;

        TimestampedVector<int> relPVehToInternalIdx; // Maps vehicle IDs of relevant pVehs to consecutive indices
        FastResetFlagArray<> dVehStopsFlags; // Helper to deduplicate stops of dropoff vehicles
        FastResetFlagArray<> isEdgeRel; // Helper structure to deduplicate transfer edges
        std::vector<int> relEdgesToInternalIdx; // Maps transfer edges to consecutive indices

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

        // Stats for the transfer search itself
        int64_t numTransferPoints;
    };
}
