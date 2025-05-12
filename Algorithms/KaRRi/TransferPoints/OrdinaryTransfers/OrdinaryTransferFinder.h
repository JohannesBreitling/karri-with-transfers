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

#include <map>

#include "Algorithms/KaRRi/TransferPoints/VertexInEllipse.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Containers/LightweightSubset.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Tools/Timer.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Containers/FastResetFlagArray.h"

#pragma once

namespace karri {

    template<
            typename InputGraphT,
            typename VehCHEnvT,
            typename CurVehLocToPickupSearchesT,
            typename TransferPointStrategyT,
            typename TransfersDropoffALSStrategyT,
            typename InsertionAsserterT,
            typename DirectTransferDistancesFinderT>
    class OrdinaryTransferFinder {

        using VehCHQueryLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<VehCHQueryLabelSet>;


        // Both vehicles can drive a detour to the transfer point, but none drives the detour ALS
        // This implies, that the pickup is BNS or ORD, the dropoff can be BNS, ORD or ALS
        // If the dropoff is ALS, we need to consider a different set of vehicles to calculate the transfer psints between
    public:

        OrdinaryTransferFinder(
                const InputGraphT &inputGraph,
                const VehCHEnvT &vehChEnv,
                CurVehLocToPickupSearchesT &searches,
                TransferPointStrategyT &transferPointStrategy,
                TransfersDropoffALSStrategyT &dropoffALSStrategy,
                const RelevantPDLocs &relORDPickups,
                const RelevantPDLocs &relBNSPickups,
                const RelevantPDLocs &relORDDropoffs,
                const RelevantPDLocs &relBNSDropoffs,
                DirectTransferDistancesFinderT &pickupToTransferDistancesFinder,
                DirectTransferDistancesFinderT &transferToDropoffDistancesFinder,
                std::vector<AssignmentWithTransfer> &postponedAssignments,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                CostCalculator &calc,
                InsertionAsserterT &asserter
        ) :
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<VehCHQueryLabelSet>()),
                searches(searches),
                transferPointStrategy(transferPointStrategy),
                dropoffALSStrategy(dropoffALSStrategy),
                relORDPickups(relORDPickups),
                relBNSPickups(relBNSPickups),
                relORDDropoffs(relORDDropoffs),
                relBNSDropoffs(relBNSDropoffs),
                pickupToTransferDistancesFinder(pickupToTransferDistancesFinder),
                transferToDropoffDistancesFinder(transferToDropoffDistancesFinder),
                postponedAssignments(postponedAssignments),
                fleet(fleet),
                routeState(routeState),
                requestState(requestState),
                calc(calc),
                asserter(asserter),
                pVehs(fleet.size()),
                dVehs(fleet.size()),
                dVehIdsALS(Subset(0)),
                edgesSubset(inputGraph.numEdges()),
                stopSeen(fleet.size()),
                ellipsesLogger(LogManager<std::ofstream>::getLogger("ellipses.csv",
                                                                    "size\n")),
                ellipseIntersectionLogger(LogManager<std::ofstream>::getLogger("ellipse-intersection.csv",
                                                                               "size\n")) {}


        void init() {
            totalTime = 0;
            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesDropoffBNS = 0;
            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;
            numPartialsTriedPickupBNS = 0;
            numPartialsTriedPickupORD = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffBNS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;
            numStopPairs = 0;
            numTransferPoints = 0;
            numDijkstraSearchesRun = 0;
            numEdgesRelaxed = 0;
            numVerticesScanned = 0;
            searchTime = 0;
        }

        void findAssignments() {
            Timer total;
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 &&
                relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0)
                return;

            numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffBNS += relBNSDropoffs.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();

            promisingPartials.clear();

            // Construct the set of possible pickup and dropoff vehicles
            constructPVehSet();
            constructDVehSet();

//            // Construct set of pairs of stop pairs for which a transfer may be possible.
//            // TODO: Use for parallelization in the future
//            struct PairOfStopPairs {
//                int firstStopIdPVeh = INVALID_ID;
//                int firstStopIdDVeh = INVALID_ID;
//            };
//            std::vector<PairOfStopPairs> pairsOfStopPairs;
//            for (const auto pVehId: pVehs) {
//                const auto earliestRelevantStopIdxPVeh = getEarliestRelevantStopIdxForPVeh(pVehId);
//                const auto stopIdsPVeh = routeState.stopIdsFor(pVehId);
//                const auto numStopsPVeh = routeState.numStopsOf(pVehId);
//                for (const auto dVehId : dVehs) {
//                    if (pVehId == dVehId) {
//                        continue;
//                    }
//                    const auto latestRelevantStopIdxDVeh = getLatestRelevantStopIdxForDVeh(dVehId);
//                    const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
//
//                    for (int trIdxPVeh = earliestRelevantStopIdxPVeh; trIdxPVeh < numStopsPVeh - 1; ++trIdxPVeh) {
//                        for (int trIdxDVeh = 0; trIdxDVeh < latestRelevantStopIdxDVeh + 1; ++trIdxDVeh) {
//                            const auto stopIdPVeh = stopIdsPVeh[trIdxPVeh];
//                            const auto stopIdDVeh = stopIdsDVeh[trIdxDVeh];
//                            pairsOfStopPairs.emplace_back(stopIdPVeh, stopIdDVeh);
//                        }
//                    }
//                }
//            }

            // Calculate the necessary ellipses for every vehicle for pickup and dropoff
            if (routeState.getMaxStopId() + 1 > stopSeen.size())
                stopSeen.resize(routeState.getMaxStopId() + 1);
            stopSeen.reset();
            std::vector<int> pVehStopIds;
            for (const auto pVehId: pVehs) {
                const int earliestRelevantStopIdx = getEarliestRelevantStopIdxForPVeh(pVehId);
                const auto stopIds = routeState.stopIdsFor(pVehId);
                for (int i = earliestRelevantStopIdx; i < routeState.numStopsOf(pVehId) - 1; i++) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        pVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            stopSeen.reset();
            std::vector<int> dVehStopIds;
            for (const auto dVehId: dVehs) {
                const int latestRelevantStopIdx = getLatestRelevantStopIdxForDVeh(dVehId);
                KASSERT(latestRelevantStopIdx + 1 <= routeState.numStopsOf(dVehId) - 1);
                const auto stopIds = routeState.stopIdsFor(dVehId);
                for (int i = 0; i < latestRelevantStopIdx + 1; i++) {
                    if (!stopSeen.isSet(stopIds[i])) {
                        stopSeen.set(stopIds[i]);
                        dVehStopIds.push_back(stopIds[i]);
                    }
                }
            }

            // Calculate transfer points
            Timer searchTimer;
            transferPointStrategy.computeTransferPoints(pVehStopIds, dVehStopIds);
            searchTime += searchTimer.elapsed<std::chrono::nanoseconds>();

            // Run selection phase for many-to-many searches used to find distances from pickups to transfers and from
            // transfers to dropoffs.
            std::vector<int> pdLocRanks;
            std::vector<int> pdLocOffsets;
            pdLocRanks.reserve(requestState.numPickups());
            pdLocOffsets.reserve(requestState.numPickups());
            for (const auto &p: requestState.pickups) {
                pdLocRanks.push_back(vehCh.rank(inputGraph.edgeHead(p.loc)));
                pdLocOffsets.push_back(0);
            }
            pickupToTransferDistancesFinder.runSelectionForPdLocs(pdLocRanks, pdLocOffsets);

            pdLocRanks.clear();
            pdLocOffsets.clear();
            pdLocRanks.reserve(requestState.numDropoffs());
            pdLocOffsets.reserve(requestState.numDropoffs());
            for (const auto &d: requestState.dropoffs) {
                pdLocRanks.push_back(vehCh.rank(inputGraph.edgeTail(d.loc)));
                pdLocOffsets.push_back(inputGraph.travelTime(d.loc));
            }
            transferToDropoffDistancesFinder.runSelectionForPdLocs(pdLocRanks, pdLocOffsets);



            // Loop over all the possible vehicle combinations
            for (const auto pVehId: pVehs) {
                for (const auto dVehId: dVehs) {

                    // pVeh and dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;

                    // Find the assignments with the transfer points
                    findAssignmentsForVehiclePair(pVehId, dVehId);

                    if (promisingPartials.empty())
                        continue;

                    for (auto &asgn: promisingPartials) {
                        tryDropoffBNS(asgn);
                        tryDropoffORD(asgn);
                        tryDropoffALS(asgn);
                    }

                    promisingPartials.clear();

                    if (postponedAssignments.empty())
                        continue;

                    // Finish the postponed assignments
                    finishAssignments(pVehId, dVehId);

                    postponedAssignments.clear();
                }
            }

            // Write the statss
            auto &stats = requestState.stats().ordinaryTransferStats;

            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            stats.numCandidateVehiclesDropoffBNS += numCandidateVehiclesDropoffBNS;
            stats.numCandidateVehiclesDropoffORD += numCandidateVehiclesDropoffORD;
            stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;
            stats.numPartialsTriedPickupBNS += numPartialsTriedPickupBNS;
            stats.numPartialsTriedPickupORD += numPartialsTriedPickupORD;
            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedDropoffBNS += numAssignmentsTriedDropoffBNS;
            stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numStopPairs += numStopPairs;
            stats.numTransferPoints += numTransferPoints;
            stats.numDijkstraSearchesRun += numDijkstraSearchesRun;
            stats.numEdgesRelaxed += numEdgesRelaxed;
            stats.numVerticesScanned += numVerticesScanned;
            stats.searchTime += searchTime;
        }

    private:

        int getEarliestRelevantStopIdxForPVeh(const int vehId) const {
            if (relBNSPickups.hasRelevantSpotsFor(vehId))
                return 0;
            if (relORDPickups.hasRelevantSpotsFor(vehId))
                return relORDPickups.relevantSpotsFor(vehId)[0].stopIndex;
            return INFTY;
        }

        int getLatestRelevantStopIdxForDVeh(const int vehId) const {
            if (dVehIdsALS.contains(vehId))
                return routeState.numStopsOf(vehId) - 2;
            if (relORDDropoffs.hasRelevantSpotsFor(vehId))
                return relORDDropoffs.relevantSpotsFor(vehId)[relORDDropoffs.relevantSpotsFor(vehId).size() -
                                                              1].stopIndex;
            if (relBNSDropoffs.hasRelevantSpotsFor(vehId))
                return 0;
            return -1;
        }

        void findAssignmentsForVehiclePair(const int pVehId, const int dVehId) {

            const int numStopsPVeh = routeState.numStopsOf(pVehId);

            if (!relORDPickups.hasRelevantSpotsFor(pVehId) &&
                !relBNSPickups.hasRelevantSpotsFor(pVehId))
                return;

            if (!relBNSDropoffs.hasRelevantSpotsFor(dVehId) &&
                !relORDDropoffs.hasRelevantSpotsFor(dVehId) && dVehIdsALS.size() == 0)
                return;

            const auto stopIdsPVeh = routeState.stopIdsFor(pVehId);
            const auto stopIdsDVeh = routeState.stopIdsFor(dVehId);
            const auto earliestRelevantStopIdxPVeh = getEarliestRelevantStopIdxForPVeh(pVehId);
            const auto latestRelevantStopIdxDVeh = getLatestRelevantStopIdxForDVeh(dVehId);
            for (int trIdxPVeh = earliestRelevantStopIdxPVeh; trIdxPVeh < numStopsPVeh - 1; ++trIdxPVeh) {
                const auto stopIdPVeh = stopIdsPVeh[trIdxPVeh];

                for (int trIdxDVeh = 0; trIdxDVeh < latestRelevantStopIdxDVeh + 1; ++trIdxDVeh) {
                    ++numStopPairs;
                    const auto stopIdDVeh = stopIdsDVeh[trIdxDVeh];

                    const auto transferPoints = transferPointStrategy.getTransferPoints(stopIdPVeh, stopIdDVeh);
                    numTransferPoints += transferPoints.size();
                    ellipseIntersectionLogger << transferPoints.size() << '\n';

                    // For fixed transfer indices, try to find possible pickups
                    for (const auto &tp: transferPoints) {
                        tryPickupBNS(pVehId, dVehId, trIdxPVeh, trIdxDVeh, tp);
                        tryPickupORD(pVehId, dVehId, trIdxPVeh, trIdxDVeh, tp);
                    }

                }
            }
        }

        void tryPickupORD(const int pVehId, const int dVehId, const int trIdxPVeh, const int trIdxDVeh,
                          const TransferPoint &tp) {
            if (trIdxPVeh == 0 || !relORDPickups.hasRelevantSpotsFor(pVehId))
                return;

            for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                if (pickup.stopIndex > trIdxPVeh)
                    break; // pickups in relevant stops are ordered by stop index

                // Build the partial assignment with the transfer point
                PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                if (pickupPDLoc->loc == tp.loc || transferIsLaterOnRoute(dVehId, trIdxDVeh, tp.loc))
                    continue;

                AssignmentWithTransfer asgn(&fleet[pVehId], &fleet[dVehId], tp, pickupPDLoc, pickup.stopIndex,
                                            pickup.distToPDLoc,
                                            pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);

                finishDistancesPVeh(asgn);

                asgn.pickupType = ORDINARY;
                asgn.transferTypePVeh = ORDINARY;

                assert(asgn.pickup->id >= 0);
                // Try the partial assignment
                tryPartialAssignment(asgn);
            }
        }

        void tryPickupBNS(const int pVehId, const int dVehId, const int trIdxPVeh, const int trIdxDVeh,
                          const TransferPoint &tp) {
            if (!relBNSPickups.hasRelevantSpotsFor(pVehId))
                return;

            for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVehId)) {
                // Build the partial assignment with the transfer point
                PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                if (pickupPDLoc->loc == tp.loc || transferIsLaterOnRoute(dVehId, trIdxDVeh, tp.loc))
                    continue;

                AssignmentWithTransfer asgn(&fleet[pVehId], &fleet[dVehId], tp, pickupPDLoc, pickup.stopIndex,
                                            pickup.distToPDLoc,
                                            pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                finishDistancesPVeh(asgn);

                asgn.pickupType = BEFORE_NEXT_STOP;
                asgn.transferTypePVeh = trIdxPVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                assert(asgn.pickup->id >= 0);
                // Try the partial assignment
                tryPartialAssignment(asgn);
            }
        }

        bool transferIsLaterOnRoute(const int vehId, const int idx, const int loc) {
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            for (int i = idx + 1; i < stopLocations.size(); ++i) {
                if (stopLocations[i] == loc)
                    return true;
            }

            return false;
        }

        void tryPartialAssignment(AssignmentWithTransfer &asgn) {
            const bool unfinished = asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed;
            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) are set
            RequestCost pickupVehCost;

            assert(asgn.pVeh && asgn.pickup);
            if (asgn.distToPickup == INFTY || asgn.distFromPickup == INFTY || asgn.distToTransferPVeh == INFTY ||
                asgn.distFromTransferPVeh == INFTY)
                return;

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    ++numPartialsTriedPickupBNS;
                    break;
                case ORDINARY:
                    ++numPartialsTriedPickupORD;
                    break;
                default:
                    assert(false);
            }


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

            for (const auto &dropoff: relBNSDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId)) {
                assert(asgn.transferIdxDVeh == 0);
                assert(dropoff.stopIndex == 0);

                PDLoc *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                if (dropoffPDLoc->loc == asgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoff = &requestState.dropoffs[dropoff.pdId];
                newAssignment.dropoffIdx = dropoff.stopIndex;

                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                finishDistancesDVeh(newAssignment, 0);

                newAssignment.dropoffType = BEFORE_NEXT_STOP;
                newAssignment.transferTypeDVeh = BEFORE_NEXT_STOP;

                if (newAssignment.dropoff->loc == newAssignment.transfer.loc)
                    continue;

                tryAssignment(newAssignment);
            }
        }

        void tryDropoffORD(const AssignmentWithTransfer &asgn) {
            if (relORDDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId).size() == 0)
                return;

            for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(asgn.dVeh->vehicleId)) {
                if (dropoff.stopIndex < asgn.transferIdxDVeh)
                    continue;

                PDLoc *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                if (dropoffPDLoc->loc == asgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoffIdx = dropoff.stopIndex;
                newAssignment.dropoff = dropoffPDLoc;

                newAssignment.distToDropoff = dropoff.distToPDLoc;
                newAssignment.distFromDropoff = dropoff.distFromPDLocToNextStop;
                finishDistancesDVeh(newAssignment, 0);

                newAssignment.dropoffType = ORDINARY;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                if (newAssignment.dropoff->loc == newAssignment.transfer.loc)
                    continue;

                tryAssignment(newAssignment);
            }
        }

        void tryDropoffALS(const AssignmentWithTransfer &asgn) {
            if (!dVehIdsALS.contains(asgn.dVeh->vehicleId))
                return;

            for (const auto &dropoff: requestState.dropoffs) {
                int distanceToDropoff = dropoffALSStrategy.getDistanceToDropoff(asgn.dVeh->vehicleId, dropoff.id);
                if (distanceToDropoff >= INFTY)
                    continue;

                if (dropoff.loc == asgn.transfer.loc)
                    continue;

                AssignmentWithTransfer newAssignment(asgn);
                newAssignment.dropoffIdx = routeState.numStopsOf(asgn.dVeh->vehicleId) - 1;
                newAssignment.dropoff = &dropoff;

                newAssignment.distToDropoff = distanceToDropoff;
                newAssignment.distFromDropoff = 0;
                finishDistancesDVeh(newAssignment, distanceToDropoff);

                newAssignment.dropoffType = AFTER_LAST_STOP;
                newAssignment.transferTypeDVeh = newAssignment.transferIdxDVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                tryAssignment(newAssignment);
            }
        }

        void finishAssignments(const int pVehId, const int dVehId) {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate;
            std::vector<AssignmentWithTransfer> currentlyCalculating;
            std::vector<AssignmentWithTransfer> temp;

            RequestCost total;
            // Start with the pickups with postponed bns distance
            for (auto &asgn: postponedAssignments) {
                if (asgn.pickupBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                } else {
                    toCalculate.push_back(asgn);
                }
            }

            postponedAssignments.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactDistancesVia(fleet[pVehId]);

            for (auto &asgn: currentlyCalculating) {
                KASSERT(searches.knowsCurrentLocationOf(pVehId));
                KASSERT(searches.knowsDistance(pVehId, asgn.pickup->id));
                const int distance = searches.getDistance(pVehId, asgn.pickup->id);
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
            currentlyCalculating.clear();

            // Calculate the exact paired distance between pickup and transfer
            std::vector<int> sources;
            std::vector<int> targets;
            std::vector<int> offsets;

            for (auto &asgn: toCalculate) {
                if (asgn.pickupPairedLowerBoundUsed) {
                    const int transferRank = vehCh.rank(inputGraph.edgeTail(asgn.transfer.loc));
                    const int transferOffset = inputGraph.travelTime(asgn.transfer.loc);

                    pickupToTransferDistancesFinder.runQueryForTransferRank(transferRank);
                    const int distance =
                            pickupToTransferDistancesFinder.getDistances().getDistance(asgn.pickup->id, transferRank) +
                            transferOffset;

                    asgn.distToTransferPVeh = distance;
                    asgn.pickupPairedLowerBoundUsed = false;

                    // Try the assignments with the calculated distances
                    if (!asgn.isFinished()) {
                        total = calc.calcBaseLowerBound<true, true>(asgn, requestState);

                        if (total.total > requestState.getBestCost())
                            continue;

                        temp.push_back(asgn);
                    } else {
                        requestState.tryAssignment(asgn);
                        continue;
                    }
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();
            currentlyCalculating.clear();
            // Calculate the dropoffs with postponed bns distance
            for (auto &asgn: toCalculate) {
                if (asgn.dropoffBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    asgn.dropoffBNSLowerBoundUsed = false;
                    searches.addTransferForProcessing(asgn.transfer.loc, asgn.distToTransferDVeh);
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactTransferDistancesVia(fleet[dVehId]);

            for (auto &asgn: currentlyCalculating) {
                assert(searches.knowsDistanceTransfer(dVehId, asgn.transfer.loc));
                assert(searches.knowsCurrentLocationOf(dVehId));
                const int distance = searches.getDistanceTransfer(dVehId, asgn.transfer.loc);
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

            sources.clear();
            targets.clear();
            offsets.clear();

            for (auto &asgn: toCalculate) {
                assert(asgn.dropoffPairedLowerBoundUsed);

                const int transferRank = vehCh.rank(inputGraph.edgeHead(asgn.transfer.loc));
                transferToDropoffDistancesFinder.runQueryForTransferRank(transferRank);
                const int distance = transferToDropoffDistancesFinder.getDistances().getDistance(asgn.dropoff->id,
                                                                                                 transferRank);

                asgn.distToDropoff = distance;
                asgn.dropoffPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                assert(asgn.isFinished());
                requestState.tryAssignment(asgn);
            }
        }

        void finishDistancesPVeh(AssignmentWithTransfer &asgn) {
            const auto stopLocations = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const int numStops = routeState.numStopsOf(asgn.pVeh->vehicleId);
            unused(numStops);

            const auto schedDepTimes = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimes = routeState.schedArrTimesFor(asgn.pVeh->vehicleId);

            const int pickupIdx = asgn.pickupIdx;
            const int transferIdx = asgn.transferIdxPVeh;

            assert(pickupIdx < numStops - 1);

            const int pickup = asgn.pickup->loc;
            const int transfer = asgn.transfer.loc;

            const bool bns = pickupIdx == 0;
            const bool paired = pickupIdx == transferIdx;

            const bool pickupAtStop = stopLocations[pickupIdx] == pickup;
            const bool transferAtStop = stopLocations[transferIdx] == transfer;

            const int legPickup = schedArrTimes[pickupIdx + 1] - schedDepTimes[pickupIdx];
            const int legTransfer = schedArrTimes[transferIdx + 1] - schedDepTimes[transferIdx];

            //* Pickup distances
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (paired) {
                // Paired Assignment (pVeh)
                // Try the lower bound for the paired assignment
                asgn.distFromPickup = 0;
                asgn.pickupPairedLowerBoundUsed = true;
            }

            if (pickupAtStop && !paired)
                asgn.distFromPickup = legPickup;

            if (bns) {
                // Assignment with pickup BNS
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
                }
            }

            //* Transfer distances pVeh
            if (paired) {
                asgn.distToTransferPVeh = 0;
                asgn.pickupPairedLowerBoundUsed = true;
            }

            if (transferAtStop && !paired)
                asgn.distToTransferPVeh = 0;

            if (transferAtStop)
                asgn.distFromTransferPVeh = legTransfer;

        }

        void finishDistancesDVeh(AssignmentWithTransfer &asgn, const int dropoffAlsDistance) {
            const auto stopLocations = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            const int numStops = routeState.numStopsOf(asgn.dVeh->vehicleId);

            const auto schedDepTimes = routeState.schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimes = routeState.schedArrTimesFor(asgn.dVeh->vehicleId);

            const int transferIdx = asgn.transferIdxDVeh;
            const int dropoffIdx = asgn.dropoffIdx;

            assert(transferIdx < numStops - 1);

            const int transfer = asgn.transfer.loc;
            const int dropoff = asgn.dropoff->loc;

            const bool bns = transferIdx == 0;
            const bool paired = transferIdx == dropoffIdx;
            const bool dropoffALS = dropoffIdx == numStops - 1;

            const bool transferAtStop = stopLocations[transferIdx] == transfer;
            const bool dropoffAtStop = stopLocations[dropoffIdx] == dropoff;

            const int legTransfer = schedArrTimes[transferIdx + 1] - schedDepTimes[transferIdx];
            const int legDropoff =
                    dropoffIdx < numStops - 1 ? schedArrTimes[dropoffIdx + 1] - schedDepTimes[dropoffIdx] : 0;

            //* Transfer distances
            if (transferAtStop)
                asgn.distToTransferDVeh = 0;

            if (paired) {
                // Paired Assignment (dVeh)
                // Try the lower bound for the paired assignment
                asgn.distFromTransferDVeh = 0;
                asgn.dropoffPairedLowerBoundUsed = true;
            }

            if (transferAtStop && !paired)
                asgn.distFromTransferDVeh = legTransfer;

            if (bns) {
                // Transfer bns in dropoff vehicle
                if (searches.knowsDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc)) {
                    asgn.distToTransferDVeh = searches.getDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc);
                } else {
                    asgn.dropoffBNSLowerBoundUsed = true;
                }
            }

            //* Dropoff distances
            if (paired) {
                asgn.distToDropoff = 0;
                asgn.dropoffPairedLowerBoundUsed = true;
            }

            if (dropoffAtStop && !paired)
                asgn.distToDropoff = 0;

            if (dropoffALS)
                asgn.distFromDropoff = dropoffAlsDistance;

            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);

            if ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                || (asgn.transferIdxPVeh < numStopsPVeh - 1 &&
                    asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1])
                || (asgn.transferIdxDVeh < numStopsDVeh - 1 &&
                    asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
                || (asgn.dropoffIdx < numStopsDVeh - 1 && asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]))
                return;

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    ++numAssignmentsTriedPickupBNS;
                    break;
                case ORDINARY:
                    ++numAssignmentsTriedPickupORD;
                    break;
                default:
                    assert(false);
            }

            switch (asgn.dropoffType) {
                case BEFORE_NEXT_STOP:
                    ++numAssignmentsTriedDropoffBNS;
                    break;
                case ORDINARY:
                    ++numAssignmentsTriedDropoffORD;
                    break;
                case AFTER_LAST_STOP:
                    ++numAssignmentsTriedDropoffALS;
                    break;
                default:
                    assert(false);
            }

            Timer time;
            requestState.tryAssignment(asgn);
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }


        void constructPVehSet() {
            pVehs.clear();

            for (const int pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                pVehs.insert(pVehId);
            }

            for (const int pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                pVehs.insert(pVehId);
            }
        }

        void constructDVehSet() {
            dVehs.clear();

            for (const int dVehId: relBNSDropoffs.getVehiclesWithRelevantPDLocs()) {
                dVehs.insert(dVehId);
            }

            for (const int dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                dVehs.insert(dVehId);
            }

            // Calculate the possible vehicles for dropoff als
            dVehIdsALS = dropoffALSStrategy.findDropoffsAfterLastStop();
            numCandidateVehiclesDropoffALS += dVehIdsALS.size();

            for (const int dVehId: dVehIdsALS) {
                dVehs.insert(dVehId);
            }
        }

        bool assertTransferPointCalculation(const TransferPoint tp) {
            const auto stopLocationsPVeh = routeState.stopLocationsFor(tp.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(tp.dVeh->vehicleId);

            const auto stopIndexPVeh = tp.stopIdxDVeh;
            const auto stopIndexDVeh = tp.stopIdxPVeh;

            const auto stopLocPVeh = stopLocationsPVeh[stopIndexPVeh];
            const auto nextStopLocPVeh = stopLocationsPVeh[stopIndexPVeh + 1];
            const auto stopLocDVeh = stopLocationsDVeh[stopIndexDVeh];
            const auto nextStopLocDVeh = stopLocationsDVeh[stopIndexDVeh + 1];

            const auto headStopPVeh = inputGraph.edgeHead(stopLocPVeh);
            const auto tailNextStopPVeh = inputGraph.edgeTail(nextStopLocPVeh);
            const auto headStopDVeh = inputGraph.edgeHead(stopLocDVeh);
            const auto tailNextStopDVeh = inputGraph.edgeTail(nextStopLocDVeh);

            // Calculate leg length pVeh and dVeh
            const auto headStopPVehRank = vehCh.rank(headStopPVeh);
            const auto tailNextStopPVehRank = vehCh.rank(tailNextStopPVeh);
            const auto headStopDVehRank = vehCh.rank(headStopDVeh);
            const auto tailNextStopDVehRank = vehCh.rank(tailNextStopDVeh);

            const auto nextStopPVehLength = inputGraph.travelTime(nextStopLocPVeh);
            const auto nextStopDVehLength = inputGraph.travelTime(nextStopLocDVeh);

            vehChQuery.run(headStopPVehRank, tailNextStopPVehRank);
            const int legLenthPVeh = vehChQuery.getDistance() + nextStopPVehLength;

            vehChQuery.run(headStopDVehRank, tailNextStopDVehRank);
            const int legLenthDVeh = vehChQuery.getDistance() + nextStopDVehLength;

            const auto routeStateLengthPVeh = time_utils::calcLengthOfLegStartingAt(stopIndexPVeh, tp.pVeh->vehicleId,
                                                                                    routeState);
            const auto routeStateLengthDVeh = time_utils::calcLengthOfLegStartingAt(stopIndexDVeh, tp.dVeh->vehicleId,
                                                                                    routeState);

            unused(legLenthPVeh, legLenthDVeh, routeStateLengthPVeh, routeStateLengthDVeh);
            KASSERT(routeStateLengthPVeh == legLenthPVeh);
            KASSERT(routeStateLengthDVeh == legLenthDVeh);

            // Recalculate the distance to and from the transfer point
            const auto transferPointHead = inputGraph.edgeHead(tp.loc);
            const auto transferPointTail = inputGraph.edgeTail(tp.loc);
            const auto transferPointHeadRank = vehCh.rank(transferPointHead);
            const auto transferPointTailRank = vehCh.rank(transferPointTail);
            const auto transferLength = inputGraph.travelTime(tp.loc);

            vehChQuery.run(headStopPVehRank, transferPointTailRank);
            const auto distToTransferPVeh = stopLocPVeh == tp.loc ? 0 : vehChQuery.getDistance() + transferLength;

            vehChQuery.run(transferPointHeadRank, tailNextStopPVehRank);
            const auto distFromTransferPVeh =
                    nextStopLocPVeh == tp.loc ? 0 : vehChQuery.getDistance() + nextStopPVehLength;

            vehChQuery.run(headStopDVehRank, transferPointTailRank);
            const auto distToTransferDVeh = stopLocDVeh == tp.loc ? 0 : vehChQuery.getDistance() + transferLength;

            vehChQuery.run(transferPointHeadRank, tailNextStopDVehRank);
            const auto distFromTransferDVeh =
                    nextStopLocDVeh == tp.loc ? 0 : vehChQuery.getDistance() + nextStopDVehLength;

            if (distToTransferPVeh != tp.distancePVehToTransfer ||
                distFromTransferPVeh != tp.distancePVehFromTransfer ||
                distToTransferDVeh != tp.distanceDVehToTransfer ||
                distFromTransferDVeh != tp.distanceDVehFromTransfer) {
                assert(false);
            }

            return true;
        }

        std::vector<AssignmentWithTransfer> promisingPartials;

        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        CurVehLocToPickupSearchesT &searches;
        TransferPointStrategyT &transferPointStrategy;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;
        const RelevantPDLocs &relBNSDropoffs;

        DirectTransferDistancesFinderT &pickupToTransferDistancesFinder;
        DirectTransferDistancesFinderT &transferToDropoffDistancesFinder;

        std::vector<AssignmentWithTransfer> &postponedAssignments;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        CostCalculator &calc;
        InsertionAsserterT &asserter;

        Subset pVehs;
        Subset dVehs;

        Subset dVehIdsALS;

        LightweightSubset edgesSubset; // Subset used to deduplicate locations in preparing any-to-any searches

        FastResetFlagArray<uint32_t> stopSeen;

        //* Statistics for the ordinary transfer assignment finder
        int64_t totalTime;

        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;

        int64_t numCandidateVehiclesDropoffBNS;
        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;

        // Stats for the tried assignments
        int64_t numPartialsTriedPickupBNS;
        int64_t numPartialsTriedPickupORD;

        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;

        int64_t numAssignmentsTriedDropoffBNS;
        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;

        // Stats for the transfer search itself
        int64_t numStopPairs;
        int64_t numTransferPoints;
        int64_t numDijkstraSearchesRun;
        int64_t numEdgesRelaxed;
        int64_t numVerticesScanned;
        int64_t searchTime;

        std::ofstream &ellipsesLogger;
        std::ofstream &ellipseIntersectionLogger;

    };
}
