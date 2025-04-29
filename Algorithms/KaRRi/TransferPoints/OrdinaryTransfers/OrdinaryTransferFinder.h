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

#include "Algorithms/KaRRi/TransferPoints/VertexInEllipse.h"

#include "Tools/Logging/LogManager.h"
#include "DataStructures/Containers/LightweightSubset.h"

#pragma once

namespace karri {

    template<typename TransferPointFinderT, typename TransfersDropoffALSStrategyT, typename InputGraphT, typename VehCHEnvT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT, typename CHEllipseReconstructorT>
    class OrdinaryTransferFinder {

        using VehCHQueryLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<VehCHQueryLabelSet>;


        // Both vehicles can drive a detour to the transfer point, but none drives the detour ALS
        // This implies, that the pickup is BNS or ORD, the dropoff can be BNS, ORD or ALS
        // If the dropoff is ALS, we need to consider a different set of vehicles to calculate the transfer psints between
    public:

        OrdinaryTransferFinder(
                TransferPointFinderT &tpFinder,
                TransfersDropoffALSStrategyT &dropoffALSStrategy,
                CHEllipseReconstructorT &chEllipseReconstructor,
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
                InsertionAsserterT &asserter,
                std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints
        ) : tpFinder(tpFinder),
            dropoffALSStrategy(dropoffALSStrategy),
            chEllipseReconstructor(chEllipseReconstructor),
            inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<VehCHQueryLabelSet>()),
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
            asserter(asserter),
            distanceFromVertexToNextStop(inputGraph.numVertices(), INFTY),
            transferPoints(transferPoints),
            unionOfTransferPointLocs(inputGraph.numEdges()),
            dVehIdsALS(Subset(0)),
            edgesSubset(inputGraph.numEdges()),
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

            ptAnyToAny_numSearchesRun = 0;
            ptAnyToAny_sumNumSources = 0;
            ptAnyToAny_sumNumTargets = 0;
            ptAnyToAny_numVerticesSettled = 0;
            ptAnyToAny_numEdgeRelaxations = 0;
            ptAnyToAny_numResultZero = 0;
            tdAnyToAny_numSearchesRun = 0;
            tdAnyToAny_sumNumSources = 0;
            tdAnyToAny_sumNumTargets = 0;
            tdAnyToAny_numVerticesSettled = 0;
            tdAnyToAny_numEdgeRelaxations = 0;
            tdAnyToAny_numResultZero = 0;
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

            promisingPartials = std::vector<AssignmentWithTransfer>{};

            // Construct the set of possible pickup and dropoff vehicles
            std::vector<int> pVehIds = constructPVehSet();
            std::vector<int> dVehIds = constructDVehSet();

            std::vector<const Vehicle *> pVehs = std::vector<const Vehicle *>{};
            std::vector<const Vehicle *> dVehs = std::vector<const Vehicle *>{};

            for (const int pVehId: pVehIds) {
                pVehs.push_back(&fleet[pVehId]);
            }

            for (const int dVehId: dVehIds) {
                dVehs.push_back(&fleet[dVehId]);
            }

            // Calculate the necessary ellipses for every vehicle for pickup and dropoff
            std::vector<int> idxOfStop(routeState.getMaxStopId() + 1, INVALID_INDEX);
            std::vector<int> stopIdsForEllipses;
            for (const auto *pVeh: pVehs) {
                const auto stopIds = routeState.stopIdsFor(pVeh->vehicleId);
                for (int i = 0; i < routeState.numStopsOf(pVeh->vehicleId) - 1; i++) {
                    if (idxOfStop[stopIds[i]] == INVALID_INDEX) {
                        idxOfStop[stopIds[i]] = stopIdsForEllipses.size();
                        stopIdsForEllipses.push_back(stopIds[i]);
                    }
                }
            }

            for (const auto *dVeh: dVehs) {
                const auto stopIds = routeState.stopIdsFor(dVeh->vehicleId);
                for (int i = 0; i < routeState.numStopsOf(dVeh->vehicleId) - 1; i++) {
                    if (idxOfStop[stopIds[i]] == INVALID_INDEX) {
                        idxOfStop[stopIds[i]] = stopIdsForEllipses.size();
                        stopIdsForEllipses.push_back(stopIds[i]);
                    }
                }
            }

            // Calculate all necessary ellipses
            Timer searchTimer;
            auto vertexEllipses = chEllipseReconstructor.getVerticesInEllipsesOfLegsAfterStops(stopIdsForEllipses,
                                                                                                     numVerticesScanned,
                                                                                                     numEdgesRelaxed);

            // Convert the ellipses of vertices to ellipses of edges
            std::vector<std::vector<EdgeInEllipse>> edgeEllipses;
            for (const auto &stopId: stopIdsForEllipses) {
                auto& vertexEllipse = vertexEllipses[idxOfStop[stopId]];

                // Sort vertex ellipse, which will also lead to sorted edge ellipse (if the edges in the graph are sorted).
                std::sort(vertexEllipse.begin(), vertexEllipse.end(), [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                    return v1.vertex < v2.vertex;
                });

                const auto leeway = routeState.leewayOfLegStartingAt(stopId);
                const auto edgeEllipse = convertVertexEllipseIntoEdgeEllipse(vertexEllipse, leeway);
                ellipsesLogger << edgeEllipse.size() << '\n';

                KASSERT(std::is_sorted(edgeEllipse.begin(), edgeEllipse.end(), [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                    return e1.edge < e2.edge;
                }));

                edgeEllipses.push_back(edgeEllipse);
            }

            searchTime += searchTimer.elapsed<std::chrono::nanoseconds>();

            // Loop over all the possible vehicle combinations
            for (const auto *pVeh: pVehs) {
                for (const auto *dVeh: dVehs) {

                    // pVeh an dVeh can not be the same vehicles
                    if (dVeh->vehicleId == pVeh->vehicleId)
                        continue;

                    // Now we have a vehicle pair to work with
                    // Find the transfer points between the two vehicles


                    // Populate the map
                    transferPoints.clear();
                    unionOfTransferPointLocs.clear();
                    for (int stopIdxPVeh = 0; stopIdxPVeh < routeState.numStopsOf(pVeh->vehicleId) - 1; stopIdxPVeh++) {
                        const int stopIdPVeh = routeState.stopIdsFor(pVeh->vehicleId)[stopIdxPVeh];
                        const auto &ellipsePVeh = edgeEllipses[idxOfStop[stopIdPVeh]];

                        for (int stopIdxDVeh = 0;
                             stopIdxDVeh < routeState.numStopsOf(dVeh->vehicleId) - 1; stopIdxDVeh++) {
                            const int stopIdDVeh = routeState.stopIdsFor(dVeh->vehicleId)[stopIdxDVeh];
                            const auto &ellipseDVeh = edgeEllipses[idxOfStop[stopIdDVeh]];

                            transferPoints[{stopIdxPVeh, stopIdxDVeh}] = getIntersectionOfEllipses(ellipsePVeh,
                                                                                                   ellipseDVeh, pVeh,
                                                                                                   dVeh, stopIdxPVeh,
                                                                                                   stopIdxDVeh);


                            for (const auto &tp: transferPoints[{stopIdxPVeh, stopIdxDVeh}]) {
                                unionOfTransferPointLocs.insert(tp.loc);
                            }

                            ellipseIntersectionLogger << transferPoints[{stopIdxPVeh, stopIdxDVeh}].size() << '\n';
                        }
                    }


                    // calculateTransferPoints(pVeh, dVeh);

                    if (transferPointsSize() == 0)
                        continue;

                    numTransferPoints += transferPointsSize();

                    // Find the assignments with the transfer points
                    findAssignmentsForVehiclePair(pVeh, dVeh);

                    if (promisingPartials.size() == 0)
                        continue;

                    for (auto &asgn: promisingPartials) {
                        tryDropoffBNS(asgn);
                        tryDropoffORD(asgn);
                        tryDropoffALS(asgn);
                    }

                    promisingPartials.clear();

                    if (postponedAssignments.size() == 0)
                        continue;

                    // Finish the postponed assignments
                    finishAssignments(pVeh, dVeh);

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
            stats.ptAnyToAny_numSearchesRun += ptAnyToAny_numSearchesRun;
            stats.ptAnyToAny_sumNumSources += ptAnyToAny_sumNumSources;
            stats.ptAnyToAny_sumNumTargets += ptAnyToAny_sumNumTargets;
            stats.ptAnyToAny_numVerticesSettled += ptAnyToAny_numVerticesSettled;
            stats.ptAnyToAny_numEdgeRelaxations += ptAnyToAny_numEdgeRelaxations;
            stats.ptAnyToAny_numResultZero += ptAnyToAny_numResultZero;
            stats.tdAnyToAny_numSearchesRun += tdAnyToAny_numSearchesRun;
            stats.tdAnyToAny_sumNumSources += tdAnyToAny_sumNumSources;
            stats.tdAnyToAny_sumNumTargets += tdAnyToAny_sumNumTargets;
            stats.tdAnyToAny_numVerticesSettled += tdAnyToAny_numVerticesSettled;
            stats.tdAnyToAny_numEdgeRelaxations += tdAnyToAny_numEdgeRelaxations;
            stats.tdAnyToAny_numResultZero += tdAnyToAny_numResultZero;
        }

    private:

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        void calculateTransferPoints(const Vehicle *pVeh, const Vehicle *dVeh) {
            tpFinder.init();

            //* Use the transfer point finder to find the possible transfer points  
            tpFinder.findTransferPoints(*pVeh, *dVeh);

            numDijkstraSearchesRun += tpFinder.getNumSearchesRun();
            numEdgesRelaxed += tpFinder.getNumEdgesRelaxed();
            numVerticesScanned += tpFinder.getNumVerticesScanned();
        }

        void findAssignmentsForVehiclePair(const Vehicle *pVeh, const Vehicle *dVeh) {

//            pairedLowerBoundPT = calculateLowerBoundPairedPT(pVeh);
//            pairedLowerBoundTD = calculateLowerBoundPairedTD(dVeh);

            const int numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(dVeh->vehicleId);

            if (!relORDPickups.hasRelevantSpotsFor(pVeh->vehicleId) &&
                !relBNSPickups.hasRelevantSpotsFor(pVeh->vehicleId))
                return;

            if (!relBNSDropoffs.hasRelevantSpotsFor(dVeh->vehicleId) &&
                !relORDDropoffs.hasRelevantSpotsFor(dVeh->vehicleId) && dVehIdsALS.size() == 0)
                return;

            for (int trIdxPVeh = 0; trIdxPVeh < numStopsPVeh - 1; trIdxPVeh++) {
                for (int trIdxDVeh = 0; trIdxDVeh < numStopsDVeh - 1; trIdxDVeh++) {
                    numStopPairs++;
                    // For fixed transfer indices, try to find possible pickups
                    tryPickupBNS(pVeh, dVeh, trIdxPVeh, trIdxDVeh);
                    tryPickupORD(pVeh, dVeh, trIdxPVeh, trIdxDVeh);
                }
            }
        }


        int computeLowerBoundDistanceFromAnyPickupToAnyTransfer(const std::vector<TransferPoint> &tps) {
            std::vector<int> sourceRanks;
            std::vector<int> targetRanks;
            std::vector<int> targetOffsets;

            for (const auto &p: requestState.pickups) {
                sourceRanks.push_back(vehCh.rank(inputGraph.edgeHead(p.loc)));
            }
            for (const auto &tp: tps) {
                targetRanks.push_back(vehCh.rank(inputGraph.edgeTail(tp.loc)));
                targetOffsets.push_back(inputGraph.travelTime(tp.loc));
            }

            const auto result = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks, targetOffsets);
            ++ptAnyToAny_numSearchesRun;
            ptAnyToAny_sumNumSources += sourceRanks.size();
            ptAnyToAny_sumNumTargets += targetRanks.size();
            ptAnyToAny_numVerticesSettled += vehChQuery.getNumVerticesSettled();
            ptAnyToAny_numEdgeRelaxations += vehChQuery.getNumEdgeRelaxations();
            ptAnyToAny_numResultZero += (result == 0);
            return result;
        }

        int computeLowerBoundDistanceFromAnyTransferToAnyDropoff(const std::vector<TransferPoint> &tps) {
            std::vector<int> sourceRanks;
            std::vector<int> targetRanks;
            std::vector<int> targetOffsets;

            for (const auto &tp: tps) {
                sourceRanks.push_back(vehCh.rank(inputGraph.edgeHead(tp.loc)));
            }
            for (const auto &d: requestState.dropoffs) {
                targetRanks.push_back(vehCh.rank(inputGraph.edgeTail(d.loc)));
                targetOffsets.push_back(inputGraph.travelTime(d.loc));
            }

            const auto result = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks, targetOffsets);
            ++tdAnyToAny_numSearchesRun;
            tdAnyToAny_sumNumSources += sourceRanks.size();
            tdAnyToAny_sumNumTargets += targetRanks.size();
            tdAnyToAny_numVerticesSettled += vehChQuery.getNumVerticesSettled();
            tdAnyToAny_numEdgeRelaxations += vehChQuery.getNumEdgeRelaxations();
            tdAnyToAny_numResultZero += (result == 0);
            return result;
        }

        void tryPickupORD(const Vehicle *pVeh, const Vehicle *dVeh, const int trIdxPVeh, const int trIdxDVeh) {
            if (trIdxPVeh == 0 || !relORDPickups.hasRelevantSpotsFor(pVeh->vehicleId))
                return;

            const auto transferPointsForStopPair = transferPoints[{trIdxPVeh, trIdxDVeh}];

            if (transferPointsForStopPair.empty())
                return;

            for (const auto &pickup: relORDPickups.relevantSpotsFor(pVeh->vehicleId)) {
                if (pickup.stopIndex > trIdxPVeh)
                    continue;

                // Try all possible transfer points between the stops of the two vehicles
                for (const auto tp: transferPointsForStopPair) {
                    // Build the partial assignment with the transfer point
                    PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                    if (pickupPDLoc->loc == tp.loc || transferIsLaterOnRoute(dVeh->vehicleId, trIdxDVeh, tp.loc))
                        continue;

                    AssignmentWithTransfer asgn(pVeh, dVeh, tp, pickupPDLoc, pickup.stopIndex, pickup.distToPDLoc,
                                                pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                    finishDistancesPVeh(asgn);

                    asgn.pickupType = ORDINARY;
                    asgn.transferTypePVeh = ORDINARY;

                    assert(asgn.pickup->id >= 0);
                    // Try the partial assignment
                    tryPartialAssignment(asgn);
                }
            }
        }

        void tryPickupBNS(const Vehicle *pVeh, const Vehicle *dVeh, const int trIdxPVeh, const int trIdxDVeh) {
            if (!relBNSPickups.hasRelevantSpotsFor(pVeh->vehicleId))
                return;

            const auto transferPointsForStopPair = transferPoints[{trIdxPVeh, trIdxDVeh}];

            if (transferPointsForStopPair.size() == 0)
                return;

            for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVeh->vehicleId)) {
                for (const auto tp: transferPointsForStopPair) {
                    // Build the partial assignment with the transfer point
                    PDLoc *pickupPDLoc = &requestState.pickups[pickup.pdId];

                    if (pickupPDLoc->loc == tp.loc || transferIsLaterOnRoute(dVeh->vehicleId, trIdxDVeh, tp.loc))
                        continue;

                    AssignmentWithTransfer asgn(pVeh, dVeh, tp, pickupPDLoc, pickup.stopIndex, pickup.distToPDLoc,
                                                pickup.distFromPDLocToNextStop, trIdxPVeh, trIdxDVeh);
                    finishDistancesPVeh(asgn);

                    asgn.pickupType = BEFORE_NEXT_STOP;
                    asgn.transferTypePVeh = trIdxPVeh == 0 ? BEFORE_NEXT_STOP : ORDINARY;

                    assert(asgn.pickup->id >= 0);
                    // Try the partial assignment
                    tryPartialAssignment(asgn);
                }
            }
        }

        bool transferIsLaterOnRoute(const int vehId, const int idx, const int loc) {
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            if (idx >= stopLocations.size())
                return false;

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
                    numPartialsTriedPickupBNS++;
                    break;
                case ORDINARY:
                    numPartialsTriedPickupORD++;
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
            if (!alsDropoffVehs[asgn.dVeh->vehicleId])
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

        int transferPointsSize() {
            int total = 0;
            for (const auto &it: transferPoints)
                total += it.second.size();

            return total;
        }

        void finishAssignments(const Vehicle *pVeh, const Vehicle *dVeh) {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate;
            std::vector<AssignmentWithTransfer> currentlyCalculating;
            std::vector<AssignmentWithTransfer> temp;

            RequestCost total;
            // Start with the pickups with postponed bns distance
            for (auto asgn: postponedAssignments) {
                if (asgn.pickupBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                } else {
                    toCalculate.push_back(asgn);
                }
            }

            postponedAssignments.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactDistancesVia(*pVeh);

            for (auto asgn: currentlyCalculating) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));
                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
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
            auto sources = std::vector<int>{};
            auto targets = std::vector<int>{};
            auto offsets = std::vector<int>{};

            for (auto asgn: toCalculate) {
                if (asgn.pickupPairedLowerBoundUsed) {
                    int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.pickup->loc));
                    int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.transfer.loc));
                    int offset = inputGraph.travelTime(asgn.transfer.loc);
                    vehChQuery.run(sourceRank, targetRank);
                    const int distance = vehChQuery.getDistance() + offset;
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
            for (auto asgn: toCalculate) {
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
                searches.computeExactTransferDistancesVia(*dVeh);

            for (auto asgn: currentlyCalculating) {
                assert(searches.knowsDistanceTransfer(dVeh->vehicleId, asgn.transfer.loc));
                assert(searches.knowsCurrentLocationOf(dVeh->vehicleId));
                const int distance = searches.getDistanceTransfer(dVeh->vehicleId, asgn.transfer.loc);
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

            for (auto asgn: toCalculate) {
                assert(asgn.dropoffPairedLowerBoundUsed);

                int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.transfer.loc));
                int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.dropoff->loc));
                int offset = inputGraph.travelTime(asgn.dropoff->loc);

                vehChQuery.run(sourceRank, targetRank);
                const int distance = vehChQuery.getDistance() + offset;
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
//                asgn.distToTransferPVeh = pairedLowerBoundPT;
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
//                asgn.distToDropoff = pairedLowerBoundTD;
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
                    numAssignmentsTriedPickupBNS++;
                    break;
                case ORDINARY:
                    numAssignmentsTriedPickupORD++;
                    break;
                default:
                    assert(false);
            }

            switch (asgn.dropoffType) {
                case BEFORE_NEXT_STOP:
                    numAssignmentsTriedDropoffBNS++;
                    break;
                case ORDINARY:
                    numAssignmentsTriedDropoffORD++;
                    break;
                case AFTER_LAST_STOP:
                    numAssignmentsTriedDropoffALS++;
                    break;
                default:
                    assert(false);
            }

            Timer time;
            requestState.tryAssignment(asgn);
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }


        std::vector<int> constructPVehSet() {
            std::vector<bool> markedVehs(fleet.size(), false);
            std::vector<int> pVehIds = std::vector<int>{};

            for (const int pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                if (markedVehs[pVehId])
                    continue;

                pVehIds.push_back(pVehId);
                markedVehs[pVehId] = true;
            }

            for (const int pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
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

            for (const int dVehId: relBNSDropoffs.getVehiclesWithRelevantPDLocs()) {
                if (alsDropoffVehs[dVehId])
                    continue;

                dVehIds.push_back(dVehId);
                alsDropoffVehs[dVehId] = true;
            }

            for (const int dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                if (alsDropoffVehs[dVehId])
                    continue;

                dVehIds.push_back(dVehId);
                alsDropoffVehs[dVehId] = true;
            }

            // Calculate the possible vehicles for dropoff als
            dVehIdsALS = dropoffALSStrategy.findDropoffsAfterLastStop();
            numCandidateVehiclesDropoffALS += dVehIds.size();

            for (const int dVehId: dVehIdsALS) {
                dVehIds.push_back(dVehId);
            }

            return dVehIds;
        }

        int calculateLowerBoundPairedPT(const Vehicle *pVeh) {
            std::vector<int> sourceRanks;
            std::vector<int> targetRanks;
            std::vector<int> targetOffsets;

            edgesSubset.clear();
            for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVeh->vehicleId)) {
                edgesSubset.insert(requestState.pickups[pickup.pdId].loc);
            }

            for (const auto &pickup: relORDPickups.relevantSpotsFor(pVeh->vehicleId)) {
                edgesSubset.insert(requestState.pickups[pickup.pdId].loc);
            }

            for (const auto &pickupEdge: edgesSubset) {
                const auto head = inputGraph.edgeHead(pickupEdge);
                const auto headRank = vehCh.rank(head);
                sourceRanks.push_back(headRank);
            }

            assert(sourceRanks.size() > 0);

            for (const auto &loc: unionOfTransferPointLocs) {
                const auto tail = inputGraph.edgeTail(loc);
                const auto tailRank = vehCh.rank(tail);
                const auto offset = inputGraph.travelTime(loc);
                targetRanks.push_back(tailRank);
                targetOffsets.push_back(offset);
            }

            assert(targetRanks.size() > 0);

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks, targetOffsets);
            ++ptAnyToAny_numSearchesRun;
            ptAnyToAny_sumNumSources += sourceRanks.size();
            ptAnyToAny_sumNumTargets += targetRanks.size();
            ptAnyToAny_numVerticesSettled += vehChQuery.getNumVerticesSettled();
            ptAnyToAny_numEdgeRelaxations += vehChQuery.getNumEdgeRelaxations();
            ptAnyToAny_numResultZero += (lb == 0);

            return lb;
        }

        int calculateLowerBoundPairedTD(const Vehicle *dVeh) {
            std::vector<int> sourceRanks;
            std::vector<int> targetRanks;
            std::vector<int> targetOffsets;

            edgesSubset.clear();
            for (const auto &dropoff: relBNSDropoffs.relevantSpotsFor(dVeh->vehicleId)) {
                edgesSubset.insert(requestState.dropoffs[dropoff.pdId].loc);
            }

            for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVeh->vehicleId)) {
                edgesSubset.insert(requestState.dropoffs[dropoff.pdId].loc);
            }

            for (const auto& dropoffEdge : edgesSubset) {
                const auto tail = inputGraph.edgeTail(dropoffEdge);
                const auto tailRank = vehCh.rank(tail);
                const auto offset = inputGraph.travelTime(dropoffEdge);
                targetRanks.push_back(tailRank);
                targetOffsets.push_back(offset);
            }

            // We dont have to consider the als dropoffs, because a paired assignemnt with dropoff als would imply that the transfer is als

            for (const auto &loc: unionOfTransferPointLocs) {
                const auto head = inputGraph.edgeHead(loc);
                const auto headRank = vehCh.rank(head);
                sourceRanks.push_back(headRank);
            }

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks, targetOffsets);
            ++tdAnyToAny_numSearchesRun;
            tdAnyToAny_sumNumSources += sourceRanks.size();
            tdAnyToAny_sumNumTargets += targetRanks.size();
            tdAnyToAny_numVerticesSettled += vehChQuery.getNumVerticesSettled();
            tdAnyToAny_numEdgeRelaxations += vehChQuery.getNumEdgeRelaxations();
            tdAnyToAny_numResultZero += (lb == 0);

            return lb;
        }

        std::vector<EdgeInEllipse>
        convertVertexEllipseIntoEdgeEllipse(const std::vector<VertexInEllipse> &vertexEllipse, const int leeway) {
            std::vector<EdgeInEllipse> result;
            result.reserve(vertexEllipse.size() * 2);

            for (auto vertexInEllipse: vertexEllipse) {
                distanceFromVertexToNextStop[vertexInEllipse.vertex] = vertexInEllipse.distFromVertex;
            }

            for (auto vertexInEllipse: vertexEllipse) {

                FORALL_INCIDENT_EDGES(inputGraph, vertexInEllipse.vertex, e) {

                    const int travelTime = inputGraph.travelTime(e);
                    const int edgeHead = inputGraph.edgeHead(e);
                    const int distToTail = vertexInEllipse.distToVertex;
                    const int distFromHead = distanceFromVertexToNextStop[edgeHead];

                    if (distToTail + travelTime + distFromHead < leeway) {
                        result.push_back(EdgeInEllipse{e, distToTail, distFromHead});
                    }
                }
            }

            distanceFromVertexToNextStop.clear();
            return result;
        }

        std::vector<TransferPoint> getIntersectionOfEllipses(const std::vector<EdgeInEllipse> &ellipsePVeh,
                                                             const std::vector<EdgeInEllipse> &ellipseDVeh,
                                                             const Vehicle *pVeh, const Vehicle *dVeh,
                                                             const int stopIdxPVeh, const int stopIdxDVeh) {
            std::vector<TransferPoint> result;
            int posPVeh = 0, posDVeh = 0;

            const auto numEdgesPVeh = ellipsePVeh.size();
            const auto numEdgesDVeh = ellipseDVeh.size();
            while (posPVeh < numEdgesPVeh && posDVeh < numEdgesDVeh) {
                if (ellipsePVeh[posPVeh].edge < ellipseDVeh[posDVeh].edge) {
                    posPVeh++;
                    continue;
                }

                if (ellipsePVeh[posPVeh].edge > ellipseDVeh[posDVeh].edge) {
                    posDVeh++;
                    continue;
                }

                const auto edgePVeh = ellipsePVeh[posPVeh];
                const auto edgeDVeh = ellipseDVeh[posDVeh];
                const int loc = edgePVeh.edge;


                const int distPVehToTransfer = edgePVeh.distToTail + inputGraph.travelTime(loc);
                const int distPVehFromTransfer = edgePVeh.distFromHead;
                const int distDVehToTransfer = edgeDVeh.distToTail + inputGraph.travelTime(loc);
                const int distDVehFromTransfer = edgeDVeh.distFromHead;

                result.emplace_back(loc, pVeh, dVeh, stopIdxPVeh, stopIdxDVeh, distPVehToTransfer, distPVehFromTransfer,
                                    distDVehToTransfer, distDVehFromTransfer);

                ++posPVeh;
                ++posDVeh;
            }

            return result;
        }

        bool assertTransferPointCalculation(const TransferPoint tp) {
            const auto stopLocationsPVeh = routeState.stopLocationsFor(tp.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(tp.dVeh->vehicleId);

            const auto stopIndexPVeh = tp.dropoffAtTransferStopIdx;
            const auto stopIndexDVeh = tp.pickupFromTransferStopIdx;

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

            assert(routeStateLengthPVeh == legLenthPVeh);
            assert(routeStateLengthDVeh == legLenthDVeh);

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


        int pairedLowerBoundPT = INFTY;
        int pairedLowerBoundTD = INFTY;

        std::vector<AssignmentWithTransfer> promisingPartials;

        TransferPointFinderT &tpFinder;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;
        CHEllipseReconstructorT &chEllipseReconstructor;

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
        InsertionAsserterT &asserter;


        TimestampedVector<int> distanceFromVertexToNextStop;
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;

        LightweightSubset unionOfTransferPointLocs;

        std::vector<bool> alsDropoffVehs;
        Subset dVehIdsALS;

        LightweightSubset edgesSubset; // Subset used to deduplicate locations in preparing any-to-any searches

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

        int64_t ptAnyToAny_numSearchesRun;
        int64_t ptAnyToAny_sumNumSources;
        int64_t ptAnyToAny_sumNumTargets;
        int64_t ptAnyToAny_numVerticesSettled;
        int64_t ptAnyToAny_numEdgeRelaxations;
        int64_t ptAnyToAny_numResultZero;

        int64_t tdAnyToAny_numSearchesRun;
        int64_t tdAnyToAny_sumNumSources;
        int64_t tdAnyToAny_sumNumTargets;
        int64_t tdAnyToAny_numVerticesSettled;
        int64_t tdAnyToAny_numEdgeRelaxations;
        int64_t tdAnyToAny_numResultZero;

        std::ofstream &ellipsesLogger;
        std::ofstream &ellipseIntersectionLogger;

    };
}
