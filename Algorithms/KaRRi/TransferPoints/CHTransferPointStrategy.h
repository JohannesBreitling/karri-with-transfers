/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "CHEllipseReconstructorQuery.h"

namespace karri::TransferPointStrategies {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename InputGraphT,
            typename CHEnvT,
            typename EllipticBucketsEnvironmentT,
            typename WeightT = TraversalCostAttribute,
            typename LabelSetT = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>,
            typename LoggerT = NullLogger>
    class CHTransferPointStrategy {

        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        static constexpr int K = LabelSetT::K;

        using Query = CHEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSetT, WeightT>;


        using P2PLabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
        using P2PQuery = typename CHEnvT::template FullCHQuery<P2PLabelSet>;

    public:

        CHTransferPointStrategy(const InputGraphT &inputGraph,
                                const CHEnvT &chEnv,
                                const Fleet &fleet,
                                const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                const RequestState &requestState,
                                const RouteState &routeState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  fleet(fleet),
                  requestState(requestState),
                  routeState(routeState),
                  calc(routeState),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  topDownRankPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  inputGraphWithTopDownRankVertexIds(inputGraph),
                  query(ch, downGraph, upGraph, topDownRankPermutation, ellipticBucketsEnv, routeState),
                  p2pQuery(chEnv.template getFullCHQuery<P2PLabelSet>()),
                  pathUnpacker(chEnv.getCH()),
                  distanceFromVertexToNextStop(inputGraph.numVertices(), INFTY),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                        "num_ellipses_without_leeway,"
                                                        "num_ellipses_with_leeway,"
                                                        "p2p_search_time,"
                                                        "topo_init_time,"
                                                        "topo_search_time,"
                                                        "topo_postprocess_time,"
                                                        "total_time,"
                                                        "topo_search_num_vertices_settled,"
                                                        "topo_search_num_edges_relaxed\n")) {
            KASSERT(downGraph.numVertices() == upGraph.numVertices());
            const int numVertices = downGraph.numVertices();
            for (int r = 0; r < numVertices; ++r)
                topDownRankPermutation[r] = numVertices - r - 1;

            downGraph.permuteVertices(topDownRankPermutation);
            upGraph.permuteVertices(topDownRankPermutation);

            Permutation inputGraphVertexIdsToTopDownRank(inputGraph.numVertices());
            for (int i = 0; i < inputGraph.numVertices(); ++i) {
                inputGraphVertexIdsToTopDownRank[i] = topDownRankPermutation[ch.rank(i)];
            }
            KASSERT(inputGraphVertexIdsToTopDownRank.validate());
            inputGraphWithTopDownRankVertexIds.permuteVertices(inputGraphVertexIdsToTopDownRank,
                                                               ellipseEdgeIdsToOriginalEdgeIds);
            ellipseEdgeIdsToOriginalEdgeIds.invert();
        }

        // Given a set of stop IDs for pickup vehicles and dropoff vehicles, this computes the transfer points between
        // any pair of stops in the two sets. The given stop IDs should indicate the first stop in a pair of stops of
        // the respective vehicle.
        void computeTransferPoints(const std::vector<int> &pVehStopIds, const std::vector<int> &dVehStopIds) {

            numVerticesSettled = 0;
            numEdgesRelaxed = 0;

            if (pVehStopIds.empty() || dVehStopIds.empty())
                return;

            std::fill(idxOfStopPVeh.begin(), idxOfStopPVeh.end(), INVALID_INDEX);
            idxOfStopPVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);
            std::fill(idxOfStopDVeh.begin(), idxOfStopDVeh.end(), INVALID_INDEX);
            idxOfStopDVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            numStopsPVeh = 0;
            for (const auto &pVehStopId: pVehStopIds) {
                if (idxOfStopPVeh[pVehStopId] == INVALID_INDEX) {
                    idxOfStopPVeh[pVehStopId] = numStopsPVeh++;
                }
            }

            numStopsDVeh = 0;
            for (const auto &dVehStopId: dVehStopIds) {
                if (idxOfStopDVeh[dVehStopId] == INVALID_INDEX) {
                    idxOfStopDVeh[dVehStopId] = numStopsDVeh++;
                }
            }

            // Compute ellipses
            std::vector<int> allStopIds;
            allStopIds.insert(allStopIds.end(), pVehStopIds.begin(), pVehStopIds.end());
            allStopIds.insert(allStopIds.end(), dVehStopIds.begin(), dVehStopIds.end());
            const auto edgeEllipses = getEdgesInEllipsesOfLegsAfterStops(allStopIds, numVerticesSettled,
                                                                         numEdgesRelaxed);

            // Compute pairwise intersections of ellipses
            if (transferPoints.size() < numStopsPVeh * numStopsDVeh)
                transferPoints.resize(numStopsPVeh * numStopsDVeh);

            // Flatten for parallelization of work
            std::vector<std::pair<int, int>> stopIdPairs;
            stopIdPairs.reserve(numStopsPVeh * numStopsDVeh);
            for (const auto &stopIdPStop: pVehStopIds) {
                const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);

                for (const auto &stopIdDStop: dVehStopIds) {
                    const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);

                    if (vehIdPStop == vehIdDStop)
                        continue;

                    stopIdPairs.emplace_back(stopIdPStop, stopIdDStop);
                }
            }

            // TODO parallelize this loop
            for (const auto &[stopIdPStop, stopIdDStop]: stopIdPairs) {
                const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);
                const auto internalIdxPStop = idxOfStopPVeh[stopIdPStop];
                auto &ellipsePStop = edgeEllipses[internalIdxPStop];

                const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);
                const auto internalIdxDStop = idxOfStopDVeh[stopIdDStop];
                auto &ellipseDStop = edgeEllipses[numStopsPVeh + internalIdxDStop];

                KASSERT(vehIdPStop != vehIdDStop);

                auto &transferPointsForPair = transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
                transferPointsForPair.clear();
                intersectEllipses(ellipsePStop, ellipseDStop, vehIdPStop, vehIdDStop,
                                  routeState.stopPositionOf(stopIdPStop),
                                  routeState.stopPositionOf(stopIdDStop), transferPointsForPair);
            }
        }

        // Given the IDs of the respective first stops in a pair of stops in a pickup vehicle and a pair of stops in a
        // dropoff vehicle, this returns the feasible transfer points for these stop pairs.
        const std::vector<TransferPoint> &getTransferPoints(const int pVehStopId, const int dVehStopId) {
            const auto internalIdxPStop = idxOfStopPVeh[pVehStopId];
            const auto internalIdxDStop = idxOfStopDVeh[dVehStopId];
            KASSERT(internalIdxPStop != INVALID_INDEX && internalIdxDStop != INVALID_INDEX);
            return transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
        }

    private:

        std::vector<std::vector<EdgeInEllipse>>
        getEdgesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds, int64_t &totalNumVerticesSettled,
                                           int64_t &totalNumEdgesRelaxed) {
            if (stopIds.empty())
                return {};

            Timer timer;

            const size_t numEllipses = stopIds.size();

            // Differentiate between stops with and without leeway since the ellipse of stops without leeway is just
            // the shortest path between the stops (i.e. no detour allowed).
            std::vector<int> indicesWithoutLeeway;
            std::vector<int> indicesWithLeeway;
            for (int i = 0; i < numEllipses; ++i) {
                const int stopPos = routeState.stopPositionOf(stopIds[i]);
                const int vehId = routeState.vehicleIdOf(stopIds[i]);
                const int legLength = time_utils::calcLengthOfLegStartingAt(stopPos, vehId, routeState);
                const int leeway = routeState.leewayOfLegStartingAt(stopIds[i]);
                if (leeway == legLength) {
                    indicesWithoutLeeway.push_back(i);
                } else {
                    indicesWithLeeway.push_back(i);
                }
            }

            std::vector<std::vector<EdgeInEllipse>> edgeEllipses(numEllipses);

            // Construct ellipses without leeway by running point-to-point CH queries.
            Timer p2pTimer;
            computeEdgeEllipsesWithoutLeeway(stopIds, indicesWithoutLeeway, edgeEllipses);
            const auto p2pTime = p2pTimer.elapsed<std::chrono::nanoseconds>();

            computeEdgeEllipsesWithLeeway(stopIds, indicesWithLeeway, edgeEllipses);

            const auto totalTime = timer.elapsed<std::chrono::nanoseconds>();
            KASSERT(sanityCheckEdgeEllipses(stopIds, edgeEllipses));

            logger << indicesWithoutLeeway.size() << "," << indicesWithLeeway.size() << "," << p2pTime << ","
                   << queryStats.initTime << "," << queryStats.topoSearchTime << ","
                   << queryStats.postprocessTime << "," << totalTime << "," << queryStats.numVerticesSettled << ","
                   << queryStats.numEdgesRelaxed << "\n";

            totalNumVerticesSettled += queryStats.numVerticesSettled;
            totalNumEdgesRelaxed += queryStats.numEdgesRelaxed;
            queryStats.reset();

            return edgeEllipses;
        }

        void computeEdgeEllipsesWithoutLeeway(const std::vector<int> &stopIds,
                                              const std::vector<int> &indicesWithoutLeeway,
                                              std::vector<std::vector<EdgeInEllipse>> &edgeEllipses) {
            const size_t numEllipsesWithoutLeeway = indicesWithoutLeeway.size();
            std::vector<int> edgePathInInputGraph;
            std::vector<VertexInEllipse> vertexEllipse;
            // TODO: parallelize this loop
            for (int i = 0; i < numEllipsesWithoutLeeway; ++i) {
                // Reconstruct shortest path between stops by running point-to-point CH query.
                // TODO: what if there is more than one shortest path?
                const int stopId = stopIds[indicesWithoutLeeway[i]];
                const auto stopIdx = routeState.stopPositionOf(stopId);
                KASSERT(routeState.leewayOfLegStartingAt(stopId) ==
                        time_utils::calcLengthOfLegStartingAt(stopIdx, routeState.vehicleIdOf(stopId), routeState));
                const auto &stopLocs = routeState.stopLocationsFor(routeState.vehicleIdOf(stopId));
                const auto source = ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]));
                const auto target = ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]));
                p2pQuery.run(source, target);
                KASSERT(p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]) ==
                        routeState.leewayOfLegStartingAt(stopId));
                edgePathInInputGraph.clear();
                pathUnpacker.unpackUpDownPath(p2pQuery.getUpEdgePath(), p2pQuery.getDownEdgePath(),
                                              edgePathInInputGraph);

                // Add all vertices along path to ellipse.
                // Convert vertex IDs to vertex order of ellipse reconstructor and sort the vertices as expected.
                vertexEllipse.clear();
                vertexEllipse.reserve(edgePathInInputGraph.size() + 1);
                int distFromFirstStopToHead = 0;
                int distFromHeadToSecondStop =
                        p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]);
                vertexEllipse.emplace_back(topDownRankPermutation[source], distFromFirstStopToHead,
                                           distFromHeadToSecondStop);
                for (const auto e: edgePathInInputGraph) {
                    distFromFirstStopToHead += inputGraph.travelTime(e);
                    distFromHeadToSecondStop -= inputGraph.travelTime(e);
                    const auto v = topDownRankPermutation[ch.rank(inputGraph.edgeHead(e))];
                    vertexEllipse.emplace_back(v, distFromFirstStopToHead, distFromHeadToSecondStop);
                }
                KASSERT(vertexEllipse.front().vertex ==
                        topDownRankPermutation[ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]))]);
                KASSERT(vertexEllipse.back().vertex ==
                        topDownRankPermutation[ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]))]);

                std::sort(vertexEllipse.begin(), vertexEllipse.end(),
                          [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                              return v1.vertex < v2.vertex;
                          });

                // Convert ellipse of vertices to ellipse of edges.
                auto &edgeEllipse = edgeEllipses[indicesWithoutLeeway[i]];
                convertVertexEllipseIntoEdgeEllipse(vertexEllipse, routeState.leewayOfLegStartingAt(stopId),
                                                    edgeEllipse);
            }
        }

        void computeEdgeEllipsesWithLeeway(const std::vector<int> &stopIds,
                                           const std::vector<int> &indicesWithLeeway,
                                           std::vector<std::vector<EdgeInEllipse>> &edgeEllipses) {
            // Construct ellipses with leeway by running topological downward sweep in CH.
            const size_t numEllipsesWithLeeway = indicesWithLeeway.size();
            const size_t numBatchesWithLeeway = numEllipsesWithLeeway / K + (numEllipsesWithLeeway % K != 0);
            // TODO: parallelize this loop
            for (int i = 0; i < numBatchesWithLeeway; ++i) {
                std::array<int, K> batchStopIds;
                int numEllipsesInBatch = 0;
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    batchStopIds[j] = stopIds[indicesWithLeeway[i * K + j]];
                    ++numEllipsesInBatch;
                }
                const std::array<std::vector<VertexInEllipse>, K> batchResult = query.run(batchStopIds,
                                                                                          numEllipsesInBatch,
                                                                                          queryStats);
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    const auto leeway = routeState.leewayOfLegStartingAt(stopIds[indicesWithLeeway[i * K + j]]);
                    convertVertexEllipseIntoEdgeEllipse(batchResult[j], leeway,
                                                        edgeEllipses[indicesWithLeeway[i * K + j]]);
                }
            }
        }

        void
        convertVertexEllipseIntoEdgeEllipse(const std::vector<VertexInEllipse> &vertexEllipse, const int leeway,
                                            std::vector<EdgeInEllipse> &edgeEllipse) {
            edgeEllipse.reserve(vertexEllipse.size() * 2);

            KASSERT(std::all_of(distanceFromVertexToNextStop.begin(), distanceFromVertexToNextStop.end(),
                                [](const int d) { return d == INFTY; }));

            for (const auto &vertexInEllipse: vertexEllipse)
                distanceFromVertexToNextStop[vertexInEllipse.vertex] = vertexInEllipse.distFromVertex;

            for (const auto &vertexInEllipse: vertexEllipse) {

                FORALL_INCIDENT_EDGES(inputGraphWithTopDownRankVertexIds, vertexInEllipse.vertex, e) {

                    const int travelTime = inputGraphWithTopDownRankVertexIds.travelTime(e);
                    const int edgeHead = inputGraphWithTopDownRankVertexIds.edgeHead(e);
                    const int distToTail = vertexInEllipse.distToVertex;
                    const int distFromHead = distanceFromVertexToNextStop[edgeHead];

                    if (distToTail + travelTime + distFromHead <= leeway) {
                        edgeEllipse.emplace_back(e, distToTail, distFromHead);
                    }
                }
            }

            // Reset distances
            for (const auto &vertexInEllipse: vertexEllipse)
                distanceFromVertexToNextStop[vertexInEllipse.vertex] = INFTY;
        }

        bool sanityCheckEdgeEllipses(const std::vector<int> &stopIds,
                                     const std::vector<std::vector<EdgeInEllipse>> &ellipses) {
            std::vector<int> edgePathInInputGraph;
            for (int i = 0; i < stopIds.size(); ++i) {
                const auto &ellipse = ellipses[i];

                // Make sure ellipse is sorted by increasing vertex ID
                KASSERT(std::is_sorted(ellipse.begin(), ellipse.end(),
                                       [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                                           return e1.edge < e2.edge;
                                       }));

                const auto stopId = stopIds[i];
                const auto stopIdx = routeState.stopPositionOf(stopId);
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopLocs = routeState.stopLocationsFor(vehId);

                // Make sure ellipse at least contains edges on shortest path
                const auto source = ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]));
                const auto target = ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]));
                p2pQuery.run(source, target);
                edgePathInInputGraph.clear();
                pathUnpacker.unpackUpDownPath(p2pQuery.getUpEdgePath(), p2pQuery.getDownEdgePath(),
                                              edgePathInInputGraph);

                for (const auto& eInInputGraph : edgePathInInputGraph) {
                    if (std::find_if(ellipse.begin(), ellipse.end(), [&](const EdgeInEllipse &e) {
                        return ellipseEdgeIdsToOriginalEdgeIds[e.edge] == eInInputGraph;
                    }) == ellipse.end()) {
                        KASSERT(false);
                        return false;
                    }
                }
            }
            return true;
        }

        bool transferPointDominates(const TransferPoint &tp1, const TransferPoint &tp2) {
            const auto detourPVeh1 = tp1.distancePVehToTransfer + tp1.distancePVehFromTransfer;
            const auto detourPVeh2 = tp2.distancePVehToTransfer + tp2.distancePVehFromTransfer;
            const auto detourDVeh1 = tp1.distanceDVehToTransfer + tp1.distanceDVehFromTransfer;
            const auto detourDVeh2 = tp2.distanceDVehToTransfer + tp2.distanceDVehFromTransfer;
            return detourPVeh1 < detourPVeh2 && detourDVeh1 < detourDVeh2;
        }

        // Computes intersection of two ellipses. Ellipses are expected to be sorted by edge IDs in input graph
        // permuted by top down rank vertex IDs.
        // The result is a vector of transfer points specified with edge IDs of the original input graph.
        // The transfer points are checked for pareto-dominance and only non-dominated points are returned.
        void intersectEllipses(const std::vector<EdgeInEllipse> &ellipsePVeh,
                               const std::vector<EdgeInEllipse> &ellipseDVeh,
                               const int pVehId, const int dVehId,
                               const int stopIdxPVeh, const int stopIdxDVeh,
                               std::vector<TransferPoint> &result) {
            result.clear();
            result.reserve(std::max(ellipsePVeh.size(), ellipseDVeh.size()));

            auto itEdgesPVeh = ellipsePVeh.begin();
            auto itEdgesDVeh = ellipseDVeh.begin();
            const auto endEdgesPVeh = ellipsePVeh.end();
            const auto endEdgesDVeh = ellipseDVeh.end();

            while (itEdgesPVeh < endEdgesPVeh && itEdgesDVeh < endEdgesDVeh) {
                const auto &edgePVeh = *itEdgesPVeh;
                const auto &edgeDVeh = *itEdgesDVeh;
                if (edgePVeh.edge < edgeDVeh.edge) {
                    ++itEdgesPVeh;
                    continue;
                }

                if (edgePVeh.edge > edgeDVeh.edge) {
                    ++itEdgesDVeh;
                    continue;
                }

                const int locInPermutedGraph = edgePVeh.edge;
                const int distPVehToTransfer =
                        edgePVeh.distToTail + inputGraphWithTopDownRankVertexIds.travelTime(locInPermutedGraph);
                const int distPVehFromTransfer = edgePVeh.distFromHead;
                const int distDVehToTransfer =
                        edgeDVeh.distToTail + inputGraphWithTopDownRankVertexIds.travelTime(locInPermutedGraph);
                const int distDVehFromTransfer = edgeDVeh.distFromHead;


                ++itEdgesPVeh;
                ++itEdgesDVeh;

                const int locInOriginalGraph = ellipseEdgeIdsToOriginalEdgeIds[locInPermutedGraph];
                const auto tp = TransferPoint(locInOriginalGraph, &fleet[pVehId], &fleet[dVehId], stopIdxPVeh,
                                              stopIdxDVeh, distPVehToTransfer,
                                              distPVehFromTransfer,
                                              distDVehToTransfer, distDVehFromTransfer);

                // Check whether known transfer points dominate tp
                bool dominated = false;
                for (const auto &knownTP: result) {
                    if (transferPointDominates(knownTP, tp)) {
                        dominated = true;
                        break;
                    }
                }
                if (dominated) {
                    continue;
                }

                // If tp is not dominated, add it to result and remove any dominated by tp
                result.push_back(tp);
                for (int i = 0; i < result.size();) {
                    if (transferPointDominates(tp, result[i])) {
                        std::swap(result[i], result.back());
                        result.pop_back();
                        continue;
                    }
                    ++i;
                }
            }

            // TODO: validate that this lower bound is correct
            for (int i = 0; i < result.size();) {
                const auto minTpCost = calc.calcMinCostForTransferPoint(result[i], requestState);
                if (minTpCost.total > requestState.getBestCost()) {
                    std::swap(result[i], result.back());
                    result.pop_back();
                    continue;
                }
                ++i;
            }
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const Fleet &fleet;
        const RequestState &requestState;
        const RouteState &routeState;
        const CostCalculator calc;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.

        // Identical to input graph but vertex IDs permuted by topDownRankPermutation.
        InputGraphT inputGraphWithTopDownRankVertexIds;

        // Permutation that maps edge IDs in permuted input graph to edge IDs in original input graph.
        Permutation ellipseEdgeIdsToOriginalEdgeIds;

        Query query;
        CHEllipseReconstructorStats queryStats;

        P2PQuery p2pQuery;
        CHPathUnpacker pathUnpacker;

        std::vector<int> distanceFromVertexToNextStop;

        int numStopsPVeh;
        int numStopsDVeh;

        // Maps a stop ID to an internal index in the vector of stop IDs.
        std::vector<int> idxOfStopPVeh;
        std::vector<int> idxOfStopDVeh;

        // For two stop IDs i (pVeh) and j (dVeh), transferPoints[idxOfStopPVeh[i] * numStopsDVeh + idxOfStopDVeh[j]] contains the
        // transfer points between the two stops.
        std::vector<std::vector<TransferPoint>> transferPoints;

        int64_t numVerticesSettled;
        int64_t numEdgesRelaxed;
        LoggerT &logger;
    };

} // karri