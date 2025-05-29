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

#include "EdgeEllipseContainer.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/CH/CHPathUnpacker.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "VertexInEllipse.h"
#include "CHEllipseReconstructorQuery.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"

namespace karri {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename InputGraphT,
            typename CHEnvT,
            typename EllipticBucketsEnvironmentT,
            typename WeightT = TraversalCostAttribute,
            typename LabelSetT = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>,
            typename LoggerT = NullLogger>
    class CHEllipseReconstructor {

        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        static constexpr int K = LabelSetT::K;

        using Query = CHEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSetT, WeightT>;


        using P2PLabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;
        using P2PQuery = typename CHEnvT::template FullCHQuery<P2PLabelSet>;

    public:

        CHEllipseReconstructor(const InputGraphT &inputGraph,
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
                  query(ch, downGraph, upGraph, topDownRankPermutation, ellipticBucketsEnv, routeState),
                  p2pQuery(chEnv.template getFullCHQuery<P2PLabelSet>()),
                  pathUnpacker(chEnv.getCH()),
                  distanceFromVertexToNextStop(inputGraph.numVertices(), INFTY),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                        "num_ellipses_without_leeway,"
                                                        "num_ellipses_with_leeway,"
                                                        "init_time,"
                                                        "without_leeway_search_time,"
                                                        "with_leeway_search_time,"
                                                        "total_time,"
                                                        "with_leeway_init_time,"
                                                        "with_leeway_topo_sweep_time,"
                                                        "with_leeway_postprocess_time\n")) {
            KASSERT(downGraph.numVertices() == upGraph.numVertices());
            const int numVertices = downGraph.numVertices();
            for (int r = 0; r < numVertices; ++r)
                topDownRankPermutation[r] = numVertices - r - 1;

            inverseTopDownRankPermutation = topDownRankPermutation.getInversePermutation();

            downGraph.permuteVertices(topDownRankPermutation);
            upGraph.permuteVertices(topDownRankPermutation);
        }

        // Given a set of stop IDs for pickup vehicles and dropoff vehicles, this computes the transfer points between
        // any pair of stops in the two sets. The given stop IDs should indicate the first stop in a pair of stops of
        // the respective vehicle.
        EdgeEllipseContainer computeEllipses(const std::vector<int> &stopIds) {

            Timer timer;

            numVerticesSettled = 0;
            numEdgesRelaxed = 0;

            EdgeEllipseContainer container;
            std::fill(container.idxOfStop.begin(), container.idxOfStop.end(), INVALID_INDEX);
            container.idxOfStop.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            if (stopIds.empty())
                return container;

            int numStops = 0;
            for (const auto &stopId: stopIds) {
                if (container.idxOfStop[stopId] == INVALID_INDEX) {
                    container.idxOfStop[stopId] = numStops++;
                }
            }

            // Compute ellipses
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
            const auto initTime = timer.elapsed<std::chrono::nanoseconds>();

            int64_t withoutLeewayTime = 0, withLeewayTime = 0;
            computeEdgesInEllipsesOfLegsAfterStops(stopIds, indicesWithoutLeeway, indicesWithLeeway, container,
                                                   withoutLeewayTime, withLeewayTime);
            const auto computeEllipsesTime = withoutLeewayTime + withLeewayTime;

            const auto totalTime = initTime + computeEllipsesTime;

            for (const auto &edgeEllipse: container.edgeEllipses) {
                KASSERT(std::is_sorted(edgeEllipse.begin(), edgeEllipse.end(),
                                       [](const EdgeInEllipse &e1, const EdgeInEllipse &e2) {
                                           return e1.edge < e2.edge;
                                       }));
            }

            logger << indicesWithoutLeeway.size() << "," << indicesWithLeeway.size() << "," << initTime << ","
                   << withoutLeewayTime << "," << withLeewayTime << ","
                   << totalTime << "," << queryStats.initTime << "," << queryStats.topoSearchTime << ","
                   << queryStats.postprocessTime << "\n";

            return container;
        }

//        const std::vector<EdgeInEllipse> &getEdgesInEllipse(const int stopId) const {
//            KASSERT(stopId < static_cast<int>(idxOfStop.size()));
//            const auto idx = idxOfStop[stopId];
//            KASSERT(idx != INVALID_INDEX);
//            return edgeEllipses[idx];
//        }

    private:

        void computeEdgesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds,
                                                    const std::vector<int> &indicesWithoutLeeway,
                                                    const std::vector<int> &indicesWithLeeway,
                                                    EdgeEllipseContainer& container,
                                                    int64_t &withoutLeewayTime,
                                                    int64_t &withLeewayTime) {

//            for (auto &ellipse: edgeEllipses)
//                ellipse.clear();

            if (stopIds.empty())
                return;

            container.edgeEllipses.resize(stopIds.size());

            queryStats.reset();
            Timer timer;

            // Construct ellipses without leeway by running point-to-point CH queries.
            computeEdgeEllipsesWithoutLeeway(stopIds, indicesWithoutLeeway, container);
            withoutLeewayTime += timer.elapsed<std::chrono::nanoseconds>();

            timer.restart();
            computeEdgeEllipsesWithLeeway(stopIds, indicesWithLeeway, container);
            withLeewayTime += timer.elapsed<std::chrono::nanoseconds>();

            KASSERT(sanityCheckEdgeEllipses(stopIds, container.edgeEllipses));

            numVerticesSettled += queryStats.numVerticesSettled;
            numEdgesRelaxed += queryStats.numEdgesRelaxed;

            return;
        }

        void computeEdgeEllipsesWithoutLeeway(const std::vector<int> &stopIds,
                                              const std::vector<int> &indicesWithoutLeeway,
                                              EdgeEllipseContainer& container) {
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

                // Add all vertices along path to ellipse, and sort vertices. Then convert back to edges (thus, all
                // edges that shortcut the path with equal distance will be included).
                vertexEllipse.clear();
                vertexEllipse.reserve(edgePathInInputGraph.size() + 1);
                int distFromFirstStopToHead = 0;
                int distFromHeadToSecondStop =
                        p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]);
                vertexEllipse.emplace_back(inputGraph.edgeHead(stopLocs[stopIdx]), distFromFirstStopToHead,
                                           distFromHeadToSecondStop);
                for (const auto e: edgePathInInputGraph) {
                    distFromFirstStopToHead += inputGraph.travelTime(e);
                    distFromHeadToSecondStop -= inputGraph.travelTime(e);
                    const auto v = inputGraph.edgeHead(e);
                    vertexEllipse.emplace_back(v, distFromFirstStopToHead, distFromHeadToSecondStop);
                }
                KASSERT(vertexEllipse.back().vertex == inputGraph.edgeTail(stopLocs[stopIdx + 1]));

                std::sort(vertexEllipse.begin(), vertexEllipse.end(),
                          [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                              return v1.vertex < v2.vertex;
                          });

                // Convert ellipse of vertices to ellipse of edges.
                auto &edgeEllipse = container.edgeEllipses[indicesWithoutLeeway[i]];
                convertVertexEllipseIntoEdgeEllipse(vertexEllipse, routeState.leewayOfLegStartingAt(stopId),
                                                    edgeEllipse);
            }
        }

        void computeEdgeEllipsesWithLeeway(const std::vector<int> &stopIds,
                                           const std::vector<int> &indicesWithLeeway,
                                           EdgeEllipseContainer& container) {
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
                std::array<std::vector<VertexInEllipse>, K> batchResult = query.run(batchStopIds,
                                                                                    numEllipsesInBatch,
                                                                                    queryStats);
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    // transform vertices to original input graph vertex IDs and sort
                    auto &verticesInEllipse = batchResult[j];
                    for (auto &v: verticesInEllipse)
                        v.vertex = ch.contractionOrder(inverseTopDownRankPermutation[v.vertex]);
                    std::sort(verticesInEllipse.begin(), verticesInEllipse.end(),
                              [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                                  return v1.vertex < v2.vertex;
                              });

                    const auto leeway = routeState.leewayOfLegStartingAt(stopIds[indicesWithLeeway[i * K + j]]);
                    convertVertexEllipseIntoEdgeEllipse(verticesInEllipse, leeway,
                                                        container.edgeEllipses[indicesWithLeeway[i * K + j]]);
                }
            }
        }

        // Vertex ellipse must be given using original input graph vertex IDs.
        // Edges in output are edge IDs in original input graph.
        // Vertices should be sorted so that edges are sorted due to sorted inputGraph.
        void convertVertexEllipseIntoEdgeEllipse(const std::vector<VertexInEllipse> &vertexEllipse, const int leeway,
                                                 std::vector<EdgeInEllipse> &edgeEllipse) {
            edgeEllipse.reserve(vertexEllipse.size() * 2);

            KASSERT(std::all_of(distanceFromVertexToNextStop.begin(), distanceFromVertexToNextStop.end(),
                                [](const int d) { return d == INFTY; }));

            for (const auto &vertexInEllipse: vertexEllipse) {
                distanceFromVertexToNextStop[vertexInEllipse.vertex] = vertexInEllipse.distFromVertex;
            }

            for (const auto &vertexInEllipse: vertexEllipse) {

                FORALL_INCIDENT_EDGES(inputGraph, vertexInEllipse.vertex, e) {

                    const int travelTime = inputGraph.travelTime(e);
                    const int edgeHead = inputGraph.edgeHead(e);
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

                // If stopId is at position 0 in the route and the stop at position 1 has just been inserted as an
                // intermediate stop, we never generated target entries for stop at position 0 so the ellipse will be
                // empty. This is okay, since the vehicle is already at this intermediate stop so we can ignore the
                // ellipse.
                if (stopIdx == 0 && routeState.schedArrTimesFor(vehId)[1] == requestState.originalRequest.requestTime)
                    continue;

                const auto source = ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]));
                const auto target = ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]));
                p2pQuery.run(source, target);
                edgePathInInputGraph.clear();
                pathUnpacker.unpackUpDownPath(p2pQuery.getUpEdgePath(), p2pQuery.getDownEdgePath(),
                                              edgePathInInputGraph);

                for (const auto &eInInputGraph: edgePathInInputGraph) {
                    if (std::find_if(ellipse.begin(), ellipse.end(), [&](const EdgeInEllipse &e) {
                        return e.edge == eInInputGraph;
                    }) == ellipse.end()) {
                        KASSERT(false);
                        return false;
                    }
                }
            }
            return true;
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
        Permutation inverseTopDownRankPermutation;

        Query query;
        CHEllipseReconstructorStats queryStats;

        P2PQuery p2pQuery;
        CHPathUnpacker pathUnpacker;

        std::vector<int> distanceFromVertexToNextStop;

        int64_t numVerticesSettled;
        int64_t numEdgesRelaxed;
        LoggerT &logger;
    };

} // karri