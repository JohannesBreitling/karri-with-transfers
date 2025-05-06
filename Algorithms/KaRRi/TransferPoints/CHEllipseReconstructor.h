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
                               const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                               const RouteState &routeState)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  routeState(routeState),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  topDownRankPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  query(ch, downGraph, upGraph, topDownRankPermutation, ellipticBucketsEnv, routeState),
                  p2pQuery(chEnv.template getFullCHQuery<P2PLabelSet>()),
                  pathUnpacker(chEnv.getCH()),
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
        }

        // Return the permutation of vertex IDs used by the ellipse reconstructor.
        // This permutation maps vertex IDs in the input graph to vertex IDs in the top-down-permuted CH search graphs.
        // Vertices in ellipses returned by getVerticesInEllipsesOfLegsAfterStops() can be expected to be sorted in
        // increasing order of these vertex IDs.
        Permutation getVertexPermutation() const {
            Permutation temp(upGraph.numVertices());
            for (int v = 0; v < upGraph.numVertices(); ++v)
                temp[v] = topDownRankPermutation[ch.rank(v)];
            KASSERT(temp.validate());
            return temp;
        }

        std::vector<std::vector<VertexInEllipse>>
        getVerticesInEllipsesOfLegsAfterStops(const std::vector<int> &stopIds, int64_t &totalNumVerticesSettled,
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

            // Construct ellipses without leeway by running point-to-point CH queries.
            Timer p2pTimer;
            const size_t numEllipsesWithoutLeeway = indicesWithoutLeeway.size();
            std::vector<std::vector<VertexInEllipse>> ellipsesWithoutLeeway(numEllipsesWithoutLeeway);
            std::vector<int> edgePathInInputGraph;
            for (int i = 0; i < numEllipsesWithoutLeeway; ++i) {
                // Reconstruct shortest path between stops by running point-to-point CH query.
                const int stopId = stopIds[indicesWithoutLeeway[i]];
                const auto stopIdx = routeState.stopPositionOf(stopId);
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
                // Convert vertex IDs to output format of ellipse reconstructor and sort the vertices as expected.
                auto &ellipse = ellipsesWithoutLeeway[i];
                ellipse.reserve(edgePathInInputGraph.size() + 1);
                int distFromFirstStopToHead = 0;
                int distFromHeadToSecondStop = p2pQuery.getDistance() + inputGraph.travelTime(stopLocs[stopIdx + 1]);
                ellipse.emplace_back(topDownRankPermutation[source], distFromFirstStopToHead, distFromHeadToSecondStop);
                for (const auto e: edgePathInInputGraph) {
                    distFromFirstStopToHead += inputGraph.travelTime(e);
                    distFromHeadToSecondStop -= inputGraph.travelTime(e);
                    const auto v = topDownRankPermutation[ch.rank(inputGraph.edgeHead(e))];
                    ellipse.emplace_back(v, distFromFirstStopToHead, distFromHeadToSecondStop);
                }
                KASSERT(ellipse.front().vertex ==
                        topDownRankPermutation[ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]))]);
                KASSERT(ellipse.back().vertex ==
                        topDownRankPermutation[ch.rank(inputGraph.edgeTail(stopLocs[stopIdx + 1]))]);

                std::sort(ellipse.begin(), ellipse.end(), [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                    return v1.vertex < v2.vertex;
                });
            }
            const auto p2pTime = p2pTimer.elapsed<std::chrono::nanoseconds>();

            // Construct ellipses with leeway by running topological downward sweep in CH.
            const size_t numEllipsesWithLeeway = indicesWithLeeway.size();
            const size_t numBatchesWithLeeway = numEllipsesWithLeeway / K + (numEllipsesWithLeeway % K != 0);
            std::vector<std::vector<VertexInEllipse>> ellipsesWithLeeway(numEllipsesWithLeeway);
            for (int i = 0; i < numBatchesWithLeeway; ++i) {
                std::vector<int> batchStopIds;
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    batchStopIds.push_back(stopIds[indicesWithLeeway[i * K + j]]);
                }
                auto batchResult = query.run(batchStopIds, queryStats);
                for (int j = 0; j < K && i * K + j < numEllipsesWithLeeway; ++j) {
                    ellipsesWithLeeway[i * K + j].swap(batchResult[j]);
                }
            }

            // Combine ellipses with and without leeway in order.
            std::vector<std::vector<VertexInEllipse>> ellipses(numEllipses);
            for (int i = 0; i < indicesWithoutLeeway.size(); ++i) {
                const int idx = indicesWithoutLeeway[i];
                ellipses[idx].swap(ellipsesWithoutLeeway[i]);
            }
            for (int i = 0; i < indicesWithLeeway.size(); ++i) {
                const int idx = indicesWithLeeway[i];
                ellipses[idx].swap(ellipsesWithLeeway[i]);
            }

            KASSERT(sanityCheckEllipses(stopIds, ellipses));

            const auto totalTime = timer.elapsed<std::chrono::nanoseconds>();

            logger << numEllipsesWithoutLeeway << "," << numEllipsesWithLeeway << "," << p2pTime << ","
                   << queryStats.initTime << "," << queryStats.topoSearchTime << ","
                   << queryStats.postprocessTime << "," << totalTime << "," << queryStats.numVerticesSettled << ","
                   << queryStats.numEdgesRelaxed << "\n";

            totalNumVerticesSettled += queryStats.numVerticesSettled;
            totalNumEdgesRelaxed += queryStats.numEdgesRelaxed;
            queryStats.reset();

            return ellipses;
        }


    private:

        bool sanityCheckEllipses(const std::vector<int> &stopIds,
                                 const std::vector<std::vector<VertexInEllipse>> &ellipses) {
            for (int i = 0; i < stopIds.size(); ++i) {
                const auto &ellipse = ellipses[i];

                // Make sure ellipse is sorted by increasing vertex ID
                KASSERT(std::is_sorted(ellipse.begin(), ellipse.end(),
                                       [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                                           return v1.vertex < v2.vertex;
                                       }));

                const auto stopId = stopIds[i];
                const auto stopIdx = routeState.stopPositionOf(stopId);
                const auto vehId = routeState.vehicleIdOf(stopId);
                const auto stopLocs = routeState.stopLocationsFor(vehId);
                const auto firstStopVertex = topDownRankPermutation[ch.rank(inputGraph.edgeHead(stopLocs[stopIdx]))];
                const auto secondStopVertex = topDownRankPermutation[ch.rank(
                        inputGraph.edgeTail(stopLocs[stopIdx + 1]))];

                // Make sure ellipse at least contains the two stops
                if (std::find_if(ellipse.begin(), ellipse.end(), [firstStopVertex](const VertexInEllipse &v) {
                    return v.vertex == firstStopVertex;
                }) == ellipse.end()) {
                    KASSERT(false);
                    return false;
                }
                if (std::find_if(ellipse.begin(), ellipse.end(), [secondStopVertex](const VertexInEllipse &v) {
                    return v.vertex == secondStopVertex;
                }) == ellipse.end()) {
                    KASSERT(false);
                    return false;
                }
            }
            return true;
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const RouteState &routeState;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.

        Query query;
        CHEllipseReconstructorStats queryStats;


        P2PQuery p2pQuery;
        CHPathUnpacker pathUnpacker;


        LoggerT &logger;
    };

} // karri
