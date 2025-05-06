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
    template<typename CHEnvT, typename EllipticBucketsEnvironmentT, typename WeightT = TraversalCostAttribute,
            typename LabelSetT = SimdLabelSet<3, ParentInfo::NO_PARENT_INFO>,
            typename LoggerT = NullLogger>
    class CHEllipseReconstructor {

        using DistanceLabel = typename LabelSetT::DistanceLabel;
        using LabelMask = typename LabelSetT::LabelMask;
        static constexpr int K = LabelSetT::K;

        using Query = CHEllipseReconstructorQuery<EllipticBucketsEnvironmentT, LabelSetT, WeightT>;


    public:

        CHEllipseReconstructor(const CHEnvT &chEnv, const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                               const RouteState &routeState)
                : ch(chEnv.getCH()),
                  downGraph(chEnv.getCH().downwardGraph()),
                  upGraph(chEnv.getCH().upwardGraph()),
                  topDownRankPermutation(chEnv.getCH().downwardGraph().numVertices()),
                  query(ch, downGraph, upGraph, topDownRankPermutation, ellipticBucketsEnv, routeState),
                  logger(LogManager<LoggerT>::getLogger("ch_ellipse_reconstruction.csv",
                                                        "num_ellipses,"
                                                        "init_time,"
                                                        "topo_search_time,"
                                                        "postprocess_time,"
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
            const size_t numBatches = numEllipses / K + (numEllipses % K != 0);

            std::vector<std::vector<VertexInEllipse>> ellipses;
            ellipses.resize(numEllipses);
            for (int i = 0; i < numBatches; ++i) {
                std::vector<int> batchStopIds;
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    batchStopIds.push_back(stopIds[i * K + j]);
                }
                auto batchResult = query.run(batchStopIds, queryStats);
                for (int j = 0; j < K && i * K + j < numEllipses; ++j) {
                    KASSERT(std::is_sorted(batchResult[j].begin(), batchResult[j].end(),
                                           [](const VertexInEllipse &v1, const VertexInEllipse &v2) {
                                               return v1.vertex < v2.vertex;
                                           }));
                    ellipses[i * K + j].swap(batchResult[j]);
                }
            }

            const auto totalTime = timer.elapsed<std::chrono::nanoseconds>();

            logger << numEllipses << "," << queryStats.initTime << "," << queryStats.topoSearchTime << ","
                   << queryStats.postprocessTime << "," << totalTime << "," << queryStats.numVerticesSettled << ","
                   << queryStats.numEdgesRelaxed << "\n";

            totalNumVerticesSettled += queryStats.numVerticesSettled;
            totalNumEdgesRelaxed += queryStats.numEdgesRelaxed;
            queryStats.reset();

            return ellipses;
        }


    private:

        const CH &ch;
        CH::SearchGraph downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        CH::SearchGraph upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.

        Permutation topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.

        Query query;
        typename Query::QueryStats queryStats;

        LoggerT &logger;
    };

} // karri
