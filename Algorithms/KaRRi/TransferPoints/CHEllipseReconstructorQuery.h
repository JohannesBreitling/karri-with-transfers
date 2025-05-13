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
#include "CHEllipseReconstructorStats.h"
#include "DataStructures/Containers/TimestampedVector.h"
#include "DataStructures/Containers/FastResetFlagArray.h"

#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

namespace karri::TransferPointStrategies {

    // Computes the set of vertices contained in the detour ellipse between a pair of consecutive stops in a vehicle
    // route using bucket entries and a CH topological downward search.
    template<typename EllipticBucketsEnvironmentT, typename LabelSet, typename WeightT = TraversalCostAttribute>
    class CHEllipseReconstructorQuery {

        using DistanceLabel = typename LabelSet::DistanceLabel;
        using LabelMask = typename LabelSet::LabelMask;
        static constexpr int K = LabelSet::K;

    public:

        CHEllipseReconstructorQuery(const CH &ch,
                                    const typename CH::SearchGraph &downGraph,
                                    const typename CH::SearchGraph &upGraph,
                                    const Permutation &topDownRankPermutation,
                                    const std::vector<int>& firstIdxOfLargeLevels,
                                    const EllipticBucketsEnvironmentT &ellipticBucketsEnv,
                                    const RouteState &routeState)
                : ch(ch),
                  numVertices(downGraph.numVertices()),
                  downGraph(downGraph),
                  upGraph(upGraph),
                  topDownRankPermutation(topDownRankPermutation),
                  firstIdxOfLargeLevels(firstIdxOfLargeLevels),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  routeState(routeState),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  verticesInAnyEllipse(),
                  relevantInToSearch(numVertices),
                  relevantInFromSearch(numVertices) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);
            KASSERT(!firstIdxOfLargeLevels.empty() && firstIdxOfLargeLevels.back() == numVertices);
            verticesInAnyEllipse.reserve(numVertices);

            // TODO: padding to align beginning of each large level in distTo, distFrom, relevantInToSearch,
            //  relevantInFromSearch with cache lines and avoid false sharing
        }

        std::array<std::vector<VertexInEllipse>, K> run(const std::array<int, K> &stopIds,
                                                      const int numEllipses, // number of stopIds actually used in batch
                                                      CHEllipseReconstructorStats& stats) {
            KASSERT(numEllipses <= K);

            Timer timer;
            initializeDistanceArrays();
            DistanceLabel leeways = 0;
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesForStopBasedOnBuckets(stopIds[i], i);
                leeways[i] = routeState.leewayOfLegStartingAt(stopIds[i]);
            }
            stats.initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            timer.restart();

            // Process vertices in all small levels sequentially
            for (int r = 0; r < firstIdxOfLargeLevels.front(); ++r) {
                settleVertexInTopodownSearch(r, leeways, verticesInAnyEllipse);
            }

            // Process vertices in each large level in parallel
            for (int l = 0; l < firstIdxOfLargeLevels.size() - 1; ++l) {
                const int firstIdx = firstIdxOfLargeLevels[l];
                const int lastIdx = firstIdxOfLargeLevels[l + 1];
                static constexpr int CHUNK_SIZE = 1 << 10;
                tbb::parallel_for(tbb::blocked_range<int>(firstIdx, lastIdx, CHUNK_SIZE), [&](const tbb::blocked_range<int>& range) {
                    auto& localVerticesInEllipse = threadLocalVerticesInEllipse.local();
                    for (int r = range.begin(); r != range.end(); ++r) {
                        settleVertexInTopodownSearch(r, leeways, localVerticesInEllipse);
                    }
                }, tbb::simple_partitioner());
            }

            for (auto& localVerticesInEllipse : threadLocalVerticesInEllipse) {
                verticesInAnyEllipse.insert(verticesInAnyEllipse.end(), localVerticesInEllipse.begin(),
                                             localVerticesInEllipse.end());
                localVerticesInEllipse.clear();
            }
            std::sort(verticesInAnyEllipse.begin(), verticesInAnyEllipse.end(), [](const int v1, const int v2) {
                return v1 < v2;
            });

//            for (int r = 0; r < numVertices; ++r) {
//                settleVertexInTopodownSearch(r, leeways);
//            }
            stats.topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.numVerticesSettled += numVertices;
            stats.numEdgesRelaxed += upGraph.numEdges() + downGraph.numEdges();

            // Accumulate result per ellipse
            timer.restart();
            std::array<std::vector<VertexInEllipse>, K> ellipses;
            for (auto& ellipse : ellipses)
                ellipse.reserve(verticesInAnyEllipse.size());
            for (const auto &r: verticesInAnyEllipse) {

                const auto &distToVertex = distTo[r];
                const auto &distFromVertex = distFrom[r];
                const auto breaksLeeway = distToVertex + distFromVertex > leeways;
                KASSERT(!allSet(breaksLeeway));
                for (int j = 0; j < numEllipses; ++j) {
                    if (!breaksLeeway[j]) {
                        KASSERT(distToVertex[j] < INFTY && distFromVertex[j] < INFTY);
                        ellipses[j].emplace_back(r, distToVertex[j], distFromVertex[j]);
                    }
                }
            }
            stats.postprocessTime += timer.elapsed<std::chrono::nanoseconds>();

            return ellipses;
        }


    private:

        void initializeDistanceArrays() {
            KASSERT(distTo.size() == distFrom.size());
            while (numVertices > distTo.size()) {
                distTo.resize(std::max(1ul, distTo.size() * 2));
                distFrom.resize(std::max(1ul, distFrom.size() * 2));
            }

            relevantInToSearch.reset();
            relevantInFromSearch.reset();

            verticesInAnyEllipse.clear();
        }

        void initializeDistancesForStopBasedOnBuckets(const int stopId, const int ellipseIdx) {
            KASSERT(ellipseIdx >= 0 && ellipseIdx < K);
            const int vehId = routeState.vehicleIdOf(stopId);
            const int stopIdx = routeState.stopPositionOf(stopId);
            KASSERT(stopIdx < routeState.numStopsOf(vehId) - 1);
            const auto ranksWithSourceBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithSourceBucketEntries(vehId, stopIdx,
                                                                             enumerateBucketEntriesSearchSpace);
            const auto ranksWithTargetBucketEntries =
                    ellipticBucketsEnv.enumerateRanksWithTargetBucketEntries(vehId, stopIdx + 1,
                                                                             enumerateBucketEntriesSearchSpace);

            for (const auto &e: ranksWithSourceBucketEntries) {

                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!relevantInToSearch.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distTo[r] = INFTY;
                }
                relevantInToSearch.set(r);

                distTo[r][ellipseIdx] = e.distance;
            }

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!relevantInFromSearch.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distFrom[r] = INFTY;
                }
                relevantInFromSearch.set(r);

                // Distances for input ranks are initialized here. Distances of other ranks are initialized
                // on-the-fly later.
                distFrom[r][ellipseIdx] = e.distance;
            }
        }

        void settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway, std::vector<int>& verticesInEllipse) {


            auto &distToV = distTo[v];
            if (!relevantInToSearch.isSet(v)) {
                distToV = INFTY;
            }

            FORALL_INCIDENT_EDGES(downGraph, v, e) {
                const auto head = downGraph.edgeHead(e);

                const auto &distToHead = distTo[head];
                const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                distToV.min(distViaHead);
            }

            auto &distFromV = distFrom[v];
            if (!relevantInFromSearch.isSet(v)) {
                distFromV = INFTY;
            }

            FORALL_INCIDENT_EDGES(upGraph, v, e) {
                const auto head = upGraph.edgeHead(e);

                const auto &distFromHead = distFrom[head];
                const auto distViaHead = distFromHead + upGraph.template get<WeightT>(e);
                distFromV.min(distViaHead);
            }

            // If vertex is in any ellipse, store it
            const auto sum = distToV + distFromV;
            const bool allSumGreaterLeeway = allSet(sum > leeway);
            if (!allSumGreaterLeeway) {
                verticesInEllipse.push_back(v);
            }
        }


        const CH &ch;
        const size_t numVertices;
        const CH::SearchGraph &downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        const CH::SearchGraph &upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        const Permutation &topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        const std::vector<int>& firstIdxOfLargeLevels; // First index of large levels in top-down rank ordering.
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        std::vector<int> verticesInAnyEllipse;
        tbb::enumerable_thread_specific<std::vector<int>> threadLocalVerticesInEllipse;

        FastResetFlagArray<> relevantInToSearch; // Flags that mark whether vertex is relevant in to search.
        FastResetFlagArray<> relevantInFromSearch; // Flags that mark whether vertex is relevant in from search.
    };

} // karri
