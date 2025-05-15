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
#include <tbb/combinable.h>

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
                                    const std::vector<int> &firstIdxOfLargeLevels,
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
                  shiftedIndexForAlignment(downGraph.numVertices()),
                  enumerateBucketEntriesSearchSpace(numVertices),
                  distTo(numVertices, INFTY),
                  distFrom(numVertices, INFTY),
                  relevantInToSearch(numVertices),
                  relevantInFromSearch(numVertices) {
            KASSERT(downGraph.numVertices() == numVertices);
            KASSERT(upGraph.numVertices() == numVertices);
            KASSERT(!firstIdxOfLargeLevels.empty() && firstIdxOfLargeLevels.back() == numVertices);

            int offsetForCurLevelInEntries = 0;
            KASSERT(reinterpret_cast<std::uintptr_t>(&distTo[0]) % CACHE_LINE_SIZE == 0);
            KASSERT(reinterpret_cast<std::uintptr_t>(&distFrom[0]) % CACHE_LINE_SIZE == 0);
            for (int r = 0; r < firstIdxOfLargeLevels.front(); ++r) {
                shiftedIndexForAlignment[r] = r;
            }
            static constexpr int NUM_ENTRIES_PER_CACHE_LINE = CACHE_LINE_SIZE / sizeof(DistanceLabel);
            for (int i = 0; i < firstIdxOfLargeLevels.size() - 1; ++i) {
                const int firstIdx = firstIdxOfLargeLevels[i];
                const int lastIdx = firstIdxOfLargeLevels[i + 1];
                offsetForCurLevelInEntries += (NUM_ENTRIES_PER_CACHE_LINE -
                                               ((firstIdx + offsetForCurLevelInEntries) % NUM_ENTRIES_PER_CACHE_LINE)) %
                                              NUM_ENTRIES_PER_CACHE_LINE;
                for (int r = firstIdx; r < lastIdx; ++r) {
                    shiftedIndexForAlignment[r] = r + offsetForCurLevelInEntries;
                }
            }

            distTo.resize(shiftedIndexForAlignment.back() + 1, INFTY);
            distFrom.resize(shiftedIndexForAlignment.back() + 1, INFTY);

            for (int i = 0; i < firstIdxOfLargeLevels.size() - 1; ++i) {
                const int shiftedFirstIdx = shiftedIndexForAlignment[firstIdxOfLargeLevels[i]];
                KASSERT(shiftedFirstIdx % NUM_ENTRIES_PER_CACHE_LINE == 0);
                KASSERT(reinterpret_cast<std::uintptr_t>(&distTo[shiftedFirstIdx]) % CACHE_LINE_SIZE == 0);
                KASSERT(reinterpret_cast<std::uintptr_t>(&distFrom[shiftedFirstIdx]) % CACHE_LINE_SIZE == 0);
            }


        }

        // Runs a topological downward search in the CH graph to find all vertices in the detour ellipse between
        // each pair of stops in the given batch of numEllipses <= K stop pairs.
        // Returns a set of vertices s.t. every vertex v appears in at least one of the ellipses.
        void run(const std::array<int, K> &stopIds,
                 const DistanceLabel &leeways,
                 const int numEllipses, // number of stopIds actually used in batch
                 std::vector<std::vector<VertexInEllipse>>::iterator firstEllipseInBatch,
                 CHEllipseReconstructorStats &stats) {
            KASSERT(numEllipses <= K);

            Timer timer;
            initializeDistanceArrays();
            for (int i = 0; i < numEllipses; ++i) {
                initializeDistancesForStopBasedOnBuckets(stopIds[i], i);
            }
            stats.initTime += timer.elapsed<std::chrono::nanoseconds>();

            // Run search until queue becomes empty.
            // The number of vertices that need to be settled can be expected to be quite large. Thus, we avoid
            // using a PQ with many costly deleteMin() operations and instead settle every vertex in the graph.
            timer.restart();

            // Process vertices in all small levels sequentially
            auto& verticesInAnyEllipseForSeq = threadLocalVerticesInEllipse.local();
            for (int r = 0; r < firstIdxOfLargeLevels.front(); ++r) {
                settleVertexInTopodownSearch(r, leeways, verticesInAnyEllipseForSeq);
            }

            // Process vertices in each large level in parallel
            for (int l = 0; l < firstIdxOfLargeLevels.size() - 1; ++l) {
                const int firstIdx = firstIdxOfLargeLevels[l];
                const int lastIdx = firstIdxOfLargeLevels[l + 1];
                static constexpr int CHUNK_SIZE = (1 << 2) * CACHE_LINE_SIZE / sizeof(int);
                // Need to use static_partitioner to ensure that the vertices in localVerticesInEllipse are in
                // increasing order.
                tbb::parallel_for(tbb::blocked_range<int>(firstIdx, lastIdx, CHUNK_SIZE),
                                  [&](const tbb::blocked_range<int> &range) {
                                      auto &localVerticesInEllipse = threadLocalVerticesInEllipse.local();
                                      for (int r = range.begin(); r != range.end(); ++r) {
                                          settleVertexInTopodownSearch(r, leeways, localVerticesInEllipse);
                                      }
                                  }, tbb::static_partitioner());
            }
            stats.topoSearchTime += timer.elapsed<std::chrono::nanoseconds>();
            stats.numVerticesSettled += numVertices;
            stats.numEdgesRelaxed += upGraph.numEdges() + downGraph.numEdges();

            // Accumulate result per ellipse
            timer.restart();
            auto verticesInAnyEllipse = threadLocalVerticesInEllipse.combine([](const auto &v1, const auto &v2) {
                KASSERT(std::is_sorted(v1.begin(), v1.end()));
                KASSERT(std::is_sorted(v2.begin(), v2.end()));
                return mergeSortedVectors(v1, v2);
            });
            threadLocalVerticesInEllipse.combine_each([](std::vector<int> &v) { v.clear(); });
            KASSERT(std::is_sorted(verticesInAnyEllipse.begin(), verticesInAnyEllipse.end()));

            for (auto it = firstEllipseInBatch; it != firstEllipseInBatch + numEllipses; ++it) {
                it->reserve(verticesInAnyEllipse.size());
            }

            tbb::parallel_for(0, numEllipses, [&](const int j) {
                const auto leeway = leeways[j];
                auto& ellipse = firstEllipseInBatch[j];
                for (const auto &r: verticesInAnyEllipse) {
                    const int shifted = shiftedIndexForAlignment[r];

                    const int distToVertex = distTo[shifted][j];
                    const int distFromVertex = distFrom[shifted][j];
                    const bool breaksLeeway = distToVertex + distFromVertex > leeway;
                    if (!breaksLeeway) {
                        KASSERT(distToVertex < INFTY && distFromVertex < INFTY);
                        ellipse.emplace_back(r, distToVertex, distFromVertex);
                    }
                }
            });

            stats.postprocessTime += timer.elapsed<std::chrono::nanoseconds>();
        }

        const DistanceLabel& getDistanceLabelFrom(const int v) const {
            return distFrom[shiftedIndexForAlignment[v]];
        }

        const DistanceLabel& getDistanceLabelTo(const int v) const {
            return distTo[shiftedIndexForAlignment[v]];
        }

    private:

        static std::vector<int> mergeSortedVectors(const std::vector<int> &v1, const std::vector<int> &v2) {
            std::vector<int> result;
            result.reserve(v1.size() + v2.size());
            std::merge(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(result));
            return result;
        }

        void initializeDistanceArrays() {
            relevantInToSearch.reset();
            relevantInFromSearch.reset();
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
                    distTo[shiftedIndexForAlignment[r]] = INFTY;
                }
                relevantInToSearch.set(r);

                distTo[shiftedIndexForAlignment[r]][ellipseIdx] = e.distance;
            }

            for (const auto &e: ranksWithTargetBucketEntries) {
                // Map to vertex ordering of CH graphs used
                const auto r = topDownRankPermutation[e.rank];

                if (!relevantInFromSearch.isSet(r)) {
                    // Distances for input ranks are initialized here. Distances of other ranks are initialized
                    // on-the-fly later.
                    distFrom[shiftedIndexForAlignment[r]] = INFTY;
                }
                relevantInFromSearch.set(r);

                // Distances for input ranks are initialized here. Distances of other ranks are initialized
                // on-the-fly later.
                distFrom[shiftedIndexForAlignment[r]][ellipseIdx] = e.distance;
            }
        }

        void
        settleVertexInTopodownSearch(const int v, const DistanceLabel &leeway, std::vector<int> &verticesInEllipse) {


            auto &distToV = distTo[shiftedIndexForAlignment[v]];
            if (!relevantInToSearch.isSet(v)) {
                distToV = INFTY;
            }

            FORALL_INCIDENT_EDGES(downGraph, v, e) {
                const auto head = downGraph.edgeHead(e);

                const auto &distToHead = distTo[shiftedIndexForAlignment[head]];
                const auto distViaHead = distToHead + downGraph.template get<WeightT>(e);
                distToV.min(distViaHead);
            }

            auto &distFromV = distFrom[shiftedIndexForAlignment[v]];
            if (!relevantInFromSearch.isSet(v)) {
                distFromV = INFTY;
            }

            FORALL_INCIDENT_EDGES(upGraph, v, e) {
                const auto head = upGraph.edgeHead(e);

                const auto &distFromHead = distFrom[shiftedIndexForAlignment[head]];
                const auto distViaHead = distFromHead + upGraph.template get<WeightT>(e);
                distFromV.min(distViaHead);
            }

            // If vertex is in any ellipse, store it
            const auto sum = distToV + distFromV;
            const auto breaksLeewayV = sum > leeway;
            const bool allSumGreaterLeeway = allSet(breaksLeewayV);
            if (!allSumGreaterLeeway) {
                verticesInEllipse.push_back(v);
            }
        }


        const CH &ch;
        const size_t numVertices;
        const CH::SearchGraph &downGraph; // Reverse downward edges in CH. Vertices ordered by decreasing rank.
        const CH::SearchGraph &upGraph; // Upward edges in CH. Vertices ordered by decreasing rank.
        const Permutation &topDownRankPermutation; // Maps vertex rank to n - rank in order to linearize top-down passes.
        const std::vector<int> &firstIdxOfLargeLevels; // First index of large levels in top-down rank ordering.
        const EllipticBucketsEnvironmentT &ellipticBucketsEnv;
        const RouteState &routeState;

        // Offset of each vertex in distTo and distFrom arrays. Used to align the large levels with cache lines to
        // avoid false sharing.
        std::vector<int> shiftedIndexForAlignment;

        Subset enumerateBucketEntriesSearchSpace;

        AlignedVector<DistanceLabel> distTo;
        AlignedVector<DistanceLabel> distFrom;
        tbb::combinable<std::vector<int>> threadLocalVerticesInEllipse;

        FastResetFlagArray<> relevantInToSearch; // Flags that mark whether vertex is relevant in to search.
        FastResetFlagArray<> relevantInFromSearch; // Flags that mark whether vertex is relevant in from search.
    };

} // karri
