/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
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

#include <array>
#include <cstdint>
#include <type_traits>
#include <vector>

#include "Algorithms/CH/CH.h"
#include "Algorithms/Dijkstra/BiDijkstra.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/TraversalCostAttribute.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Labels/Containers/StampedDistanceLabelContainer.h"
#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/Constants.h"

// Implementation of a CH query. Depending on the used label set, it keeps parent vertices and/or
// edges, and computes multiple shortest paths simultaneously, optionally using SSE or AVX
// instructions. The algorithm can be used with different distance label containers and queues.
template<
        typename LabelSetT, bool USE_STALLING = true,
        template<typename> class DistanceLabelContainerT = StampedDistanceLabelContainer,
        typename QueueT = AddressableQuadHeap>
class CHQuery {
private:
    static constexpr int K = LabelSetT::K; // The number of simultaneous shortest-path computations.
    using DistLabel = typename LabelSetT::DistanceLabel; // The distance label of a vertex.

public:
    // The pruning criterion for a CH query, also known as stall-on-demand.
    struct PruningCriterion {
        using LabelMask = typename LabelSetT::LabelMask;     // Marks a subset of components in a label.

        // Constructs a pruning criterion for a CH query.
        PruningCriterion(const CH::SearchGraph &oppositeGraph) noexcept
                : oppositeGraph(oppositeGraph) {}

        // Returns true if the search can be pruned at v.
        template<typename DistLabelContainerT>
        bool operator()(const int v, const DistLabel &distToV, DistLabelContainerT &distLabels) const {
            LabelMask continueSearch = true;
            FORALL_INCIDENT_EDGES(oppositeGraph, v, e)continueSearch &=
                                                              distToV < distLabels[oppositeGraph.edgeHead(e)] +
                                                                        oppositeGraph.traversalCost(e);
            return !anySet(continueSearch);
        }

        const CH::SearchGraph &oppositeGraph; // The down (up) graph if we prune the up (down) search.
    };

    // Constructs a CH query instance that uses stall-on-demand.
    template<bool COND = USE_STALLING>
    explicit CHQuery(std::enable_if_t<COND, const CH &> ch)
            : search(ch.upwardGraph(), ch.downwardGraph(), {ch.downwardGraph()}, {ch.upwardGraph()}) {}

    // Constructs a CH query instance that does not use stall-on-demand.
    template<bool COND = USE_STALLING>
    explicit CHQuery(std::enable_if_t<!COND, const CH &> ch)
            : search(ch.upwardGraph(), ch.downwardGraph()) {}

    // Runs a CH query from s to t.
    void run(const int s, const int t) {
        search.run(s, t);
        numVerticesSettled = search.getNumVerticesSettled();
        numEdgesRelaxed = search.getNumEdgeRelaxations();
    }

    // Runs a CH query that computes multiple shortest paths simultaneously.
    void run(const std::array<int, K> &sources, const std::array<int, K> &targets) {
        search.run(sources, targets);
        numVerticesSettled = search.getNumVerticesSettled();
        numEdgesRelaxed = search.getNumEdgeRelaxations();
    }

    int runAnyShortestPath(const std::vector<int> &sources, const std::vector<int> &targets, const std::vector<int>& targetOffsets) {
        const int result = search.runAnyShortestPath(sources, targets, targetOffsets);
        numVerticesSettled = search.getNumVerticesSettled();
        numEdgesRelaxed = search.getNumEdgeRelaxations();
        return result;
    }

    // Used for dropoff als / transfer als
    std::vector<int> runOneToMany(const int source, const std::vector<int> &targets, const std::vector<int> &offsets) {
        int totalNumVerticesSettled = 0;
        int totalNumEdgesRelaxed = 0;
        std::vector<int> distances = std::vector<int>{};

        for (int i = 0; i < targets.size(); i += K) {
            // Construct sources / target arrays for the search of K simultaneous searches
            std::array<int, K> sourcesSearch;
            std::array<int, K> targetsSearch;

            const int elementsLeft = targets.size() - i * K;

            if (elementsLeft < K) {
                for (int j = 0; j < elementsLeft; j++) {
                    sourcesSearch[j] = source;
                    targetsSearch[j] = targets[i + j];
                }

                for (int j = elementsLeft; j < K; j++) {
                    sourcesSearch[j] = source;
                    targetsSearch[j] = targets[0];
                }


            } else {
                for (int j = 0; j < K; j++) {
                    sourcesSearch[j] = source;
                    targetsSearch[j] = targets[i + j];
                }
            }

            // Run the search
            run(sourcesSearch, targetsSearch);
            totalNumEdgesRelaxed += numEdgesRelaxed;
            totalNumVerticesSettled += numVerticesSettled;
            const auto distancesSearch = getAllDistances();

            for (int j = 0; j < K; j++) {
                distances.push_back(distancesSearch[j] + offsets[i + j]);
            }
        }

        numEdgesRelaxed = totalNumEdgesRelaxed;
        numVerticesSettled = totalNumVerticesSettled;

        return distances;
    }

    // Used for dropoff als / transfer als
    std::vector<int> runManyToOne(const std::vector<int> &sources, const int target, const int offset) {
        int totalNumVerticesSettled = 0;
        int totalNumEdgesRelaxed = 0;
        std::vector<int> distances = std::vector<int>{};

        for (int i = 0; i < sources.size(); i += K) {
            // Construct sources / target arrays for the search of K simultaneous searches
            std::array<int, K> sourcesSearch;
            std::array<int, K> targetsSearch;

            const int elementsLeft = sources.size() - i * K;

            if (elementsLeft < K) {
                for (int j = 0; j < elementsLeft; j++) {
                    sourcesSearch[j] = sources[i + j];
                    targetsSearch[j] = target;
                }

                for (int j = elementsLeft; j < K; j++) {
                    sourcesSearch[j] = sources[0];
                    targetsSearch[j] = target;
                }


            } else {
                for (int j = 0; j < K; j++) {
                    sourcesSearch[j] = sources[i + j];
                    targetsSearch[j] = target;
                }
            }

            // Run the search
            run(sourcesSearch, targetsSearch);
            totalNumEdgesRelaxed += numEdgesRelaxed;
            totalNumVerticesSettled += numVerticesSettled;

            const auto distancesSearch = getAllDistances();

            for (int j = 0; j < K; j++) {
                distances.push_back(distancesSearch[j] + offset);
            }
        }

        numEdgesRelaxed = totalNumEdgesRelaxed;
        numVerticesSettled = totalNumVerticesSettled;

        return distances;
    }

    // Returns the length of the i-th shortest path.
    int getDistance(const int i = 0) {
        return search.getDistance(i);
    }

    DistLabel getAllDistances() {
        return search.getAllDistances();
    }

    // Returns the edges in the upward graph on the up segment of the up-down path (in reverse order).
    const std::vector<int32_t> &getUpEdgePath(const int i = 0) {
        return search.getEdgePathToMeetingVertex(i);
    }

    // Returns the edges in the downward graph on the down segment of the up-down path.
    const std::vector<int32_t> &getDownEdgePath(const int i = 0) {
        return search.getEdgePathFromMeetingVertex(i);
    }

    // Returns the number of vertices settled in the last call to run(), runAnyShortestPath(), runOneToMany(), or runManyToOne().
    int getNumVerticesSettled() const {
        return numVerticesSettled;
    }

    int getNumEdgeRelaxations() const {
        return numEdgesRelaxed;
    }

private:
    // The stopping criterion for a CH query that computes k shortest paths simultaneously. We can
    // stop the forward search as soon as mu_i <= Qf.minKey for all i = 1, ..., k. The reverse search
    // is stopped in the same way.
    template<typename>
    struct StoppingCriterion {
        // Constructs a stopping criterion for a CH query.
        StoppingCriterion(
                const QueueT &forwardQueue, const QueueT &reverseQueue, const int &maxTentativeDistance)
                : forwardQueue(forwardQueue),
                  reverseQueue(reverseQueue),
                  maxTentativeDistance(maxTentativeDistance) {}

        // Returns true if we can stop the forward search.
        bool stopForwardSearch() const {
            return forwardQueue.empty() || maxTentativeDistance <= forwardQueue.minKey();
        }

        // Returns true if we can stop the reverse search.
        bool stopReverseSearch() const {
            return reverseQueue.empty() || maxTentativeDistance <= reverseQueue.minKey();
        }

        const QueueT &forwardQueue;      // The priority queue of the forward search.
        const QueueT &reverseQueue;      // The priority queue of the reverse search.
        const int &maxTentativeDistance; // The largest of all k tentative distances.
    };

    using UpwardSearchStall = Dijkstra<
            CH::SearchGraph, TraversalCostAttribute, LabelSetT, dij::NoCriterion, PruningCriterion,
            DistanceLabelContainerT, QueueT>;
    using UpwardSearchNoStall = Dijkstra<
            CH::SearchGraph, TraversalCostAttribute, LabelSetT, dij::NoCriterion, dij::NoCriterion,
            DistanceLabelContainerT, QueueT>;
    using UpwardSearch = std::conditional_t<USE_STALLING, UpwardSearchStall, UpwardSearchNoStall>;

    BiDijkstra<UpwardSearch, StoppingCriterion> search; // The modified bidirectional search.

    int64_t numVerticesSettled;
    int64_t numEdgesRelaxed;
};
