/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2020 Valentin Buchhold
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>

#include "Tools/Constants.h"

namespace bidij {

// The stopping criterion for a standard bidirectional search that computes k shortest paths. We can
// stop the search as soon as mu_i <= Qf.minKey + Qr.minKey for all i = 1, ..., k.
    template<typename QueueT>
    struct StoppingCriterion {
        // Constructs a stopping criterion for a standard bidirectional search.
        StoppingCriterion(
                const QueueT &forwardQueue, const QueueT &reverseQueue, const int &maxTentativeDistance)
                : forwardQueue(forwardQueue),
                  reverseQueue(reverseQueue),
                  maxTentativeDistance(maxTentativeDistance) {}

        // Returns true if we can stop the forward search.
        bool stopForwardSearch() const {
            return forwardQueue.empty() || reverseQueue.empty() ||
                   maxTentativeDistance <= forwardQueue.minKey() + reverseQueue.minKey();
        }

        // Returns true if we can stop the reverse search.
        bool stopReverseSearch() const {
            return forwardQueue.empty() || reverseQueue.empty() ||
                   maxTentativeDistance <= forwardQueue.minKey() + reverseQueue.minKey();
        }

        const QueueT &forwardQueue;      // The priority queue of the forward search.
        const QueueT &reverseQueue;      // The priority queue of the reverse search.
        const int &maxTentativeDistance; // The largest of all k tentative distances.
    };

}

// Implementation of a bidirectional search. Depending on the underlying Dijkstra implementation, it
// keeps parent vertices and/or edges, and computes multiple shortest paths simultaneously,
// optionally using SSE or AVX instructions. The algorithm can be used with different stopping
// criteria, allowing it to be used as CH query algorithm.
template<
        typename DijkstraT, template<typename> class StoppingCriterionT = bidij::StoppingCriterion>
class BiDijkstra {
private:
    static constexpr int K = DijkstraT::K; // The number of simultaneous shortest-path computations.

    using StoppingCriterion = StoppingCriterionT<typename DijkstraT::Queue>;
    using DistanceLabel = typename DijkstraT::DistanceLabel;
    using ParentLabel = typename DijkstraT::ParentLabel;

    template<typename, typename>
    friend
    class FindPDLocsInRadiusQuery;

public:
    using Graph = typename DijkstraT::Graph; // The graph on which we compute shortest paths.

    // Constructs a bidirectional search instance.
    BiDijkstra(
            const Graph &forwardGraph, const Graph &reverseGraph,
            typename DijkstraT::PruningCriterion pruneForwardSearch = {},
            typename DijkstraT::PruningCriterion pruneReverseSearch = {})
            : forwardSearch(forwardGraph, {}, pruneForwardSearch),
              reverseSearch(reverseGraph, {}, pruneReverseSearch),
              stoppingCriterion(forwardSearch.queue, reverseSearch.queue, maxTentativeDistance) {
        assert(forwardGraph.numVertices() == reverseGraph.numVertices());
    }

    // Move constructor.
    BiDijkstra(BiDijkstra &&other) noexcept
            : forwardSearch(std::move(other.forwardSearch)),
              reverseSearch(std::move(other.reverseSearch)),
              stoppingCriterion(forwardSearch.queue, reverseSearch.queue, maxTentativeDistance),
              tentativeDistances(other.tentativeDistances),
              meetingVertices(other.meetingVertices),
              maxTentativeDistance(other.maxTentativeDistance) {}

    // Runs a bidirectional search from s to t.
    void run(const int s, const int t) {
        std::array<int, K> sources;
        std::array<int, K> targets;
        sources.fill(s);
        targets.fill(t);
        run(sources, targets);
    }

    void runWithTargetOffset(const int s, const int t, const int offsetAtT) {
        std::array<int, K> sources;
        std::array<int, K> targets;
        std::array<int, K> offsets;
        sources.fill(s);
        targets.fill(t);
        offsets.fill(offsetAtT);
        run(sources, targets, offsets);
    }

    // Runs a bidirectional search that computes multiple shortest paths simultaneously.
    void run(const std::array<int, K> &sources, const std::array<int, K> &targets,
             const std::array<int, K> &targetOffsets = {}) {
        forwardSearch.init(sources);
        reverseSearch.init(targets, targetOffsets);
        tentativeDistances = INFTY;
        maxTentativeDistance = INFTY;
        bool advanceForward = false;
        while (!stoppingCriterion.stopForwardSearch() || !stoppingCriterion.stopReverseSearch()) {
            advanceForward = !advanceForward; // Alternate between the forward and reverse search.
            if ((advanceForward && !stoppingCriterion.stopForwardSearch()) || stoppingCriterion.stopReverseSearch())
                updateTentativeDistances(forwardSearch.settleNextVertex());
            else
                updateTentativeDistances(reverseSearch.settleNextVertex());
        }
    }

    int runAnyShortestPath(const std::vector<int> &sources, const std::vector<int> &targets, const std::vector<int>& targetOffsets) {
        std::vector<int> sourceZeroOffsets(sources.size(), 0);
        forwardSearch.init(sources, sourceZeroOffsets);
        reverseSearch.init(targets, targetOffsets);

        tentativeDistances = INFTY;
        maxTentativeDistance = INFTY;
        bool advanceForward = false;

        while (!stoppingCriterion.stopForwardSearch() || !stoppingCriterion.stopReverseSearch()) {
            advanceForward = !advanceForward; // Alternate between the forward and reverse search.
            if ((advanceForward && !stoppingCriterion.stopForwardSearch()) || stoppingCriterion.stopReverseSearch())
                updateTentativeDistances(forwardSearch.settleNextVertex());
            else
                updateTentativeDistances(reverseSearch.settleNextVertex());
        }

        return tentativeDistances[0];
    }

    // Returns the length of the i-th shortest path.
    int getDistance(const int i = 0) {
        return tentativeDistances[i];
    }

    DistanceLabel getAllDistances() {
        return tentativeDistances;
    }

    // Returns the edges in the forward graph on the path to the meeting vertex (in reverse order).
    const std::vector<int32_t> &getEdgePathToMeetingVertex(const int i = 0) {
        assert(tentativeDistances[i] != INFTY);
        return forwardSearch.getReverseEdgePath(meetingVertices.vertex(i), i);
    }

    // Returns the edges in the reverse graph on the path from the meeting vertex.
    const std::vector<int32_t> &getEdgePathFromMeetingVertex(const int i = 0) {
        assert(tentativeDistances[i] != INFTY);
        return reverseSearch.getReverseEdgePath(meetingVertices.vertex(i), i);
    }

    int getNumEdgeRelaxations() const {
        return forwardSearch.getNumEdgeRelaxations() + reverseSearch.getNumEdgeRelaxations();
    }

    int getNumVerticesSettled() const {
        return forwardSearch.getNumVerticesSettled() + reverseSearch.getNumVerticesSettled();
    }

private:
    // Checks whether the path via v improves the tentative distance for any search.
    void updateTentativeDistances(const int v) {
        const auto distances = forwardSearch.distanceLabels[v] + reverseSearch.distanceLabels[v];
        const auto improved = distances < tentativeDistances;
        if (anySet(improved)) {
            meetingVertices.setVertex(v, improved);
            tentativeDistances.setIf(distances, improved);
            maxTentativeDistance = tentativeDistances.horizontalMax();
        }
    }


    DijkstraT forwardSearch;             // The forward search from the source(s).
    DijkstraT reverseSearch;             // The reverse search from the target(s).
    StoppingCriterion stoppingCriterion; // The criterion used to stop the search.

    DistanceLabel tentativeDistances; // One tentative distance per simultaneous search.
    ParentLabel meetingVertices;      // One meeting vertex per simultaneous search.
    int maxTentativeDistance;         // The largest of all k tentative distances.
};
