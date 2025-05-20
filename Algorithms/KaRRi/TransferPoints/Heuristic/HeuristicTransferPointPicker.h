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

#include <vector>
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/BucketEntry.h"
#include "Algorithms/Buckets/DynamicBucketContainer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"

namespace karri {

    // Given a heuristic sample of edges that comprise eligible transfer locations and a request, this class
    // heuristically computes a fixed-size subset of locations that should be considered as transfer points
    // for the request.
    template<typename InputGraphT, typename CHEnvT>
    class HeuristicTransferPointPicker {

        struct GenerateBucketEntry {

            GenerateBucketEntry(int &curEligibleIdx, DynamicBucketContainer<BucketEntry> &buckets)
                    : curEligibleIdx(curEligibleIdx), buckets(buckets) {}

            template<typename DistLabelT, typename DistContainerT>
            bool operator()(const int v, const DistLabelT &dist, const DistContainerT &) {
                buckets.insert(v, BucketEntry(curEligibleIdx, dist[0]));
                return false;
            }

        private:
            int &curEligibleIdx;
            DynamicBucketContainer<BucketEntry> &buckets;
        };

        using GenerateEntriesLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using GenerateEntriesSearch = typename CHEnvT::template UpwardSearch<dij::NoCriterion, GenerateBucketEntry, GenerateEntriesLabelSet>;


        using AnyToManyLabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        using AnyPdLocToEligibleQuery = typename CHEnvT::template UpwardSearch<dij::NoCriterion, dij::NoCriterion, AnyToManyLabelSet>;


    public:

        HeuristicTransferPointPicker(const InputGraphT &inputGraph,
                                     const CHEnvT &chEnv,
                                     const std::vector<int> &eligible)
                : inputGraph(inputGraph),
                  ch(chEnv.getCH()),
                  eligible(eligible),
                  indices(eligible.size()),
                  distSums(eligible.size()),
                  firstSourceBucketEntry(inputGraph.numVertices() + 1),
                  firstTargetBucketEntry(inputGraph.numVertices() + 1),
                  sourceBucketEntries(),
                  targetBucketEntries(),
                  distancesFromPickups(eligible.size(), INFTY),
                  distancesToDropoffs(eligible.size(), INFTY),
                  pickupQuery(chEnv.template getForwardSearch<dij::NoCriterion, dij::NoCriterion, AnyToManyLabelSet>()),
                  dropoffQuery(chEnv. template getReverseSearch<dij::NoCriterion, dij::NoCriterion, AnyToManyLabelSet>()) {
            for (auto i = 0; i < eligible.size(); ++i)
                indices[i] = i;

            // Construct buckets for eligible edges
            constructSourceBuckets(chEnv);
            constructTargetBuckets(chEnv);
        }

        // Given pickups and dropoffs for a request, this function selects a fixed number of
        // transfer point locations (edges) that should be considered for the request.
        template<typename PDLocs>
        std::vector<int> pickTransferPointLocations(const PDLocs &pickups, const PDLocs &dropoffs, const int n) {
            KASSERT(n >= 0);
            if (n == 0)
                return {};
            if (n >= eligible.size())
                return eligible;

            KASSERT(std::all_of(distancesFromPickups.begin(), distancesFromPickups.end(), [](int d) { return d == INFTY; }));
            KASSERT(std::all_of(distancesToDropoffs.begin(), distancesToDropoffs.end(), [](int d) { return d == INFTY; }));
            runPickupQuery(pickups);
            runDropoffQuery(dropoffs);

            // Pick n eligible edges with the smallest sum of distance from any pickup and to any dropoff.
            for (int i = 0; i < eligible.size(); ++i) {
                KASSERT(distancesFromPickups[i] != INFTY);
                KASSERT(distancesToDropoffs[i] != INFTY);
                distSums[i] = distancesFromPickups[i] + distancesToDropoffs[i];
                distancesFromPickups[i] = INFTY;
                distancesToDropoffs[i] = INFTY;
            }
            std::partial_sort(indices.begin(), indices.begin() + n, indices.end(),
                              [&](int a, int b) { return distSums[a] < distSums[b]; });

            std::vector<int> selectedTransferPoints;
            for (int i = 0; i < n; ++i)
                selectedTransferPoints.push_back(eligible[indices[i]]);

            return selectedTransferPoints;
        }


    private:

        // Run forward BCH query rooted at all pickups to find the smallest distance from any pickup to e
        // for each eligible edge e.
        template<typename PDLocs>
        void runPickupQuery(const PDLocs &pickups) {
            std::vector<int> pdLocHeads;
            std::vector<int> offsets;
            for (const auto &p: pickups) {
                pdLocHeads.push_back(ch.rank(inputGraph.edgeHead(p.loc)));
                offsets.push_back(0);
            }

            // Any-to-many BCH query
            pickupQuery.init(pdLocHeads, offsets); // Init for many sources
            while (!pickupQuery.queue.empty()) { // Settle vertices until exhaustion
                const int v = pickupQuery.settleNextVertex();
                const int distFromPickupToV = pickupQuery.getDistance(v);
                for (auto i = firstSourceBucketEntry[v]; i < firstSourceBucketEntry[v + 1]; ++i) {
                    const auto &entry = sourceBucketEntries[i];
                    const auto newDist = distFromPickupToV + entry.distToTarget;
                    distancesFromPickups[entry.targetId] = std::min(distancesFromPickups[entry.targetId], newDist);
                }
            }
        }

        // Run reverse BCH query rooted at all dropoffs to find the smallest distance from e to any dropoff
        // for each eligible edge e.
        template<typename PDLocs>
        void runDropoffQuery(const PDLocs &dropoffs) {
            std::vector<int> pdLocTails;
            std::vector<int> offsets;
            for (const auto &d: dropoffs) {
                pdLocTails.push_back(ch.rank(inputGraph.edgeTail(d.loc)));
                offsets.push_back(inputGraph.travelTime(d.loc));
            }

            // Many-to-any BCH query
            dropoffQuery.init(pdLocTails, offsets); // Init for many sources
            while (!dropoffQuery.queue.empty()) { // Settle vertices until exhaustion
                const int v = dropoffQuery.settleNextVertex();
                const int distFromVToDropoff = dropoffQuery.getDistance(v);
                for (auto i = firstTargetBucketEntry[v]; i < firstTargetBucketEntry[v + 1]; ++i) {
                    const auto &entry = targetBucketEntries[i];
                    const auto newDist = entry.distToTarget + distFromVToDropoff;
                    distancesToDropoffs[entry.targetId] = std::min(distancesToDropoffs[entry.targetId], newDist);
                }
            }
        }

        void constructSourceBuckets(const CHEnvT& chEnv) {
            // Construct buckets using dynamic bucket container
            DynamicBucketContainer<BucketEntry> dynamicSourceBuckets(inputGraph.numVertices());
            int curEligibleIdx = -1;
            GenerateEntriesSearch generateSourceEntries = chEnv.template getForwardSearch<dij::NoCriterion, GenerateBucketEntry, GenerateEntriesLabelSet>({}, {curEligibleIdx, dynamicSourceBuckets});
            for (int i = 0; i < eligible.size(); ++i) {
                const auto e = eligible[i];
                const auto headRank = ch.rank(inputGraph.edgeHead(e));
                curEligibleIdx = i;
                generateSourceEntries.run(headRank);
            }

            // Buckets no longer change from now. Convert dynamic bucket container to static representation that is
            // better suited for queries.
            convertToStaticBuckets(dynamicSourceBuckets, firstSourceBucketEntry, sourceBucketEntries);
        }

        void constructTargetBuckets(const CHEnvT& chEnv) {

            // Construct buckets using dynamic bucket container
            DynamicBucketContainer<BucketEntry> dynamicTargetBuckets(inputGraph.numVertices());
            int curEligibleIdx = -1;
            GenerateEntriesSearch generateTargetEntries = chEnv.template getReverseSearch<dij::NoCriterion, GenerateBucketEntry, GenerateEntriesLabelSet>({}, {curEligibleIdx, dynamicTargetBuckets});
            for (int i = 0; i < eligible.size(); ++i) {
                const auto e = eligible[i];
                const auto tailRank = ch.rank(inputGraph.edgeTail(e));
                const int offset = inputGraph.travelTime(e);
                curEligibleIdx = i;
                generateTargetEntries.runWithOffset(tailRank, offset);
            }

            // Buckets no longer change from now. Convert dynamic bucket container to static representation that is
            // better suited for queries.
            convertToStaticBuckets(dynamicTargetBuckets, firstTargetBucketEntry, targetBucketEntries);
        }

        // Convert dynamic bucket container used during construction of buckets to static representation that is
        // better suited for queries. Buckets for vertices are stored consecutively in memory without gaps and each
        // bucket is internally sorted by target ID.
        void convertToStaticBuckets(const DynamicBucketContainer<BucketEntry> &dynamic,
                                    std::vector<int> &firstBucketEntry,
                                    std::vector<BucketEntry> &bucketEntries) {
            for (int r = 0; r < inputGraph.numVertices(); ++r) {
                firstBucketEntry[r] = bucketEntries.size();
                const auto &bucket = dynamic.getBucketOf(r);
                bucketEntries.insert(bucketEntries.end(), bucket.begin(), bucket.end());
                std::sort(bucketEntries.begin() + firstBucketEntry[r], bucketEntries.end(),
                          [](const BucketEntry &a, const BucketEntry &b) { return a.targetId < b.targetId; });
            }
            firstBucketEntry[inputGraph.numVertices()] = bucketEntries.size();
        }

        const InputGraphT &inputGraph;
        const CH &ch;
        const std::vector<int> &eligible;

        std::vector<int> indices; // Contains indices in interval [0 .. eligible.size() - 1]
        std::vector<int> distSums;

        // Static buckets for BCH queries. The bucket entries for vertex v are stored
        // at xBucketEntries[firstXBucketEntry[v] .. firstXBucketEntry[v + 1] - 1].
        std::vector<int> firstSourceBucketEntry;
        std::vector<int> firstTargetBucketEntry;
        std::vector<BucketEntry> sourceBucketEntries;
        std::vector<BucketEntry> targetBucketEntries;

        std::vector<int> distancesFromPickups;
        std::vector<int> distancesToDropoffs;
        AnyPdLocToEligibleQuery pickupQuery;
        AnyPdLocToEligibleQuery dropoffQuery;
    };
}