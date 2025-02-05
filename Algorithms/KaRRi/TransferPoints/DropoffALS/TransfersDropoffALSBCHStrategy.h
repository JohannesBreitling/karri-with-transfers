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


#pragma once

#include "DataStructures/Containers/FastResetFlagArray.h"
#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Algorithms/KaRRi/LastStopSearches/TentativeLastStopDistances.h"

#include "Algorithms/KaRRi/CostCalculator.h"

namespace karri::Transfers {

    template<typename InputGraphT,
            typename CHEnvT,
            typename LastStopBucketsEnvT,
            typename LabelSet>
    struct TransfersDropoffALSBCHStrategy {
    private:


        static constexpr int K = LabelSet::K;
        using LabelMask = typename LabelSet::LabelMask;
        using DistanceLabel = typename LabelSet::DistanceLabel;


        struct DropoffAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = false;

            DropoffAfterLastStopPruner(TransfersDropoffALSBCHStrategy &strat,
                                       const CostCalculator &calc)
                    : strat(strat), calc(calc) {}

            // Returns whether a given distance from a vehicle's last stop to the dropoff cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel /*&distancesToDropoffs*/,
                                                   const bool /*considerWalkingDists*/) const {
                const DistanceLabel costLowerBound = calc.template calcKVehicleIndependentCostLowerBoundsForDALSWithKnownMinDistToDropoff<LabelSet>(
                        0, /* distancesToDropoffs */ 0, 0, strat.requestState);
                return strat.upperBoundCost < costLowerBound;  // No actual pruning because of the transfer
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the dropoff cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToDropoffs needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtDropoffs,
                                                  const DistanceLabel &minDistancesToDropoffs) const {
                return ~((arrTimesAtDropoffs < INFTY) & (minDistancesToDropoffs < INFTY));  // No actual pruning because of the transfer
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int /*vehId*/,
                                                            const DistanceLabel& distancesToDropoffs) {
                return ~(distancesToDropoffs < INFTY);  // No actual pruning because of the transfer
            }

            void updateUpperBoundCost(const int, const DistanceLabel &) {
                // issue: for an upper bound on the cost, we need an upper bound on what the cost for a pickup using this
                // vehicle is (i.e. an upper bound on the detour, trip time and added trip time for existing passengers
                // until the last stop). However, for pickups before the next stop, we can't derive an upper bound on the
                // detour since we only know lower bounds from the elliptic BCH queries
            }

            bool isVehicleEligible(const int /*vehId*/) const {
                return true;  // No actual pruning because of the transfer
            }

        private:
            TransfersDropoffALSBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using DropoffBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, DropoffAfterLastStopPruner, LabelSet>;

    public: 

        TransfersDropoffALSBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculator &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const RouteState &routeState,
                              RequestState &requestState)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  routeState(routeState),
                  requestState(requestState),
                  checkPBNSForVehicle(fleet.size()),
                  vehiclesSeenForDropoffs(fleet.size()),
                  search(lastStopBucketsEnv, lastStopDistances, chEnv, routeState,
                         vehiclesSeenForDropoffs,
                         DropoffAfterLastStopPruner(*this, calculator)),
                  lastStopDistances(fleet.size()) {}

        Subset findDropoffsAfterLastStop() {
            runBchQueries();

            return vehiclesSeenForDropoffs;
        }

        int getDistanceToDropoff(const int vehId, const unsigned int dropoffId) {
            return lastStopDistances.getDistance(vehId, dropoffId);
        }

    private:

        // Run BCH queries that obtain distances from last stops to dropoffs
        void runBchQueries() {
            // Timer timer;

            initDropoffSearches();
            for (unsigned int i = 0; i < requestState.numDropoffs(); i += K)
                runSearchesForDropoffBatch(i);

            // const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            // requestState.stats().dalsAssignmentsStats.searchTime += searchTime;
            // requestState.stats().dalsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            // requestState.stats().dalsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            // requestState.stats().dalsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            // requestState.stats().dalsAssignmentsStats.numCandidateVehicles += vehiclesSeenForDropoffs.size();
        }

        void initDropoffSearches() {
            totalNumEdgeRelaxations = 0;
            totalNumVerticesSettled = 0;
            totalNumEntriesScanned = 0;

            upperBoundCost = requestState.getBestCost();
            vehiclesSeenForDropoffs.clear();

            // Construct more space for dropoff labels if needed.
            const int numDropoffBatches =
                    requestState.numDropoffs() / K + (requestState.numDropoffs() % K != 0);
            lastStopDistances.init(numDropoffBatches);

        }

        void runSearchesForDropoffBatch(const unsigned int firstDropoffId) {
            assert(firstDropoffId % K == 0 && firstDropoffId < requestState.numDropoffs());
            const int batchIdx = firstDropoffId / K;

            std::array<int, K> dropoffTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &dropoff =
                        firstDropoffId + i < requestState.numDropoffs() ? requestState.dropoffs[firstDropoffId + i]
                                                                        : requestState.dropoffs[firstDropoffId];
                dropoffTails[i] = inputGraph.edgeTail(dropoff.loc);
                travelTimes[i] = inputGraph.travelTime(dropoff.loc);
                currentDropoffWalkingDists[i] = dropoff.walkingDist;
            }

            lastStopDistances.setCurBatchIdx(batchIdx);
            search.run(dropoffTails, travelTimes);

            // totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            // totalNumVerticesSettled += search.getNumVerticesSettled();
            // totalNumEntriesScanned += search.getNumEntriesScanned();
        }


        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const RouteState &routeState;
        RequestState &requestState;

        // Flag per vehicle that tells us if we still have to consider a pickup before the next stop of the vehicle.
        FastResetFlagArray<> checkPBNSForVehicle;

        int upperBoundCost;

        // Vehicles seen by any last stop search
        Subset vehiclesSeenForDropoffs;
        DropoffBCHQuery search;
        DistanceLabel currentDropoffWalkingDists;
        TentativeLastStopDistances<LabelSet> lastStopDistances;

        int totalNumEdgeRelaxations;
        int totalNumVerticesSettled;
        int totalNumEntriesScanned;

    };

}