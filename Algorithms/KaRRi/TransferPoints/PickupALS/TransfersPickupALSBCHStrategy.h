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

#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/LastStopSearches/LastStopBCHQuery.h"
#include "Tools/Timer.h"

namespace karri::Transfers {

    template<typename InputGraphT, typename CHEnvT, typename LastStopBucketsEnvT, typename PDDistancesT, typename LabelSetT>
    class TransfersPickupALSBCHStrategy {

        static constexpr int K = LabelSetT::K;
        using LabelMask = typename LabelSetT::LabelMask;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

        struct PickupAfterLastStopPruner {

            static constexpr bool INCLUDE_IDLE_VEHICLES = true;

            PickupAfterLastStopPruner(TransfersPickupALSBCHStrategy &strat, const CostCalculator &calc)
                    : strat(strat), calc(calc) {}

            // Returns whether a given distance from a vehicle's last stop to the pickup cannot lead to a better
            // assignment than the best known. Uses vehicle-independent lower bounds s.t. if this returns true, then
            // any vehicle with a last stop distance greater than the given one can also never lead to a better
            // assignment than the best known.
            LabelMask doesDistanceNotAdmitBestAsgn(const DistanceLabel &distancesToPickups,
                                                   const bool /* considerPickupWalkingDists = false */) const {
                assert(strat.requestState.minDirectPDDist < INFTY);
                return ~(distancesToPickups < INFTY); // No actual pruning because of the transfer
            }

            // Returns whether a given arrival time and minimum distance from a vehicle's last stop to the pickup cannot
            // lead to a better assignment than the best known. Uses vehicle-independent lower bounds s.t. if this
            // returns true, then any vehicle with an arrival time later than the given one can also never lead to a
            // better assignment than the best known.
            // minDistancesToPickups needs to be a vehicle-independent lower bound on the last stop distance.
            LabelMask doesArrTimeNotAdmitBestAsgn(const DistanceLabel &arrTimesAtPickups,
                                                  const DistanceLabel &minDistancesToPickups) const {
                assert(strat.requestState.minDirectPDDist < INFTY);

                return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                
                if (strat.upperBoundCost >= INFTY) {
                    // If current best is INFTY, only indices i with arrTimesAtPickups[i] >= INFTY or
                    // minDistancesToPickups[i] >= INFTY are worse than the current best.
                    return ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY));
                }
                
                const auto detourTillDepAtPickup = minDistancesToPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                auto depTimeAtPickup = arrTimesAtPickups + DistanceLabel(InputConfig::getInstance().stopTime);
                const auto reqTime = DistanceLabel(strat.requestState.originalRequest.requestTime);
                depTimeAtPickup.max(reqTime);
                const auto tripTimeTillDepAtPickup = depTimeAtPickup - reqTime;
                DistanceLabel costLowerBound = calc.template calcLowerBoundCostForKPairedAssignmentsAfterLastStop<LabelSetT>(
                        detourTillDepAtPickup, tripTimeTillDepAtPickup, 0, 0, strat.requestState);

                costLowerBound.setIf(DistanceLabel(INFTY),
                                     ~((arrTimesAtPickups < INFTY) & (minDistancesToPickups < INFTY)));
                return strat.upperBoundCost < costLowerBound;
            }

            LabelMask isWorseThanBestKnownVehicleDependent(const int /* vehId */, const DistanceLabel &distancesToPickups) {
                return ~(distancesToPickups < INFTY); // No actual pruning because of the transfer
            }

            void updateUpperBoundCost(const int /* vehId */, const DistanceLabel & /*distancesToPickups*/) {
                return; // No actual pruning because of the transfer
            }

            bool isVehicleEligible(const int &) const {
                return true; // All vehicles can perform PALS assignments
            }

        private:
            TransfersPickupALSBCHStrategy &strat;
            const CostCalculator &calc;
        };

        using PickupBCHQuery = LastStopBCHQuery<CHEnvT, LastStopBucketsEnvT, PickupAfterLastStopPruner, LabelSetT>;

    public:

        TransfersPickupALSBCHStrategy(const InputGraphT &inputGraph,
                              const Fleet &fleet,
                              const CHEnvT &chEnv,
                              const CostCalculator &calculator,
                              const LastStopBucketsEnvT &lastStopBucketsEnv,
                              const PDDistancesT &pdDistances,
                              const RouteState &routeState,
                              RequestState &requestState,
                              const int &bestCostBeforeQuery)
                : inputGraph(inputGraph),
                  fleet(fleet),
                  calculator(calculator),
                  pdDistances(pdDistances),
                  routeState(routeState),
                  requestState(requestState),
                  bestCostBeforeQuery(bestCostBeforeQuery),
                  distances(fleet.size()),
                  search(lastStopBucketsEnv, distances, chEnv, routeState, vehiclesSeenForPickups,
                         PickupAfterLastStopPruner(*this, calculator)),
                  vehiclesSeenForPickups(fleet.size()) {}

        int getDistanceToPickup(const int vehId, const unsigned int pickupId) {
            const int distance = distances.getDistance(vehId, pickupId); 
            assert(distance >= 0);
            return distance;
        }
        
        Subset findPickupsAfterLastStop() {
            runBchSearches();
            
            return vehiclesSeenForPickups;
        }

    private:

        // Run BCH searches that find distances from last stops to pickups
        void runBchSearches() {
            // Timer timer;

            initPickupSearches();
            for (int i = 0; i < requestState.numPickups(); i += K)
                runSearchesForPickupBatch(i);

            // const auto searchTime = timer.elapsed<std::chrono::nanoseconds>();
            // requestState.stats().palsAssignmentsStats.searchTime += searchTime;
            // requestState.stats().palsAssignmentsStats.numEdgeRelaxationsInSearchGraph += totalNumEdgeRelaxations;
            // requestState.stats().palsAssignmentsStats.numVerticesOrLabelsSettled += totalNumVerticesSettled;
            // requestState.stats().palsAssignmentsStats.numEntriesOrLastStopsScanned += totalNumEntriesScanned;
            // requestState.stats().palsAssignmentsStats.numCandidateVehicles += vehiclesSeenForPickups.size();
        }

        

        void initPickupSearches() {
            // totalNumEdgeRelaxations = 0;
            // totalNumVerticesSettled = 0;
            // otalNumEntriesScanned = 0;

            upperBoundCost = bestCostBeforeQuery;
            vehiclesSeenForPickups.clear();
            const int numPickupBatches = requestState.numPickups() / K + (requestState.numPickups() % K != 0);
            distances.init(numPickupBatches);
        }

        void runSearchesForPickupBatch(const int firstPickupId) {
            assert(firstPickupId % K == 0 && firstPickupId < requestState.numPickups());


            std::array<int, K> pickupTails;
            std::array<int, K> travelTimes;
            for (int i = 0; i < K; ++i) {
                const auto &pickup =
                        firstPickupId + i < requestState.numPickups() ? requestState.pickups[firstPickupId + i]
                                                                      : requestState.pickups[firstPickupId];
                pickupTails[i] = inputGraph.edgeTail(pickup.loc);
                travelTimes[i] = inputGraph.travelTime(pickup.loc);
                currentPickupWalkingDists[i] = pickup.walkingDist;
                curPassengerArrTimesAtPickups[i] = requestState.getPassengerArrAtPickup(pickup.id);
                curDistancesToDest[i] = pdDistances.getDirectDistance(pickup.id, 0);
            }

            distances.setCurBatchIdx(firstPickupId / K);
            search.run(pickupTails, travelTimes);

            // totalNumEdgeRelaxations += search.getNumEdgeRelaxations();
            // totalNumVerticesSettled += search.getNumVerticesSettled();
            // totalNumEntriesScanned += search.getNumEntriesScanned();
        }

        const InputGraphT &inputGraph;
        const Fleet &fleet;
        const CostCalculator &calculator;
        const PDDistancesT &pdDistances;
        const RouteState &routeState;
        RequestState &requestState;
        const int &bestCostBeforeQuery;

        int upperBoundCost;

        TentativeLastStopDistances<LabelSetT> distances;
        PickupBCHQuery search;

        // Vehicles seen by any last stop pickup search
        Subset vehiclesSeenForPickups;
        DistanceLabel currentPickupWalkingDists;
        DistanceLabel curPassengerArrTimesAtPickups;
        DistanceLabel curDistancesToDest;

        // int totalNumEdgeRelaxations;
        // int totalNumVerticesSettled;
        // int totalNumEntriesScanned;

    };

}
