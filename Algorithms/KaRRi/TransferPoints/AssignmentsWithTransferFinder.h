/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
/// Copyright (c) 2024 Johannes Breitling <johannes.breitling@student.kit.edu>
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


// ! All the ALS Options we want to treat




#pragma once

#include "Algorithms/KaRRi/TransferPoints/TransferPointFinder.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

namespace karri {

    template <typename StrategyT, typename InputGraphT, typename VehCHEnvT, typename CurVehLocToPickupSearchesT, typename DijLabelSet>
    class AssignmentsWithTransferFinder {

    public:
        AssignmentsWithTransferFinder(
            StrategyT strategy,
            const Fleet &fleet, const RouteState &routeState,
            RequestState &requestState,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,
            CostCalculator &calc,
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs,
            TransferPointFinder<StrategyT> &tpFinder,
            std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints,
            const VehicleLocator<InputGraphT, VehCHEnvT> &locator,
            CurVehLocToPickupSearchesT &searches)
                            : strategy(strategy),
                              fleet(fleet),
                              routeState(routeState),
                              requestState(requestState),
                              inputGraph(inputGraph),
                              vehCh(vehChEnv.getCH()),
                              vehChQuery(vehChEnv.template getFullCHQuery<>()),
                              calc(calc),
                              pVehs(pVehs), dVehs(dVehs),
                              pickupDropoffPairs(std::vector<std::tuple<Vehicle, Vehicle>>{}),
                              tpFinder(tpFinder),
                              transferPoints(transferPoints),
                              locator(locator),
                              searches(searches),
                              assignmentsWithUnknownPickupDistance(std::vector<AssignmentWithTransfer>{}),
                              assignmentsWithUnknownTransferDistance(std::vector<AssignmentWithTransfer>{}),
                              calculatedDirectDistances(std::map<std::tuple<int, int>, int>{}),
                              dijSearchLowerBound(inputGraph) {}

        void init() {
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};
            transferPoints = std::map<std::tuple<int, int>, std::vector<TransferPoint>>{};
        }

        // Method to find the best assignment with exactly one transfer, using the best found cost without transfer to prune solutions
        void findBestAssignment() {
            Timer totalTimer;

            auto &stats = requestState.stats().transferStats;
            numPartialsTried = 0;
            numAssignmentsTried = 0;
            numAssignmentsWithUnkownPairedDistanceTried = 0;
            int64_t numPickupDropoffPairs = 0;
            int64_t numStopPairs = 0;

            // Pair up the vehicles
            pairUpVehicles();

            // Loop over all possible pairs of pickup and dropoff vehicles
            for (auto &pickupDropoffPair : pickupDropoffPairs) {
                numPickupDropoffPairs++;

                promisingPartials = std::vector<AssignmentWithTransfer>{};
                
                auto &pVeh = std::get<0>(pickupDropoffPair);
                auto &dVeh = std::get<1>(pickupDropoffPair);

                Timer timer;

                // Calculate the possible transfer points between the stop pairs 
                tpFinder.findTransferPoints(pVeh, dVeh);
                const auto tpsSize = countTransferPoints();

                stats.numTransferPoints += tpsSize;
                const auto transferPointCalculationTime = timer.elapsed<std::chrono::nanoseconds>();
                stats.transferPointCalculationTime += transferPointCalculationTime;
            
                if (tpsSize == 0)
                    continue;

                // Get the possible pickup and dropoff locations for the vehicle. These location are, once again, edges
                ordPickups = pVehs.getOrdPDsForVehicle(pVeh.vehicleId);
                ordDropoffs = dVehs.getOrdPDsForVehicle(dVeh.vehicleId);
                bnsPickups = pVehs.getBnsPDsForVehicle(pVeh.vehicleId);
                bnsDropoffs = dVehs.getBnsPDsForVehicle(dVeh.vehicleId);

                // Calculate lower bounds for paired distances for pickup-transfer and transfer-dropoff
                pairedLowerBoundPT = calculateLowerBoundPairedPT();
                pairedLowerBoundTD = calculateLowerBoundPairedTD();

                unfinishedAssignments = std::vector<AssignmentWithTransfer>{};

                // * ORDINARY TRANSFER (also includes transfer before next stop)
                findAssignmentsWithOrdinaryTransfer(pVeh, dVeh);

                // * TRANSFER AFTER LAST STOP (PVeh)
                // The pickup vehicle picks up the user either bns, ord or als
                // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
                findAssignmentsWithTransferALSPVeh(pVeh, dVeh);

                
                // First approach for transfer als is, that the pickup vehicle drives to a stop position of the dropoff vehicle and uses this position as the transfer point
                // findAssignmentsWithTransferAtStop(pVeh, dVeh); // TODO Das muss logischerweise dann außerhalb der festgelegten Indizes sein

                promisingPartials.clear();
            
                // Try bns / odrinary pickup and transfer als on route of dropoff vehicle
            }

            

            const auto totalTime = totalTimer.elapsed<std::chrono::nanoseconds>();

            stats.totalTime += totalTime;
            stats.numPickupDropoffPairs += numPickupDropoffPairs;
            stats.numStopPairs += numStopPairs;
            stats.numPartialAssignmentsTried += numPartialsTried;
            stats.numAssignmentsTried += numAssignmentsTried;
        }


    private:
        // Method to build up pairs of pickup and dropoff vehicles
        void pairUpVehicles() {
            if (pVehs.getVehicles()->size() == 0 || dVehs.getVehicles()->size() == 0)
                return;

            Timer timer;
            for (const auto &pVeh : *pVehs.getVehicles()) {
                for (const auto &dVeh : *dVehs.getVehicles()) {
                    // If the pickup is the same as the dropoff vehicle, we already tested the assignment without a transfer
                    if (pVeh->vehicleId == dVeh->vehicleId)
                        continue;

                    if (routeState.numStopsOf(pVeh->vehicleId) < 2 || routeState.numStopsOf(dVeh->vehicleId) < 2)
                        continue;

                    pickupDropoffPairs.push_back(std::make_tuple(*pVeh, *dVeh));
                }
            }

            if (pickupDropoffPairs.size() > 0)
                requestState.stats().transferStats.vehiclePairupTime += timer.elapsed<std::chrono::nanoseconds>();

            assert(pickupDropoffPairs.size() <= pVehs.getVehicles()->size() * dVehs.getVehicles()->size());
        }

        std::vector<TransferPoint> getTransferPointsBetweenStops(const Vehicle &pVeh, int pIndex, int dIndex) {

            const auto numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);

            std::vector<TransferPoint> tpsBetweenIndices = std::vector<TransferPoint>{};
            
            for (int p = pIndex; p < numStopsPVeh; p++) {
                for (int d = dIndex; d >= 0; d--) {
                    if (!transferPoints.count({p, d}))
                        continue;

                    for (auto tp : transferPoints[{p, d}]) { 
                        tpsBetweenIndices.push_back(tp);
                    }
                }
            }

            return tpsBetweenIndices;
        }

        int calculateLowerBoundPairedPT() {
            std::vector<int> sources = std::vector<int>{};
            std::vector<int> targets = std::vector<int>{};

            for (const auto &pickup : bnsPickups) {
                const auto pickupEdge = requestState.pickups[pickup.pdLocId].loc;
                const auto head = inputGraph.edgeHead(pickupEdge);
                
                sources.push_back(head);
            }

            for (const auto &pickup : ordPickups) {
                const auto pickupEdge = requestState.pickups[pickup.pdLocId].loc;
                const auto head = inputGraph.edgeHead(pickupEdge);

                sources.push_back(head);
            }

            for (const auto &it: transferPoints) {
                for (const auto &tp : it.second) {
                    const auto tail = inputGraph.edgeTail(tp.loc);

                    targets.push_back(tail);
                }                
            }

            const auto sourceRanks = vehCh.allRanks(sources);
            const auto targetRanks = vehCh.allRanks(targets);

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks);

            // const int lbDij = dijSearchLowerBound.runAnyShortestPath(sources, targets);
            //std::cout << "LB DIJ : " << lbDij << " =? LB CH : " << lb << std::endl;
            
            return lb; 
        }

        int calculateLowerBoundPairedTD() {
            std::vector<int> sources = std::vector<int>{};
            std::vector<int> targets = std::vector<int>{};

            for (const auto &dropoff : bnsDropoffs) {
                const auto dropoffEdge = requestState.dropoffs[dropoff.pdLocId].loc;
                const auto tail = inputGraph.edgeTail(dropoffEdge);
                
                targets.push_back(tail);
            }

            for (const auto &dropoff : ordDropoffs) {
                const auto dropoffEdge = requestState.dropoffs[dropoff.pdLocId].loc;
                const auto tail = inputGraph.edgeTail(dropoffEdge);

                targets.push_back(tail);
            }

            for (const auto &it: transferPoints) {
                for (const auto &tp : it.second) {
                    const auto head = inputGraph.edgeHead(tp.loc);

                    sources.push_back(head);
                }                
            }

            const auto sourceRanks = vehCh.allRanks(sources);
            const auto targetRanks = vehCh.allRanks(targets);

            const int lb = vehChQuery.runAnyShortestPath(sourceRanks, targetRanks);
            
            return lb;
        }

        void tryOrdinaryDropoffs() {
            for (const auto &asgn : promisingPartials) {
                const auto relOrdDropoffs = getOrdDropoffsAfter(asgn.transferIdxDVeh, routeState.numStopsOf(asgn.dVeh->vehicleId));

                for (const auto &dropoff : relOrdDropoffs) {
                    auto newAsgn = AssignmentWithTransfer(asgn);

                    newAsgn.dropoffIdx = dropoff.pdIdx;
                    assert(dropoff.pdIdx < routeState.numStopsOf(newAsgn.dVeh->vehicleId));
                    newAsgn.dropoff = &(requestState.dropoffs[dropoff.pdLocId]);
                    newAsgn.distToDropoff = dropoff.detourToPD;
                    newAsgn.distFromDropoff = dropoff.detourFromPD;
                
                    tryAssignment(newAsgn);
                }
            }
        }
            
        void tryDropoffsBNS() {
            for (const auto &asgn : promisingPartials) {
                if (asgn.transferIdxDVeh != 0)
                    continue;

                for (const auto &dropoff : bnsDropoffs) {
                    assert(dropoff.pdIdx == 0);
                    assert(asgn.transferIdxDVeh == 0);

                    auto newAsgn = AssignmentWithTransfer(asgn);

                    newAsgn.dropoffIdx = 0;
                    newAsgn.dropoff = &(requestState.dropoffs[dropoff.pdLocId]);
                    newAsgn.distToDropoff = dropoff.detourToPD;
                    newAsgn.distFromDropoff = dropoff.detourFromPD;
                
                    tryAssignment(newAsgn);
                }
            }
        }

        void tryDropoffALS(const Vehicle &dVeh) {
            // The start location of the search is the last stop of the dropoff vehicle
            const auto numStops = routeState.numStopsOf(dVeh.vehicleId);
            const auto lastStopLoc = routeState.stopIdsFor(dVeh.vehicleId)[numStops - 1];
            const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
            std::vector<int> travelTimesDropoffs = std::vector<int>{};

            // Calculate the distances to the possible dropoffs
            // Use a individual ch search approach
            const auto source = vehCh.rank(lastStopVertex);

            // We need to consider the dropoffs for the request
            std::vector<int> dropoffVertecies = std::vector<int>{};
            for (const auto &dropoff : requestState.dropoffs) {
                dropoffVertecies.push_back(inputGraph.edgeTail(dropoff.loc));
                travelTimesDropoffs.push_back(inputGraph.travelTime(dropoff.loc));
            }

            const auto targets = vehCh.allRanks(dropoffVertecies);
            
            // Run the searches for all the dropoffs
            std::vector<int> distances = vehChQuery.runOneToMany(source, targets, travelTimesDropoffs);
            assert(distances.size() == requestState.dropoffs.size());

            for (const auto &asgn : promisingPartials) {
                for (int i = 0; i < requestState.dropoffs.size(); i++) {
                    auto newAsgn = AssignmentWithTransfer(asgn);

                    newAsgn.dropoffIdx = numStops - 1;
                    newAsgn.dropoff = &requestState.dropoffs[i];
                    newAsgn.distToDropoff = distances[i];
                    newAsgn.distFromDropoff = 0;
                    tryAssignment(newAsgn);
                }
            }
        }

        void findAssignmentsWithOrdinaryTransfer(Vehicle &pVeh, Vehicle &dVeh) {
            // Loop over the possible transfer index combinations
            for (int trIdxP = 0; trIdxP < routeState.numStopsOf(pVeh.vehicleId) - 1; trIdxP++) {
                // Get the possible ordinary pickups
                const auto relOrdPickups = getOrdPickupsBefore(trIdxP);

                for (int trIdxD = 0; trIdxD < routeState.numStopsOf(dVeh.vehicleId) - 1; trIdxD++) {
                    // Get the relevant transfer points
                    const auto relTransferPoints = transferPoints[{trIdxP, trIdxD}];

                    // Ordinary pickups
                    for (const auto &pickup : relOrdPickups) {
                        for (const auto &tp : relTransferPoints) {
                            AssignmentWithTransfer asgn;
                            asgn.pVeh = &pVeh;
                            asgn.dVeh = &dVeh;
                            asgn.pickup = &requestState.pickups[pickup.pdLocId];
                            asgn.transfer = tp;
                            asgn.distToPickup = pickup.detourToPD;
                            asgn.distFromPickup = pickup.detourFromPD;
                            asgn.distToTransferPVeh = tp.distancePVehToTransfer;
                            asgn.distFromTransferPVeh = tp.distancePVehFromTransfer;
                            asgn.distToTransferDVeh = tp.distanceDVehToTransfer;
                            asgn.distFromTransferDVeh = tp.distanceDVehFromTransfer;

                            asgn.pickupIdx = pickup.pdIdx;
                            asgn.transferIdxPVeh = trIdxP;
                            asgn.transferIdxDVeh = trIdxD;

                            tryPartialAssignment(asgn);
                        }
                    }

                    // Pickups bns
                    for (const auto &pickup : bnsPickups) {
                        for (const auto &tp : relTransferPoints) {                       
                            AssignmentWithTransfer asgn;
                            asgn.pVeh = &pVeh;
                            asgn.dVeh = &dVeh;
                            asgn.pickup = &requestState.pickups[pickup.pdLocId];
                            asgn.transfer = tp;
                            // Use the distance from pickup as a lower bound and locate the vehicle later
                            asgn.distToPickup = pickup.detourToPD;
                            asgn.distFromPickup = pickup.detourFromPD;
                            asgn.distToTransferPVeh = tp.distancePVehToTransfer;
                            asgn.distFromTransferPVeh = tp.distancePVehFromTransfer;
                            asgn.distToTransferDVeh = tp.distanceDVehToTransfer;
                            asgn.distFromTransferDVeh = tp.distanceDVehFromTransfer;
                            
                            asgn.pickupIdx = pickup.pdIdx;
                            assert(asgn.pickupIdx == 0);
                            asgn.transferIdxPVeh = trIdxP;
                            asgn.transferIdxDVeh = trIdxD;

                            tryPartialAssignment(asgn);
                        }
                    }

                    // Pickup als also implies transfer als, so this case is treated separately
                }
            }

            Timer timer;

            if (promisingPartials.size() == 0)
                return;

            tryOrdinaryDropoffs();
            tryDropoffsBNS();
            // tryDropoffALS(dVeh);

            if (unfinishedAssignments.size() > 0)
                finishAssignments(pVeh, dVeh);
        }

        void findAssignmentsWithTransferALSPVeh(Vehicle &pVeh, Vehicle &dVeh) {
            // Find the distances to the possible transfer points (the stops of the dropoff vehicle)
            const auto numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const int sourceEdge = routeState.stopLocationsFor(pVeh.vehicleId)[numStopsPVeh - 1];
            const int sourceVertex = inputGraph.edgeHead(sourceEdge);

            const auto stopLocationsDVeh = routeState.stopLocationsFor(dVeh.vehicleId);

            auto targets = std::vector<int>{};
            auto offsets = std::vector<int>{};
            auto ranks = std::vector<int>{};

            for (const auto stopLocation : stopLocationsDVeh) {
                targets.push_back(inputGraph.edgeTail(stopLocation));
                offsets.push_back(inputGraph.travelTime(stopLocation));
            }

            const auto sourceRank = vehCh.rank(sourceVertex);
            const auto distances = vehChQuery.runOneToMany(sourceRank, ranks, offsets);

            std::vector<TransferPoint> transferPointsALS = std::vector<TransferPoint>{};

            for (int i = 0; i < stopLocationsDVeh.size() - 1; i++) {
                TransferPoint tp;
                tp.pVeh = &pVeh;
                tp.dVeh = &dVeh;
                tp.loc = stopLocationsDVeh[i];
                tp.dropoffAtTransferStopIdx = numStopsPVeh - 1;
                tp.pickupFromTransferStopIdx = i;
                tp.distancePVehToTransfer = distances[i];
                tp.distancePVehFromTransfer = 0;
                tp.distanceDVehToTransfer = 0;
                tp.distanceDVehFromTransfer = 0; // TODO Ist das dann so richtig?

                transferPointsALS.push_back(tp);
            }
            
            // TODO Build partial assignments with the pickups and then try to find dropoffs
        }

        void tryPartialAssignment(AssignmentWithTransfer &asgn) {
            numPartialsTried++;

            if (asgn.pickupIdx == asgn.transferIdxPVeh) {
                // Paired Assignment (pVeh)
                Timer timer;
                
                asgn.distFromPickup = 0;
                asgn.distToTransferPVeh = pairedLowerBoundPT;
                asgn.pickupPairedLowerBoundUsed = true;
                
                const int pLoc = asgn.pickup->loc;
                const int tLoc = asgn.transfer.loc;
                
                const bool distanceKnown = calculatedDirectDistances.count({pLoc, tLoc});
                
                if (distanceKnown) {
                    asgn.distToTransferPVeh = calculatedDirectDistances[{pLoc, tLoc}];
                    asgn.pickupPairedLowerBoundUsed = false;
                }

                const auto pickupPairedTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.pickupPairedTime += pickupPairedTime;
            }

            if (asgn.pickupIdx == 0) {
                Timer timer;

                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
                }

                const auto pickupBnsTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.pickupBnsTime += pickupBnsTime;               
            }

            const bool unfinished = asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed;
            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) are set
            RequestCost pickupVehCost;
            
            if (unfinished) {
                pickupVehCost = calc.calcPartialCostForPVehLowerBound<true>(asgn, requestState);
            } else {
                pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);
            }

            if (pickupVehCost.total > requestState.getBestCost())
                return;

            promisingPartials.push_back(asgn);
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {
            numAssignmentsTried++;

            if (asgn.dropoffIdx == asgn.transferIdxDVeh) {
                // Paired assignment (dVeh)
                Timer timer;

                // Try lower bound for paired assignment
                asgn.distFromTransferDVeh = 0;
                asgn.distToDropoff = pairedLowerBoundTD;
                asgn.dropoffPairedLowerBoundUsed = true;

                const int tLoc = asgn.transfer.loc;
                const int dLoc = asgn.dropoff->loc;

                if (calculatedDirectDistances.count({tLoc, dLoc})) {
                    asgn.distToDropoff = calculatedDirectDistances[{tLoc, dLoc}];
                    asgn.dropoffPairedLowerBoundUsed = false;
                }

                const auto transferPairedTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.transferPairedTime += transferPairedTime;
            }

            if (asgn.transferIdxDVeh == 0) {
                // Transfer BNS
                Timer timer;

                if (!searches.knowsDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc)) {
                    asgn.dropoffBNSLowerBoundUsed = true;
                } else {
                    asgn.distToTransferDVeh = searches.getDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc);
                }
                
                const auto transferBnsTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.transferBnsTime += transferBnsTime;
            }

            const bool unfinished = asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed || asgn.dropoffBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed;


            RequestCost assignmentCost;
            if (unfinished) {
                // std::cout << "CALC BASE LOWER BOUND" << std::endl;
                assignmentCost = calc.calcBaseLowerBound<true>(asgn, requestState);    
            } else {
                std::cout << "CALC BASE" << std::endl;
                assignmentCost = calc.calcBase<true>(asgn, requestState);
            }

            if (assignmentCost.total > requestState.getBestCost())
                return;
            
            if (unfinished) {
                unfinishedAssignments.push_back(asgn);
                return;
            }

            requestState.tryAssignmentWithTransfer(asgn);
        }

        void finishAssignments(Vehicle &pVeh, Vehicle &dVeh) {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate = std::vector<AssignmentWithTransfer>{};
            std::vector<AssignmentWithTransfer> currentlyCalculating = std::vector<AssignmentWithTransfer>{};
            std::vector<AssignmentWithTransfer> temp = std::vector<AssignmentWithTransfer>{};

            // TODO Remove later
            std::cout << "/ - - - - - - - - - - - - - - - - - - - -" << std::endl;
            std::cout << "FINISH ASSIGNMENTS..." << std::endl;
            std::cout << "Unfinished Assignments: " << unfinishedAssignments.size() << std::endl;

            bool unfinished;
            RequestCost total;
            // Start with the pickups with postponed bns distance
            for (auto asgn : unfinishedAssignments) {
                if (asgn.pickupBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
                } else {
                    toCalculate.push_back(asgn);
                }
            }

            std::cout << "Assignments using Pickup Lower Bound BNS: " << currentlyCalculating.size() << std::endl; // TODO Remove later

            if (currentlyCalculating.size() > 0)
                searches.computeExactDistancesVia(pVeh);

            for (auto asgn : currentlyCalculating) {
                assert(searches.knowsDistance(pVeh.vehicleId, asgn.pickup->id));
                const int distance = searches.getDistance(pVeh.vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;
                unfinished = asgn.pickupPairedLowerBoundUsed || asgn.dropoffBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed; 

                if (unfinished) {
                    total = calc.calcBaseLowerBound<true, true>(asgn, requestState);
                } else {
                    total = calc.calcBase<true, true>(asgn, requestState);
                }

                if (total.total > requestState.getBestCost())
                    continue;

                if (!unfinished) {
                    std::cout << "VERBESSERUNG!!!!" << std::endl;
                    requestState.tryAssignmentWithTransfer(asgn);
                    continue;
                }

                toCalculate.push_back(asgn);
            }

            std::cout << "Noch offen (1/4): " << toCalculate.size() << std::endl;

            // Calculate the exact paired distance between pickup and transfer
            currentlyCalculating.clear();

            auto sources = std::vector<int>{};
            auto targets = std::vector<int>{};
            auto offsets = std::vector<int>{};

            for (auto asgn : toCalculate) {
                if (asgn.pickupPairedLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.pickup->loc));
                    int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.transfer.loc));
                    int offset = inputGraph.travelTime(asgn.transfer.loc);
                    sources.push_back(sourceRank);
                    targets.push_back(targetRank);
                    offsets.push_back(offset);
                } else {
                    temp.push_back(asgn);
                }
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            // Calculate the direct distances between pickup and transfer
            std::vector<int> distances;
            if (currentlyCalculating.size() > 0)
                distances = vehChQuery.runManyToMany(sources, targets, offsets);

            std::cout << "Assignments using Pickup Lower Bound Paired: " << currentlyCalculating.size() << std::endl;

            for (int i = 0; i < currentlyCalculating.size(); i++) {
                auto asgn = currentlyCalculating[i];
                asgn.distToTransferPVeh = distances[i];
                asgn.pickupPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                unfinished = asgn.dropoffBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed;

                if (unfinished) {
                    total = calc.calcBaseLowerBound<true, true>(asgn, requestState);
                } else {
                    total = calc.calcBase<true, true>(asgn, requestState);
                }

                if (total.total > requestState.getBestCost())
                    continue;

                if (!unfinished) {
                    std::cout << "VERBESSERUNG!!!!" << std::endl;
                    requestState.tryAssignmentWithTransfer(asgn);
                    continue;
                }

                toCalculate.push_back(asgn);
            }

            currentlyCalculating.clear();
            std::cout << "Noch offen (2/4): " << toCalculate.size() << std::endl;

            // Calculate the dropoffs with postponed bns distance
            for (auto asgn : toCalculate) {
                if (asgn.dropoffBNSLowerBoundUsed) {
                    currentlyCalculating.push_back(asgn);
                    asgn.dropoffBNSLowerBoundUsed = false;
                    searches.addTransferForProcessing(asgn.transfer.loc, asgn.distToTransferDVeh);
                } else {
                    temp.push_back(asgn);
                }
                
                unfinished = asgn.dropoffBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed;

                if (unfinished)
                    temp.push_back(asgn);
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            if (currentlyCalculating.size() > 0)
                searches.computeExactTransferDistancesVia(dVeh);

            for (auto asgn : currentlyCalculating) {
                assert(searches.knowsDistanceTransfer(dVeh.vehicleId, asgn.transfer.loc));
                const int distance = searches.getDistanceTransfer(dVeh.vehicleId, asgn.transfer.loc);
                asgn.distToTransferDVeh = distance;
                asgn.dropoffBNSLowerBoundUsed = false;
                unfinished = asgn.dropoffPairedLowerBoundUsed; 

                if (unfinished) {
                    total = calc.calcBaseLowerBound<true, false>(asgn, requestState);
                } else {
                    total = calc.calcBase<true, false>(asgn, requestState);
                }

                if (total.total > requestState.getBestCost())
                    continue;

                if (!unfinished) {
                    std::cout << "VERBESSERUNG!!!!" << std::endl;
                    requestState.tryAssignmentWithTransfer(asgn);
                    continue;
                }

                toCalculate.push_back(asgn);
            }

            std::cout << "Noch offen (3/4): " << toCalculate.size() << std::endl;

            // Calculate the exact paired distance between transfer and dropoff
            currentlyCalculating.clear();

            sources = std::vector<int>{};
            targets = std::vector<int>{};
            offsets = std::vector<int>{};

            for (auto asgn : toCalculate) {
                assert(asgn.dropoffPairedLowerBoundUsed);
                
                currentlyCalculating.push_back(asgn);
                int sourceRank = vehCh.rank(inputGraph.edgeHead(asgn.transfer.loc));
                int targetRank = vehCh.rank(inputGraph.edgeTail(asgn.dropoff->loc));
                int offset = inputGraph.travelTime(asgn.dropoff->loc);
                sources.push_back(sourceRank);
                targets.push_back(targetRank);
                offsets.push_back(offset);
            }

            toCalculate.clear();
            toCalculate = temp;
            temp.clear();

            distances.clear();
            if (currentlyCalculating.size() > 0)
                distances = vehChQuery.runManyToMany(sources, targets, offsets);

            std::cout << "Assignments using Dropoff Lower Bound Paired: " << currentlyCalculating.size() << std::endl;

            for (int i = 0; i < currentlyCalculating.size(); i++) {
                auto asgn = currentlyCalculating[i];
                asgn.distToDropoff = distances[i];
                asgn.dropoffPairedLowerBoundUsed = false;

                // Try the assignments with the calculated distances
                assert(!asgn.pickupBNSLowerBoundUsed && !asgn.pickupPairedLowerBoundUsed && !asgn.dropoffBNSLowerBoundUsed && !asgn.dropoffPairedLowerBoundUsed);
                
                total = calc.calcBase<true, false>(asgn, requestState);

                if (total.total > requestState.getBestCost())
                    continue;

                
                std::cout << "VERBESSERUNG!!!!" << std::endl;
                requestState.tryAssignmentWithTransfer(asgn);
                continue;
            }

            std::cout << "- - - - - - - - - - - - - - - - - - - - /" << std::endl;
        }

        std::vector<Pickup> getOrdPickupsBefore(int stopIdx) {
            auto result = std::vector<Pickup>{};
            
            for (auto &pickup : ordPickups) {
                if (pickup.pdIdx <= stopIdx)
                    result.push_back(pickup);
            }
            
            return result;
        }

        std::vector<Dropoff> getOrdDropoffsAfter(int stopIdx, int numStops) { // TODO Sind hier die numStops notwendig?
            auto result = std::vector<Dropoff>{};
            
            for (auto &dropoff : ordDropoffs) {
                if (dropoff.pdIdx >= stopIdx && dropoff.pdIdx < numStops)
                    result.push_back(dropoff);
            }
            
            return result;
        }

        int countTransferPoints() const {
            int res = 0;

            for (const auto &stopPair : transferPoints) {
                res += stopPair.second.size();
            }

            return res;
        }

        void printRequestCost(RequestCost cost) {
            std::cout << "TOTAL: " << cost.total << ", Walking: " << cost.walkingCost << ", Wait: " << cost.waitTimeViolationCost << ", Trip: " << cost.tripCost << ", Change: " << cost.changeInTripCostsOfOthers << ", Veh: " << cost.vehCost << std::endl;
        }

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        CostCalculator &calc;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
        TransferPointFinder<StrategyT> &tpFinder;
        // Stores all transferpoints between two given stop pairs
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;
        std::vector<Pickup> ordPickups;
        std::vector<Dropoff> ordDropoffs;
        std::vector<Pickup> bnsPickups;
        std::vector<Dropoff> bnsDropoffs;
        std::vector<Pickup> alsPickups;
        std::vector<Dropoff> alsDropoffs;

        std::vector<AssignmentWithTransfer> promisingPartials;
        std::vector<AssignmentWithTransfer> unfinishedAssignments;

        const VehicleLocator<InputGraphT, VehCHEnvT> &locator;
        CurVehLocToPickupSearchesT &searches;
        std::vector<AssignmentWithTransfer> assignmentsWithUnknownPickupDistance;
        std::vector<AssignmentWithTransfer> assignmentsWithUnknownTransferDistance;

        std::map<std::tuple<int, int>, int> calculatedDirectDistances;

        int pairedLowerBoundPT;
        int pairedLowerBoundTD;
        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet> dijSearchLowerBound;

        int64_t numAssignmentsTried = 0;
        int64_t numPartialsTried = 0;
        int64_t numAssignmentsWithUnkownPairedDistanceTried = 0;
    };


}