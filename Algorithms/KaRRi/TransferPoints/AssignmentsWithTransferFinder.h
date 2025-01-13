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

// TODO #1 Dropoff ALS - searches from last stop of dVeh to dropoff
// TODO #2 Transfer ALS - searches from last stop of pVeh to stop location of the dVeh to use as transfer point
// TODO #3 Pickup ALS - search from last stop of pVeh to pickup, then use transfer als

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

                // Get the possible pickup and dropoff locations for the vehicle // ! These location are, once again, edges
                ordPickups = pVehs.getOrdPDsForVehicle(pVeh.vehicleId);
                ordDropoffs = dVehs.getOrdPDsForVehicle(dVeh.vehicleId);
                bnsPickups = pVehs.getBnsPDsForVehicle(pVeh.vehicleId);
                bnsDropoffs = dVehs.getBnsPDsForVehicle(dVeh.vehicleId);

                // Calculate lower bounds for paired distances for pickup-transfer and transfer-dropoff
                pairedLowerBoundPT = calculateLowerBoundPairedPT();
                pairedLowerBoundTD = calculateLowerBoundPairedTD();

                // std::cout << "LB PT : " << pairedLowerBoundPT << std::endl;
                // std::cout << "LB TD : " << pairedLowerBoundTD << std::endl;


                // * ORDINARY TRANSFER (also includes transfer before next stop)
                findAssignmentsWithOrdinaryTransfer(pVeh, dVeh);

                // * TRANSFER AFTER LAST STOP
                // First approach for transfer als is, that the pickup vehicle drives to a stop position of the dropoff vehicle and uses this position as the transfer point
                // findAssignmentsWithTransferAtStop(pVeh, dVeh); // TODO Das muss logischerweise dann außerhalb der festgelegten Indizes sein

                promisingPartials.clear();
            
                // TODO findAssignmentsWithTransferAfterLastStop(); -> Ziel: Nach Weihnachten sollte hier die ersten Ansätze laufen
                // TODO Eigentlich sollten damit dann signifikant mehr AssignmentsWithTransfer gefunden werden als bisher (Ziel: 10-20%)
                // TODO Viele Routen der Fahrzeuge sind nämlich so kurz, dass ein Transfer ALS stattfinden muss 
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
                const auto relOrdDropoffs = getOrdDropoffsAfter(asgn.transferIdxDVeh);

                for (const auto &dropoff : relOrdDropoffs) {
                    auto newAsgn = AssignmentWithTransfer(asgn); 

                    newAsgn.dropoffIdx = dropoff.pdIdx;
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

            for (int distance : distances) {
                std::cout << "distance : " << distance << std::endl;
            }

            for (const auto &asgn : promisingPartials) {
                for (int i = 0; i < requestState.dropoffs.size(); i++) {
                    auto newAsgn = AssignmentWithTransfer(asgn);

                    newAsgn.dropoffIdx = numStops - 1;
                    newAsgn.dropoff = &requestState.dropoffs[i];
                    newAsgn.distToDropoff = distances[i];
                    newAsgn.distFromDropoff = 0;
                
                    std::cout << "TRY DROPOFF ALS" << std::endl;
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

            /*

            if (assignmentsWithUnknownPickupDistance.size() > 0) {
                // Finish the calculation of the unknown pickup distances
                for (const auto &unfinishedAsgn : assignmentsWithUnknownPickupDistance) {
                    searches.addPickupForProcessing(unfinishedAsgn.pickup->id, unfinishedAsgn.distToPickup);
                }

                searches.computeExactDistancesVia(pVeh);

                for (auto &asgn : assignmentsWithUnknownPickupDistance) {
                    if (promisingPartialAssignmentsWithUnknownPairedDistance.count({asgn.pickup->id, asgn.transfer.loc}))
                        continue;
                    
                    tryPostponedPartialAssignment(asgn);
                }

                assignmentsWithUnknownPickupDistance.clear();
                
                const auto postponedPickupTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.postponedPartialsTime += postponedPickupTime;
            } */

            if (promisingPartials.size() == 0)
                return;

            tryOrdinaryDropoffs();
            tryDropoffsBNS();
            tryDropoffALS(dVeh);

            if (unfinishedAssignments.size() > 0)
                finishAssignments();

            /*
            // Clear the assignments with unknown paired distance, as they have been treated in the tryDropoff part
            promisingPartialAssignmentsWithUnknownPairedDistance.clear();

            if (assignmentsWithUnknownTransferDistance.size() > 0) {

                timer.restart();

                // Add the transfer for processing
                for (auto &asgn : assignmentsWithUnknownTransferDistance) {
                    searches.addTransferForProcessing(asgn.transfer.loc, asgn.distToTransferDVeh);
                }

                searches.computeExactTransferDistancesVia(dVeh);

                for (auto &asgn : assignmentsWithUnknownTransferDistance) {
                    tryPostponedAssignment(asgn);
                }

                assignmentsWithUnknownTransferDistance.clear();

                const auto postponedTransferTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.postponedAssignmentsTime += postponedTransferTime;
            }
            */
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

            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) are set
            const RequestCost pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);

            if (pickupVehCost.total > requestState.getBestCost())
                return;

            promisingPartials.push_back(asgn);
        }

        /*
        void tryPostponedPartialAssignment(AssignmentWithTransfer &asgn) {
            assert(asgn.pickupIdx == 0);
        
            // Calculate cost of assignment
            const RequestCost pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);

            if (pickupVehCost.total < requestState.getBestCost()) {
                promisingPartialAsssignments.push_back(asgn);
            }
        }

        void tryAssignmentWithUnkownPairedDistance(AssignmentWithTransfer &asgn) {
            // Calculation only gets postponed for paired assignments
            assert(asgn.pickupIdx == asgn.transferIdxPVeh);
            assert(searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id));
            
            numAssignmentsWithUnkownPairedDistanceTried++;
            
            // Handle the pickup bns case
            if (asgn.pickupIdx == 0) {
                // Set the distance that by now has been calculated
                asgn.distToTransferPVeh = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
            }

            const int pLoc = asgn.pickup->loc;
            const int tLoc = asgn.transfer.loc;
            const int dLoc = asgn.dropoff->loc;
            
            if (asgn.dropoffIdx == asgn.transferIdxDVeh) {
                // Try lower bound for paired assignment
                asgn.distFromTransferDVeh = 0;
                asgn.distToDropoff = 0; // TODO Use better lower bound

                const bool directDistanceKnown = calculatedDirectDistances.count({tLoc, dLoc});

                if (directDistanceKnown) {
                    asgn.distToDropoff = calculatedDirectDistances[{tLoc, dLoc}];
                }
            }
            
            // Try lower bound for the assignment
            const auto lowerBound = calc.calcBase<true>(asgn, requestState);

            if (lowerBound.total > requestState.getBestCost())
                return;
            
            // If the lower bound including the dropoff vehicle cost are still good enough, calculate the direct distance between the pickup and the transfer
            // Calculate the direct pickup-transfer distance (for the pVeh)
            if (!calculatedDirectDistances.count({pLoc, tLoc})) {
                const auto sourcePVeh = vehCh.rank(inputGraph.edgeHead(pLoc));
                const auto targetPVeh = vehCh.rank(inputGraph.edgeTail(tLoc));
                vehChQuery.run(sourcePVeh, targetPVeh);

                asgn.distToTransferPVeh = vehChQuery.getDistance();
            } else {
                // It is possible that by the execution of the if-statement, the direct distance is already calculated
                asgn.distToTransferPVeh = calculatedDirectDistances[{pLoc, tLoc}];
            }

            const auto partialCostPVeh = calc.calcPartialCostForPVeh<true>(asgn, requestState);
            if (partialCostPVeh.total > requestState.getBestCost())
                return;

            // Now we know, that the partial costs are okay, we can continue normally for the rest of the assignment
            tryAssignment(asgn);
        }
        */

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

            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) is set
            const RequestCost assignmentCost = calc.calcBase<true>(asgn, requestState);

            if (assignmentCost.total > requestState.getBestCost())
                return;
            
            if (asgn.pickupBNSLowerBoundUsed || asgn.pickupPairedLowerBoundUsed || asgn.dropoffBNSLowerBoundUsed || asgn.dropoffPairedLowerBoundUsed) {
                unfinishedAssignments.push_back(asgn);
                return;
            }

            requestState.tryAssignmentWithTransfer(asgn);
        }

        void finishAssignments() {
            // Method to finish the assignments that have lower bounds used
            std::vector<AssignmentWithTransfer> toCalculate;

            // Start with the pickups with postponed bns distance
            std::cout << "FINISH ASSIGNMENTS..." << std::endl;

            // Calculate the exact paired distance between pickup and transfer


            // Calculate the dropoffs with postponed bns distance
            

            // Calculate the exact paired distance between transfer and dropoff
            



        
        
        
        }

        void tryPostponedAssignment(AssignmentWithTransfer &asgn) {
            assert(asgn.transferIdxDVeh == 0);
                    
            assert(searches.knowsDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc));
            asgn.distToTransferDVeh = searches.getDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc);

            // Calculate cost of assignment
            const RequestCost assignmentCost = calc.calcBase<true>(asgn, requestState);

            if (assignmentCost.total < INFTY) {
                requestState.tryAssignmentWithTransfer(asgn);
            }
        }

        std::vector<Pickup> getOrdPickupsBefore(int stopIdx) {
            auto result = std::vector<Pickup>{};
            
            for (auto &pickup : ordPickups) {
                if (pickup.pdIdx <= stopIdx)
                    result.push_back(pickup);
            }
            
            return result;
        }

        std::vector<Dropoff> getOrdDropoffsAfter(int stopIdx) {
            auto result = std::vector<Dropoff>{};
            
            for (auto &dropoff : ordDropoffs) {
                if (dropoff.pdIdx >= stopIdx)
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