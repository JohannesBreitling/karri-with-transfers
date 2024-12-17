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


#pragma once

#include "Algorithms/KaRRi/TransferPoints/TransferPointFinder.h"
#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"

namespace karri {

    template <typename StrategyT, typename InputGraphT, typename VehCHEnvT, typename CurVehLocToPickupSearchesT>
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
            std::vector<TransferPoint> &possibleTransferPoints,
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
                              possibleTransferPoints(possibleTransferPoints),
                              transferPoints(std::map<std::tuple<int, int>, std::vector<TransferPoint>>{}),
                              locator(locator),
                              searches(searches),
                              assignmentsWithUnknownPickupDistance(std::vector<AssignmentWithTransfer>{}),
                              assignmentsWithUnknownTransferDistance(std::vector<AssignmentWithTransfer>{}),
                              calculatedDirectDistances(std::map<std::tuple<int, int>, int>{}) {}

        void init() {
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};
            possibleTransferPoints = std::vector<TransferPoint>{};
        }

        // Method to find the best assignment with exactly one transfer, using the best found cost without transfer to prune solutions
        void findBestAssignment() {

            Timer totalTimer;

            auto &stats = requestState.stats().transferStats;
            numPartialsTried = 0;
            numAssignmentsTried = 0;
            int64_t numPickupDropoffPairs = 0;
            int64_t numStopPairs = 0;

            // Pair up the vehicles
            pairUpVehicles();
        
            // Loop over all possible pairs of pickup and dropoff vehicles
            for (auto &pickupDropoffPair : pickupDropoffPairs) {

                numPickupDropoffPairs++;

                promisingPartialAsssignments = std::vector<AssignmentWithTransfer>{};

                auto &pVeh = std::get<0>(pickupDropoffPair);
                auto &dVeh = std::get<1>(pickupDropoffPair);

                Timer timer;

                // Loop over all possible stop pairs and calculate the possible transfer points between them
                for (int pIndex = 0; pIndex < routeState.numStopsOf(pVeh.vehicleId) - 1; pIndex++) {
                    for (int dIndex = 0; dIndex < routeState.numStopsOf(dVeh.vehicleId) - 1; dIndex++) {
                        
                        numStopPairs++;
                        
                        // Calculate the transfer points between the two stops
                        calculateTransferPointsBetweenStopPair(pVeh, dVeh, pIndex, dIndex);
                    }
                }

                stats.numTransferPoints += possibleTransferPoints.size();

                const auto transferPointCalculationTime = timer.elapsed<std::chrono::nanoseconds>();
                stats.transferPointCalculationTime += transferPointCalculationTime;
            
                if (possibleTransferPoints.size() == 0)
                    continue;

                // Get the possible pickup and dropoff locations for the vehicle
                ordPickups = pVehs.getOrdPDsForVehicle(pVeh.vehicleId);
                ordDropoffs = dVehs.getOrdPDsForVehicle(dVeh.vehicleId);
                bnsPickups = pVehs.getBnsPDsForVehicle(pVeh.vehicleId);
                bnsDropoffs = dVehs.getBnsPDsForVehicle(dVeh.vehicleId);


                // * ORDINARY TRANSFER (also includes transfer before next stop)
                findAssignmentsWithOrdinaryTransfer(pVeh, dVeh);
            
                // TODO findAssignmentsWithTransferAfterLastStop();
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

            const auto vehiclePairUpTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().transferStats.vehiclePairupTime += vehiclePairUpTime;

            assert(pickupDropoffPairs.size() <= pVehs.getVehicles()->size() * dVehs.getVehicles()->size());
        }

        void calculateTransferPointsBetweenStopPair(
            Vehicle &pVeh, Vehicle &dVeh,
            int stopIdxPVeh, int stopIdxDVeh) {

            // Get the stop locations of the two vehicles               
            const auto &stopLocationsPVeh = routeState.stopLocationsFor(pVeh.vehicleId);
            const auto &stopLocationsDVeh = routeState.stopLocationsFor(dVeh.vehicleId);
            
            // Get the start locations for the searches, Caution: These locations are edges
            assert(stopIdxPVeh + 1 < routeState.numStopsOf(pVeh.vehicleId));
            assert(stopIdxDVeh + 1 < routeState.numStopsOf(dVeh.vehicleId));

            int pickupVehicleStop = stopLocationsPVeh[stopIdxPVeh];
            int pickupVehicleNextStop = stopLocationsPVeh[stopIdxPVeh + 1];
            int dropoffVehicleStop = stopLocationsDVeh[stopIdxDVeh];
            int dropoffVehicleNextStop = stopLocationsDVeh[stopIdxDVeh + 1];

            // Get the stop ids of the stops
            int stopIdPickup = routeState.stopIdsFor(pVeh.vehicleId)[stopIdxPVeh];
            int stopIdDropoff = routeState.stopIdsFor(dVeh.vehicleId)[stopIdxDVeh];

            // Get the hard constraints for stopping criterions for the dijkstra searches
            int maxDetourPickup = routeState.leewayOfLegStartingAt(stopIdPickup);
            int maxDetourDropoff = routeState.leewayOfLegStartingAt(stopIdDropoff);

            if (maxDetourPickup < 0 || maxDetourDropoff < 0)
                return;
            
            // Use the transfer point finder to find the possible transfer points
            tpFinder.findTransferPoints(maxDetourPickup, maxDetourDropoff, pickupVehicleStop, pickupVehicleNextStop, dropoffVehicleStop, dropoffVehicleNextStop);

            // Build the possible transfer points
            std::vector<TransferPoint> tpsForStopPair = std::vector<TransferPoint>{};
            for (auto &tp : possibleTransferPoints) {
                tp.pVeh = &pVeh;
                tp.dVeh = &dVeh;
                tp.dropoffAtTransferStopIdx = stopIdxPVeh;
                tp.pickupFromTransferStopIdx = stopIdxDVeh;
            
                tpsForStopPair.push_back(tp);
            }

            transferPoints[{stopIdxPVeh, stopIdxDVeh}] = tpsForStopPair;
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

        void tryOrdinaryDropoffs() {
            for (auto &asgn : promisingPartialAsssignments) {
                const auto relOrdDropoffs = getOrdDropoffsAfter(asgn.transferIdxDVeh);

                for (const auto &dropoff : relOrdDropoffs) {
                    asgn.dropoffIdx = dropoff.pdIdx;
                    asgn.dropoff = &(requestState.dropoffs[dropoff.pdLocId]);
                    asgn.distToDropoff = dropoff.detourToPD;
                    asgn.distFromDropoff = dropoff.detourFromPD;
                
                    tryAssignment(asgn);
                }
            }
        }

        void tryDropoffsBNS() {
            for (auto &asgn : promisingPartialAsssignments) {
                if (asgn.transferIdxDVeh != 0)
                    continue;

                for (const auto &dropoff : bnsDropoffs) {
                    assert(dropoff.pdIdx == 0);
                    assert(asgn.transferIdxDVeh == 0);

                    asgn.dropoffIdx = 0;
                    asgn.dropoff = &(requestState.dropoffs[dropoff.pdLocId]);
                    asgn.distToDropoff = dropoff.detourToPD;
                    asgn.distFromDropoff = dropoff.detourFromPD;
                
                    tryAssignment(asgn);
                }
            }
        }

        void findAssignmentsWithOrdinaryTransfer(Vehicle &pVeh, Vehicle &dVeh) {
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


                }
            }

            Timer timer;

            if (assignmentsWithUnknownPickupDistance.size() > 0) {
                // Finish the calculation of the unknown pickup distances
                for (const auto &unfinishedAsgn : assignmentsWithUnknownPickupDistance) {
                    searches.addPickupForProcessing(unfinishedAsgn.pickup->id, unfinishedAsgn.distToPickup);
                }

                searches.computeExactDistancesVia(pVeh);

                for (auto &asgn : assignmentsWithUnknownPickupDistance) {
                    tryPostponedPartialAssignment(asgn);
                }

                assignmentsWithUnknownPickupDistance.clear();
                
                const auto postponedPickupTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.postponedPartialsTime += postponedPickupTime;
            }

            if (promisingPartialAsssignments.size() == 0)
                return;

            tryOrdinaryDropoffs();
            tryDropoffsBNS();

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
        }


        void tryPartialAssignment(AssignmentWithTransfer &asgn) {

            numPartialsTried++;

            if (asgn.pickupIdx == asgn.transferIdxPVeh) {

                Timer timer;

                // Paired Assignment (pVeh)

                // Try lower bound for paired assignment
                asgn.distFromPickup = 0;
                asgn.distToTransferPVeh = 0;
                const auto lowerBoundPaired = calc.calcPartialCostForPVeh<true>(asgn, requestState);

                if (lowerBoundPaired.total > requestState.getBestCost())
                    return;

                const int pLoc = asgn.pickup->loc;
                const int tLoc = asgn.transfer.loc;

                int distance;
                
                if (!calculatedDirectDistances.count({pLoc, tLoc})){
                    // Paired assignment, calculate the direct distance between origin and transfer point
                    const auto source = vehCh.rank(inputGraph.edgeHead(pLoc));
                    const auto target = vehCh.rank(inputGraph.edgeTail(tLoc));

                    vehChQuery.run(source, target);
                    
                    distance = vehChQuery.getDistance() + inputGraph.travelTime(asgn.transfer.loc);
                    calculatedDirectDistances[{pLoc, tLoc}] = distance;
                } else {
                    distance = calculatedDirectDistances[{pLoc, tLoc}];
                }

                asgn.distFromPickup = 0;
                asgn.distToTransferPVeh = distance;

                const auto pickupPairedTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.pickupPairedTime += pickupPairedTime;
            }

            if (asgn.pickupIdx == 0) {

                Timer timer;

                // Assignment with pickup bns, try lower bound
                const RequestCost lowerBoundBNS = calc.calcPartialCostForPVeh<true>(asgn, requestState);

                if (lowerBoundBNS.total > requestState.getBestCost())
                    return;

                // Get the exact distances from the last stop to the pickup via the current location
                // Check if exact distances are known
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    // Postpone the calculation of the missing exact distance
                    assignmentsWithUnknownPickupDistance.push_back(asgn);
                    return;
                }

                const auto pickupBnsTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.pickupBnsTime += pickupBnsTime;               
            }

            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) is set
            const RequestCost pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);

            if (pickupVehCost.total < requestState.getBestCost()) {
                promisingPartialAsssignments.push_back(asgn);
            }
        }

        void tryPostponedPartialAssignment(AssignmentWithTransfer &asgn) {
            assert(asgn.pickupIdx == 0);
            assert(searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id));

            // Calculate cost of assignment
            const RequestCost pickupVehCost = calc.calcPartialCostForPVeh<true>(asgn, requestState);

            if (pickupVehCost.total < requestState.getBestCost()) {
                promisingPartialAsssignments.push_back(asgn);
            }
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {

            numAssignmentsTried++;

            if (asgn.dropoffIdx == asgn.transferIdxDVeh) {

                Timer timer;

                // Paired Assignment (dVeh)

                // Try lower bound for paired assignment
                asgn.distFromTransferDVeh = 0;
                asgn.distToPickup = 0;
                const auto lowerBoundPaired = calc.calcBase<true>(asgn, requestState);

                if (lowerBoundPaired.total > requestState.getBestCost())
                    return;

                const int tLoc = asgn.transfer.loc;
                const int dLoc = asgn.dropoff->loc;

                int distance;

                if (!calculatedDirectDistances.count({tLoc, dLoc})){
                    // Paired assignment, calculate the direct distance between origin and transfer point
                    const auto source = vehCh.rank(inputGraph.edgeHead(tLoc));
                    const auto target = vehCh.rank(inputGraph.edgeTail(dLoc));

                    vehChQuery.run(source, target);
                    
                    distance = vehChQuery.getDistance() + inputGraph.travelTime(asgn.dropoff->loc);
                    calculatedDirectDistances[{tLoc, dLoc}] = distance;
                } else {
                    distance = calculatedDirectDistances[{tLoc, dLoc}];
                }

                asgn.distFromTransferDVeh = 0;
                asgn.distToDropoff = distance;

                const auto transferPairedTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.transferPairedTime += transferPairedTime;
            }

            if (asgn.transferIdxDVeh == 0) {

                Timer timer;

                // Transfer BNS
                // Try lower bound
                const RequestCost lowerBoundBNS = calc.calcBase<true>(asgn, requestState);

                if (lowerBoundBNS.total > requestState.getBestCost())
                    return;

                if (!searches.knowsDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc)) {
                    // Postpone the computation of the distance of the dropoff
                    assignmentsWithUnknownTransferDistance.push_back(asgn);
                    return;
                }
                
                asgn.distToTransferDVeh = searches.getDistanceTransfer(asgn.dVeh->vehicleId, asgn.transfer.loc);

                const auto transferBnsTime = timer.elapsed<std::chrono::nanoseconds>();
                requestState.stats().transferStats.transferBnsTime += transferBnsTime;
            }

            // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) is set
            const RequestCost assignmentCost = calc.calcBase<true>(asgn, requestState);

            if (assignmentCost.total < INFTY) {
                requestState.tryAssignmentWithTransfer(asgn);
            }
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
        std::vector<TransferPoint> &possibleTransferPoints;

        // Stores all transferpoints between two given stop pairs
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> transferPoints;
        std::vector<Pickup> ordPickups;
        std::vector<Dropoff> ordDropoffs;
        std::vector<Pickup> bnsPickups;
        std::vector<Dropoff> bnsDropoffs;

        std::vector<AssignmentWithTransfer> promisingPartialAsssignments;

        const VehicleLocator<InputGraphT, VehCHEnvT> &locator;
        CurVehLocToPickupSearchesT &searches;
        std::vector<AssignmentWithTransfer> assignmentsWithUnknownPickupDistance;
        std::vector<AssignmentWithTransfer> assignmentsWithUnknownTransferDistance;

        // std::vector<AssignmentWithTransfer> promisingPartialAssignmentsWithUnknownPickupDistance;

        std::map<std::tuple<int, int>, int> calculatedDirectDistances;

        int64_t numAssignmentsTried = 0;
        int64_t numPartialsTried = 0;
    };


}