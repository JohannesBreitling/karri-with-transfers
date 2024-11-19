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

namespace karri {

    template <typename StrategyT, typename InputGraphT, typename VehCHEnvT>
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
            std::vector<TransferPoint> &possibleTransferPoints)
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
                              transferPoints(std::map<std::tuple<int, int>, std::vector<TransferPoint>>{}) {}

        void init() {
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};
            possibleTransferPoints = std::vector<TransferPoint>{};
        }

        // Method to find the best assignment with exactly one transfer, using the best found cost without transfer to prune solutions
        void findBestAssignment(int upperBound) {
            // Pair up the vehicles
            pairUpVehicles();

            // Best currently found assignment
            // AssignmentWithTransfer bestAssignment;

            // Loop over all possible pairs of pickup and dropoff vehicles
            for (auto &pickupDropoffPair : pickupDropoffPairs) {

                auto &pVeh = std::get<0>(pickupDropoffPair);
                auto &dVeh = std::get<1>(pickupDropoffPair);

                // Get the stop locations of the two vehicles               
                const auto &stopLocationsPickupVehicle = routeState.stopLocationsFor(pVeh.vehicleId);
                const auto &stopLocationsDropoffVehicle = routeState.stopLocationsFor(dVeh.vehicleId);

                // Loop over all possible stop pairs and calculate the possible transfer points between them
                for (int pIndex = 0; pIndex < routeState.numStopsOf(pVeh.vehicleId) - 1; pIndex++) {
                    for (int dIndex = 0; dIndex < routeState.numStopsOf(dVeh.vehicleId) - 1; dIndex++) {
                        // Calculate the transfer points between the two stops
                        calculateTransferPointsBetweenStopPair(pVeh, dVeh, stopLocationsPickupVehicle, stopLocationsDropoffVehicle, pIndex, dIndex);
                    }
                }

                
                // TODO
                (void) upperBound;
                // (void) bestAssignment;                

                // * Start building the assignments
                // To build the assignments, we differentiate between the following cases:
                // TODO - Ordinary-Paired: The pickup is ordinary (between two stops in the current route), the transfer takes place between pickup and the next stop
                // TODO - Ordinary-Ordinary: The pickup is ordinary, the transfer is also ordinary but between two different stops
                // TODO - Ordinary-AfterLastStop:
                // TODO - BeforeNextStop-Paired:
                // TODO - BeforeNextStop-Ordinary:
                // TODO - BeforeNextStop-AfterLastStop:
                // TODO - AfterLastStop-AfterLastStop:

                // Ordinary pickups & dropoffs
                ordPickups = pVehs.getOrdPDsForVehicle(pVeh.vehicleId);
                // ordDropoffs = dVehs.getOrdPDsForVehicle(dVeh.vehicleId); 

                // BNS pickups & dropoffs
                bnsPickups = pVehs.getBnsPDsForVehicle(pVeh.vehicleId);
                // bnsDropoffs = dVehs.getBnsPDsForVehicle(dVeh.vehicleId);


                // * ORDINARY TRANSFERS
                // Loop over all the stop pairs to build the assignments with a transfer between two stops
                for (int trIdxP = 0; trIdxP < routeState.numStopsOf(pVeh.vehicleId) - 1; trIdxP++) {
                    
                    // Get the possible pickups
                    const auto relOrdPickups = getOrdPickupsBefore(trIdxP);
                    const auto relBnsPickups = getBnsPickupsBefore(trIdxP);
                    
                    for (int trIdxD = 0; trIdxD < routeState.numStopsOf(dVeh.vehicleId) - 1; trIdxD++) {
                        // Get the relevant transfer points
                        const auto relTransferPoints = transferPoints[{trIdxP, trIdxD}];

                        for (const auto &pickup : relOrdPickups) {
                            for (const auto &tp : relTransferPoints) {
                                AssignmentWithTransfer asgn;
                                asgn.pVeh = &pVeh;
                                asgn.dVeh = &dVeh;
                                asgn.pickup = &requestState.pickups[pickup.pdLocId];
                                asgn.distToPickup = pickup.detourToPD;
                                asgn.distFromPickup = pickup.detourFromPD;
                                asgn.distToTransferPVeh = tp.distancePVehToTransfer;
                                asgn.distFromTransferPVeh = tp.distancePVehFromTransfer;

                                asgn.pickupIdx = pickup.pdIdx;
                                asgn.transferIdxPVeh = trIdxP;
                                asgn.transferIdxDVeh = trIdxD;

                                if (pickup.pdIdx == trIdxP) {
                                    // Ordinary Paired Assignment
                                    // Calculate the direct distance between origin and transfer point
                                    const auto source = vehCh.rank(inputGraph.edgeHead(requestState.originalRequest.origin));
                                    const auto target = vehCh.rank(tp.loc);

                                    vehChQuery.run(source, target);

                                    const int distance = vehChQuery.getDistance();

                                    asgn.distFromPickup = 0;
                                    asgn.distToTransferPVeh = distance;
                                }

                                // Check the cost of the partial assignment with transfer where pickup vehicle, dropoff vehicle, pickup and transfer point (therefore also both transfer stop indices) is set
                                const int pickupVehCost = calc.calcPartialCostForOrdPickup<true>(asgn, requestState);

                                if (pickupVehCost > upperBound) {
                                    continue;
                                }

                                // TODO Transfer with vehicle is promising
                                std::cout << "Upper Bound : " << upperBound << std::endl;
                                std::cout << "Pickup Vehicle Cost : " << pickupVehCost << std::endl;
                                std::cout << "Difference : " << upperBound - pickupVehCost << std::endl;
                                


                            }
                        }
                    }
                }

                /*
            
                // Try assignment with all of the pickups & dropoffs
                for (const auto &pickup : ordinaryPickups) {
                    // Try assignment with all of the dropoffs    
                    for (const auto &dropoff : relevantOrdinaryDropoffs) {
                        // Get all transferpoints between the given indices

                        if (dropoff.pdIdx < 1 || pickup.pdIdx >= routeState.numStopsOf(pVeh.vehicleId) - 1) {
                            continue;
                        }


                        std::cout << "Searching between : " << pickup.pdIdx << ", " <<  dropoff.pdIdx << std::endl;         
                        std::vector<TransferPoint> tpsBetweenPD = getTransferPointsBetweenStops(pVeh, pickup.pdIdx + 1, dropoff.pdIdx - 1);

                        
                        for (auto &tp : tpsBetweenPD) {
                            const auto asgn = AssignmentWithTransfer(pVeh, dVeh, pickup, dropoff, tp);
                            (void) asgn;

                            // Calcualte cost of the given assignment with transfer
                            // int costOfAssignment = calc.calcBaseWithTransfer(upperBound, asgn, requestState);
                            //(void) costOfAssignment;
                        }

                        (void) pickup;
                        (void) dropoff;
                    }


                }
                */
            }
        }


    private:

    // Method to build up pairs of pickup and dropoff vehicles
    void pairUpVehicles() {
        for (const auto &pVeh : *pVehs.getVehicles()) {
            for (const auto &dVeh : *dVehs.getVehicles()) {
                // If the pickup is the same as the dropoff vehicle, we already tested the assignment without a transfer
                if (pVeh->vehicleId == dVeh->vehicleId)
                    continue;

                pickupDropoffPairs.push_back(std::make_tuple(*pVeh, *dVeh));
            }
        }
    }

    void calculateTransferPointsBetweenStopPair(
        Vehicle &pVeh, Vehicle &dVeh,
        const ConstantVectorRange<int> &stopLocationsPVeh, const ConstantVectorRange<int> &stopLocationsDVeh,
        int stopIdxPVeh, int stopIdxDVeh
    ) {

        // Get the start locations for the searches, Caution: These locations are edges
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

        // TODO Fall mit Transfer at stop eventuell extra behandeln....
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


    // TODO  Diese Methoden schneller machen....
    std::vector<Pickup> getOrdPickupsBefore(int stopIdx) {
        auto result = std::vector<Pickup>{};
        
        for (auto &pickup : ordPickups) {
            if (pickup.pdIdx <= stopIdx)
                result.push_back(pickup);
        }
        
        return result;
    }

    std::vector<Pickup> getBnsPickupsBefore(int stopIdx) {
        auto result = std::vector<Pickup>{};
        
        for (auto &pickup : bnsPickups) {
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
    
    };


}