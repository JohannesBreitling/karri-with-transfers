/// ******************************************************************************
/// MIT License
///
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

namespace karri {
    
    template<
        typename StrategyT
    >
    class TransferPointFinder {

    public:
        TransferPointFinder(
            StrategyT &strategy,
            const Fleet &fleet,
            const RouteState &routeState,
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs
        ) : strategy(strategy),
            fleet(fleet), routeState(routeState),
            pVehs(pVehs), dVehs(dVehs) {}

        void init() {
            pVehs.init();
            dVehs.init();
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};         
        }

        void findTransferPoints() {
            pairUpVehicles();
            
            // Try for all the stop pairs
            for (const auto &pickupDropoffPair : pickupDropoffPairs) {
                
                const auto &pVeh = std::get<0>(pickupDropoffPair);
                const auto &dVeh = std::get<1>(pickupDropoffPair);

                const auto &stopLocationsPickupVehicle = routeState.stopLocationsFor(pVeh.vehicleId);
                const auto &stopLocationsDropoffVehicle = routeState.stopLocationsFor(dVeh.vehicleId);
                
                for (int pIndex = 0; pIndex < routeState.numStopsOf(pVeh.vehicleId) - 1; pIndex++) {
                    for (int dIndex = 0; dIndex < routeState.numStopsOf(dVeh.vehicleId) - 1; dIndex++) {
                        int pickupVehicleStop = stopLocationsPickupVehicle[pIndex];
                        int pickupVehicleNextStop = stopLocationsPickupVehicle[pIndex + 1];
                        int dropoffVehicleStop = stopLocationsPickupVehicle[dIndex];
                        int dropoffVehicleNextStop = stopLocationsPickupVehicle[dIndex + 1];

                        // Run 4 dijkstra searches to determine the intersection of the search spaces
                        // TODO
                    }
                }




            }
        }

    private:
        void pairUpVehicles() {
            for (const auto &pVeh : *pVehs.getVehicles()) {
                for (const auto &dVeh : *dVehs.getVehicles()) {
                    // If the pickup is the same as the dropoff vehicle, we already tested the assignment without a transfer
                    if (pVeh.vehicleId == dVeh.vehicleId)
                        continue;

                    pickupDropoffPairs.push_back(std::make_tuple(pVeh, dVeh));
                }
            }
        }


        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
    };

}