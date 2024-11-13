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

#include "cassert"
#include "Algorithms/KaRRi/TimeUtils.h"

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
            DropoffVehicles &dVehs,
            std::vector<TransferPoint> &possibleTransferPoints
        ) : strategy(strategy),
            fleet(fleet), routeState(routeState),
            pVehs(pVehs), dVehs(dVehs),
            possibleTransferPoints(possibleTransferPoints) {}

        void init() {
            pVehs.init();
            dVehs.init();
            possibleTransferPoints = std::vector<TransferPoint>{};
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};         
        }

        void findTransferPoints(int maxDetourPickup, int maxDetourDropoff, int pickupVehicleStop, int pickupVehicleNextStop, int dropoffVehicleStop, int dropoffVehicleNextStop) {
            // Use the strategy to find the points for a possible transfer
            strategy.setMaxDetours(maxDetourPickup, maxDetourDropoff);                    
            strategy.findTransferPoints(pickupVehicleStop, pickupVehicleNextStop, dropoffVehicleStop, dropoffVehicleNextStop);
        }

    private:
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

        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
        std::vector<TransferPoint> &possibleTransferPoints;
    };

}