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
            const RouteState &routeState,
            std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints
        ) : strategy(strategy),
            routeState(routeState),
            transferPoints(transferPoints) {}

        void init() {
            transferPoints.clear();
        }

        void findTransferPoints(Vehicle &pVeh, Vehicle &dVeh) {

            // Get the stop locations of the two vehicles               
            const auto &stopLocationsPVeh = routeState.stopLocationsFor(pVeh.vehicleId);
            const auto &stopLocationsDVeh = routeState.stopLocationsFor(dVeh.vehicleId);

            const auto &stopIdsPVeh = routeState.stopIdsFor(pVeh.vehicleId);
            const auto &stopIdsDVeh = routeState.stopIdsFor(dVeh.vehicleId);

            const int numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(dVeh.vehicleId);
            
            strategy.findTransferPoints(pVeh, dVeh, numStopsPVeh, numStopsDVeh, stopLocationsPVeh, stopLocationsDVeh, stopIdsPVeh, stopIdsDVeh);

            



            // int maxDetourPickup, int maxDetourDropoff, int pickupVehicleStop, int pickupVehicleNextStop, int dropoffVehicleStop, int dropoffVehicleNextStop



            
            /*
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

            

            // Use the strategy to find the points for a possible transfer
            strategy.setMaxDetours(maxDetourPickup, maxDetourDropoff);                    
            strategy.findTransferPoints(pickupVehicleStop, pickupVehicleNextStop, dropoffVehicleStop, dropoffVehicleNextStop);
            */
        }

    private:
        StrategyT &strategy;
        const RouteState &routeState;
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;
    };

}