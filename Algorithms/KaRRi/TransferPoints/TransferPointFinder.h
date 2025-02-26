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
            transferPoints = std::map<std::tuple<int, int>, std::vector<TransferPoint>>{};
        }

        void findTransferPoints(const Vehicle &pVeh, const Vehicle &dVeh) {
            numSearchesRun = 0;
            numEdgesRelaxed = 0;
            numVerticesScanned = 0;

            // Get the stop locations of the two vehicles               
            const auto &stopLocationsPVeh = routeState.stopLocationsFor(pVeh.vehicleId);
            const auto &stopLocationsDVeh = routeState.stopLocationsFor(dVeh.vehicleId);

            const auto &stopIdsPVeh = routeState.stopIdsFor(pVeh.vehicleId);
            const auto &stopIdsDVeh = routeState.stopIdsFor(dVeh.vehicleId);

            const int numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(dVeh.vehicleId);
            
            strategy.findTransferPoints(pVeh, dVeh, numStopsPVeh, numStopsDVeh, stopLocationsPVeh, stopLocationsDVeh, stopIdsPVeh, stopIdsDVeh);

            numSearchesRun = strategy.getNumSearchesRun();
            numEdgesRelaxed = strategy.getNumEdgesRelaxed();
            numVerticesScanned = strategy.getNumVerticesScanned();
        }

        int64_t getNumSearchesRun() {
            return numSearchesRun;
        }

        int64_t getNumEdgesRelaxed() {
            return numEdgesRelaxed;
        }

        int64_t getNumVerticesScanned() {
            return numVerticesScanned;
        }

    private:
        StrategyT &strategy;
        const RouteState &routeState;
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;

        int64_t numSearchesRun;
        int64_t numEdgesRelaxed;
        int64_t numVerticesScanned;

    };

}