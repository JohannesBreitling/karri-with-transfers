/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/TimeUtils.h"

namespace karri {

    // Decides whether a rider accepts an assignment based on the request and the assignment finder response.
    // Simple implementation that only checks whether the assignment trip time is below a threshold which is a linear
    // function on the direct time from origin to destination.
    class TripTimeThresholdAssignmentAcceptance {

    public:

        explicit TripTimeThresholdAssignmentAcceptance(const RouteState& routeState) : routeState(routeState),
                                                                                       detourComputer(routeState) {}

        template<typename AsgnFinderResponseT>
        bool doesRiderAcceptAssignment(const Request &req, const AsgnFinderResponseT &resp) {
            using namespace time_utils;

            if (InputConfig::getInstance().epsilon == 0.0) {
                return true; // no trip time threshold, accept all assignments
            }

            const auto &bestAsgnType = resp.getBestAsgnType();
            if (bestAsgnType == RequestState::BestAsgnType::INVALID) {
                return false; // no assignment found
            }

            int tripTime;
            if (bestAsgnType == RequestState::BestAsgnType::NOT_USING_VEHICLE) {
                tripTime = resp.getNotUsingVehicleDist();
            } else if (bestAsgnType == RequestState::BestAsgnType::ONE_LEG) {
                const auto &bestAsgn = resp.getBestAssignmentWithoutTransfer();
                const auto depTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, resp, routeState);
                const auto initialPickupDetour = calcInitialPickupDetour(bestAsgn, depTimeAtPickup, resp, routeState);
                const auto dropoffAtExistingStop = isDropoffAtExistingStop(bestAsgn, routeState);
                const auto arrTimeAtDropoff = getArrTimeAtDropoff(depTimeAtPickup, bestAsgn, initialPickupDetour, dropoffAtExistingStop, routeState);
                tripTime = arrTimeAtDropoff + bestAsgn.dropoff->walkingDist - req.requestTime;
            } else {
                KASSERT(bestAsgnType == RequestState::BestAsgnType::TWO_LEGS);
                const auto &bestAsgn = resp.getBestAssignmentWithTransfer();
                const auto actualDepTimeAtPickup = getActualDepTimeAtPickup(bestAsgn, resp, routeState);
                const bool transferPVehAtExistingStop = isTransferAtExistingStopPVeh(bestAsgn, routeState);

                detourComputer.computeDetours(bestAsgn, resp);
                const int riderArrTimeAtTransfer =
                        computeRiderArrTimeAtTransfer(bestAsgn, actualDepTimeAtPickup, transferPVehAtExistingStop,
                                                      detourComputer, routeState);
                const int depTimeAtTransfer = computeDVehDepTimeAtTransfer(bestAsgn, riderArrTimeAtTransfer,
                                                                           detourComputer, routeState, resp);
                const int arrTimeAtDropoff = computeArrTimeAtDropoffAfterTransfer(
                        bestAsgn, depTimeAtTransfer, detourComputer, routeState);
                tripTime = arrTimeAtDropoff + bestAsgn.dropoff->walkingDist - req.requestTime;
            }

            const auto directTime = resp.originalReqDirectDist;
            const auto threshold = InputConfig::getInstance().epsilon * directTime + InputConfig::getInstance().phi;
            return tripTime <= threshold;
        }

    private:

        const RouteState &routeState;
        time_utils::DetourComputer detourComputer;

    };

} // karri

