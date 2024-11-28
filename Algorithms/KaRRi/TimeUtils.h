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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"
#include "cassert"

#define DO_INLINE true
#if DO_INLINE
#define INLINE inline
#else
#define INLINE
#endif

namespace karri::time_utils {

    template<typename RequestContext>
    static INLINE int getVehDepTimeAtStopForRequest(const int vehId, const int stopIdx, const RequestContext &context,
                                                    const RouteState &routeState) {
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        return (numStops == 1 ? std::max(minDepTimes[0], context.originalRequest.requestTime) : minDepTimes[stopIdx]);
    }

    static INLINE int getVehDepTimeAtStopForTransfer(const int vehId, const int stopIdx, const int arrAtTransfer,
                                                    const RouteState &routeState) {
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        return (numStops == 1 ? std::max(minDepTimes[0], arrAtTransfer) : minDepTimes[stopIdx]);
    }

    static INLINE bool isMakingStop(const int vehId, const int now, const RouteState &routeState) {
        return routeState.schedDepTimesFor(vehId)[0] > now;
    }

    static INLINE bool isPickupAtExistingStop(const PDLoc &pickup, const int vehId, const int now, const int stopIndex,
                                              const RouteState &routeState) {
        return (stopIndex > 0 || isMakingStop(vehId, now, routeState)) &&
               pickup.loc == routeState.stopLocationsFor(vehId)[stopIndex];
    }

    template <typename InputGraphT>
    static INLINE bool isTransferAtExistingStop(const int transferLoc, const int vehId, const int now, const int stopIndex,
                                              const RouteState &routeState, const InputGraphT &inputGraph) {
        const int tailOfStop = inputGraph.edgeTail(routeState.stopLocationsFor(vehId)[stopIndex]);
        
        return (stopIndex > 0 || isMakingStop(vehId, now, routeState)) &&
               transferLoc == tailOfStop;
    }

    template<typename LabelSet>
    static INLINE typename LabelSet::LabelMask
    isPickupAtExistingStop(const typename LabelSet::DistanceLabel &pickupLocs, const int vehId, const int now,
                           const int stopIndex, const RouteState &routeState) {
        if (!(stopIndex > 0 || isMakingStop(vehId, now, routeState))) return false;

        return pickupLocs == typename LabelSet::DistanceLabel(routeState.stopLocationsFor(vehId)[stopIndex]);
    }

    template<typename LabelSet>
    static INLINE typename LabelSet::LabelMask
    isPickupAtExistingStop(const std::array<PDLoc, LabelSet::K> &pickups, const int vehId, const int now,
                           const int stopIndex, const RouteState &routeState) {
        typename LabelSet::DistanceLabel pickupLocations;
        for (int i = 0; i < LabelSet::K; ++i) {
            pickupLocations[i] = pickups[i].location;
        }
        return isPickupAtExistingStop(pickupLocations, vehId, now, stopIndex, routeState);
    }

    template<typename RequestContext>
    static INLINE int getActualDepTimeAtPickup(const int vehId, const int stopIndexBeforePickup, const int distToPickup,
                                               const PDLoc &pickup, const RequestContext &context,
                                               const RouteState &routeState) {
        const bool atStop = isPickupAtExistingStop(pickup, vehId, context.now(), stopIndexBeforePickup, routeState);
        const auto minVehicleDepTimeAtPickup =
                getVehDepTimeAtStopForRequest(vehId, stopIndexBeforePickup, context, routeState) +
                !atStop * (distToPickup + InputConfig::getInstance().stopTime);

        // We assume a pickup at an existing stop takes no additional counting of stopTime, irrespective of when the
        // passenger arrives there. The vehicle can depart as soon as both the vehicle and the passenger are at the
        // location. This is how LOUD originally did it, so we adhere to it.
        return std::max(minVehicleDepTimeAtPickup, context.getPassengerArrAtPickup(pickup.id));
    }

    template<typename RequestContext, typename InputGraphT>
    static INLINE int getActualDepTimeAtTranfer(const AssignmentWithTransfer &asgn, const RequestContext &context, RouteState &routeState, const InputGraphT &inputGraph) {
        const bool atStop = isTransferAtExistingStop(asgn.transfer.loc, asgn.pVeh->vehicleId, context.now(), asgn.transferIdxDVeh, routeState, inputGraph);
        const auto minVehicleDepTimeAtTransfer = getVehDepTimeAtStopForTransfer(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.arrAtTransferPoint, routeState) + !atStop * (asgn.distToTransferDVeh + InputConfig::getInstance().stopTime);

        return std::max(minVehicleDepTimeAtTransfer, asgn.arrAtTransferPoint);
    }

    template<typename RequestContext>
    static INLINE int
    getActualDepTimeAtPickup(const Assignment &asgn, const RequestContext &context, const RouteState &routeState) {
        return getActualDepTimeAtPickup(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.distToPickup, *asgn.pickup,
                                        context, routeState);
    }

    template<typename RequestContext>
    static INLINE int getActualDepTimeAtPickup(const AssignmentWithTransfer &asgn, const RequestContext &context, const RouteState &routeState) {
        return getActualDepTimeAtPickup(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.distToPickup, *asgn.pickup, context, routeState);
    }

    template<typename LabelSet, typename RequestContext>
    static INLINE typename LabelSet::DistanceLabel
    getActualDepTimeAtPickup(const int vehId, const int stopIndexBeforePickup,
                             const typename LabelSet::DistanceLabel &distToPickup,
                             const typename LabelSet::DistanceLabel &pickupLocs,
                             const typename LabelSet::DistanceLabel &passengerArrTimesAtPickups,
                             const RequestContext &context,
                             const RouteState &routeState) {
        using LabelMask = typename LabelSet::LabelMask;

        const LabelMask atStop = isPickupAtExistingStop<LabelSet>(pickupLocs, vehId, context.now(),
                                                                  stopIndexBeforePickup, routeState);

        // We assume a pickup at an existing stop takes no additional counting of stopTime, irrespective of when the
        // passenger arrives there. The vehicle can depart as soon as both the vehicle and the passenger are at the
        // location. This is how LOUD originally did it, so we adhere to it.
        auto minDepTimeAtPickup = getVehDepTimeAtStopForRequest(vehId, stopIndexBeforePickup, context, routeState) +
                                  select(atStop, 0, distToPickup + InputConfig::getInstance().stopTime);
        minDepTimeAtPickup.max(passengerArrTimesAtPickups);
        return minDepTimeAtPickup;
    }

    static INLINE int
    calcLengthOfLegStartingAt(const int stopIndex, const int vehicleId, const RouteState &routeState) {
        if (stopIndex + 1 == routeState.numStopsOf(vehicleId))
            return 0;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehicleId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehicleId);
        return minArrTimes[stopIndex + 1] - minDepTimes[stopIndex];
    }

    static INLINE bool isDropoffAtExistingStop(const Assignment &asgn, const RouteState &routeState) {
        return asgn.pickupStopIdx != asgn.dropoffStopIdx &&
               asgn.dropoff->loc == routeState.stopLocationsFor(asgn.vehicle->vehicleId)[asgn.dropoffStopIdx];
    }

    static INLINE bool isDropoffAtExistingStop(const AssignmentWithTransfer &asgn, const RouteState &routeState) {
        return asgn.transferIdxDVeh != asgn.dropoffIdx &&
               asgn.dropoff->loc == routeState.stopLocationsFor(asgn.pVeh->vehicleId)[asgn.dropoffIdx];
    }

    static INLINE int
    getArrTimeAtDropoff(const int actualDepTimeAtPickup, const Assignment &asgn, const int initialPickupDetour,
                        const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto pickupIndex = asgn.pickupStopIdx;
        const auto dropoffIndex = asgn.dropoffStopIdx;
        const auto &minDepTimes = routeState.schedDepTimesFor(asgn.vehicle->vehicleId);
        const auto &minArrTimes = routeState.schedArrTimesFor(asgn.vehicle->vehicleId);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(asgn.vehicle->vehicleId);

        if (pickupIndex == dropoffIndex) {
            return actualDepTimeAtPickup + asgn.distToDropoff;
        }

        assert(dropoffIndex > 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[dropoffIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious =
                minArrTimes[dropoffIndex] + std::max(initialPickupDetour - stopLengthsBetween, 0);

        if (dropoffAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[dropoffIndex], arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToDropoff;
    }

    static INLINE int
    getArrTimeAtDropoff(const int actualDepTimeAtTransfer, const AssignmentWithTransfer &asgn, const int initialTransferDetour, const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto vehIdDVeh = asgn.dVeh->vehicleId;
        const auto pickupIndex = asgn.transferIdxDVeh;
        const auto dropoffIndex = asgn.dropoffIdx;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehIdDVeh);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehIdDVeh);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehIdDVeh);
    
        if (pickupIndex == dropoffIndex) {
            return actualDepTimeAtTransfer + asgn.distToDropoff;
        }

        assert(dropoffIndex > 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[dropoffIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious =
                minArrTimes[dropoffIndex] + std::max(initialTransferDetour - stopLengthsBetween, 0);

        if (dropoffAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[dropoffIndex], arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToDropoff;
    }

    static INLINE int
    getArrTimeAtTransfer(const int actualDepTimeAtPickup, const AssignmentWithTransfer &asgn, const int initialPickupDetour,
                        const bool transferAtExistingStop, const RouteState &routeState) {
        const auto vehIdPVeh = asgn.pVeh->vehicleId; 
        const auto pickupIndex = asgn.pickupIdx;
        const auto transferIndex = asgn.transferIdxPVeh;
        const auto &minDepTimes = routeState.schedDepTimesFor(vehIdPVeh);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehIdPVeh);
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehIdPVeh);

        if (pickupIndex == transferIndex) {
            return actualDepTimeAtPickup + asgn.distToTransferPVeh;
        }

        assert(transferIndex > 0);
        const auto stopLengthsBetween = vehWaitTimesPrefixSum[transferIndex - 1] - vehWaitTimesPrefixSum[pickupIndex];
        const auto arrTimeAtPrevious = minArrTimes[transferIndex] + std::max(initialPickupDetour - stopLengthsBetween, 0);

        if (transferAtExistingStop) {
            return arrTimeAtPrevious;
        }

        const auto depTimeAtPrevious = std::max(minDepTimes[transferIndex], arrTimeAtPrevious + InputConfig::getInstance().stopTime);
        return depTimeAtPrevious + asgn.distToTransferPVeh;
    }

    // Returns the accumulated vehicle wait time in the stop interval (fromIndex, toIndex].
    static INLINE int
    getTotalVehWaitTimeInInterval(const int vehId, const int fromIndex, const int toIndex,
                                  const RouteState &routeState) {
        if (fromIndex >= toIndex) return 0;
        const auto &vehWaitTimesPrefixSum = routeState.vehWaitTimesPrefixSumFor(vehId);
        return vehWaitTimesPrefixSum[toIndex] - vehWaitTimesPrefixSum[fromIndex];
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const int vehId, const int pickupIndex, const int dropoffIndex, const int depTimeAtPickup,
                            const int distFromPickup, const RequestContext &context,
                            const RouteState &routeState) {
        const auto vehDepTimeAtPrevStop = getVehDepTimeAtStopForRequest(vehId, pickupIndex, context, routeState);
        const auto timeUntilDep = depTimeAtPickup - vehDepTimeAtPrevStop;

        if (pickupIndex == dropoffIndex)
            return timeUntilDep;

        return timeUntilDep + distFromPickup - calcLengthOfLegStartingAt(pickupIndex, vehId, routeState);
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.pickupStopIdx to the given pickup, perform the pickup and drive to its next scheduled stop at index
    // asgn.pickupStopIdx + 1 instead of driving from the stop at asgn.pickupStopIdx directly to the stop
    // at asgn.pickupStopIdx + 1.
    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const Assignment &asgn, const int depTimeAtPickup, const RequestContext &context,
                            const RouteState &routeState) {
        return calcInitialPickupDetour(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                       depTimeAtPickup, asgn.distFromPickup,
                                       context, routeState);
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const AssignmentWithTransfer &asgn, const int depTimeAtPickup, const RequestContext &context,
                            const RouteState &routeState) {
        return calcInitialPickupDetour(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.transferIdxPVeh,
                                       depTimeAtPickup, asgn.distFromPickup,
                                       context, routeState);
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialTransferDetourDVeh(const AssignmentWithTransfer &asgn, int depTimeAtTransfer, const RequestContext /* &context */, const RouteState &routeState) {
        const auto vehDepTimeAtPrevStop = getVehDepTimeAtStopForTransfer(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.arrAtTransferPoint, routeState);
        const auto timeUntilDep = depTimeAtTransfer - vehDepTimeAtPrevStop;

        if (asgn.transferIdxDVeh == asgn.dropoffIdx)
            return timeUntilDep;

        const auto legLength = calcLengthOfLegStartingAt(asgn.transferIdxDVeh, asgn.dVeh->vehicleId, routeState);

        /*
        if (legLength > asgn.distFromTransferDVeh) {
            std::cout << "Leg Length : " << legLength << " DistFromTransfer: " << asgn.distFromTransferDVeh << std::endl;
        }
        */

        return timeUntilDep + asgn.distFromTransferDVeh - legLength;
        // return calcInitialPickupDetour(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.dropoffIdx, depTimeAtTransfer, asgn.distFromTransferDVeh, context, routeState);
    }

    template<typename RequestContext>
    static INLINE int
    calcInitialPickupDetour(const Assignment &asgn, const RequestContext &context,
                            const RouteState &routeState) {
        const auto actualDepTime = getActualDepTimeAtPickup(asgn, context, routeState);
        return calcInitialPickupDetour(asgn, actualDepTime, context, routeState);
    }

    static INLINE int
    calcOnlyDrivingTimeInInitialPickupDetour(const Assignment &asgn, const bool isPickupAtExistingStop,
                                             const RouteState &routeState) {
        if (isPickupAtExistingStop) return 0;
        if (asgn.pickupStopIdx == asgn.dropoffStopIdx)
            return asgn.distToPickup;
        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(asgn.pickupStopIdx, asgn.vehicle->vehicleId,
                                                                   routeState);
        return asgn.distToPickup + asgn.distFromPickup - lengthOfReplacedLeg;
    }

    // Returns the additional time needed for the vehicle to perform a pickup with initialPickupDetour after its stop
    // at pickupIndex and drive until stop toIndex instead of going to stop toIndex according to its current schedule.
    // The residual pickup detour can be smaller than the initial detour since the vehicle may currently wait for
    // passengers at stops between (pickupIndex, toIndex) which is time that it can now spend driving instead which
    // reduces the additional operation time incurred by the new pickup compared to the initial detour for the pickup.
    // Does not subtract the vehicle wait at toIndex itself if one exists, i.e. this is the residual pickup detour that
    // arrives at toIndex.
    static INLINE int
    calcResidualPickupDetour(const int vehId, const int pickupIndex, const int toIndex, const int initialPickupDetour,
                             const RouteState &routeState) {
        const auto vehWaitTime = getTotalVehWaitTimeInInterval(vehId, pickupIndex, toIndex - 1, routeState);
        return std::max(initialPickupDetour - vehWaitTime, 0);
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.dropoffStopIdx to the given dropoff, perform the dropoff and drive to its next scheduled stop at index
    // asgn.dropoffStopIdx + 1 instead of driving from the stop at asgn.dropoffStopIdx directly to the stop
    // at asgn.dropoffStopIdx + 1.
    static INLINE int
    calcInitialDropoffDetour(const int vehId, const int dropoffIndex, const int distToDropoff,
                             const int distFromDropoff,
                             const bool dropoffAtExistingStop,
                             const RouteState &routeState) {
        if (dropoffAtExistingStop) return 0;
        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(dropoffIndex, vehId, routeState);
        return distToDropoff + InputConfig::getInstance().stopTime + distFromDropoff - lengthOfReplacedLeg;
    }

    // Returns the additional time that is needed for the vehicle asgn.vehicle to drive from its stop at index
    // asgn.dropoffStopIdx to the given dropoff, perform the dropoff and drive to its next scheduled stop at index
    // asgn.dropoffStopIdx + 1 instead of driving from the stop at asgn.dropoffStopIdx directly to the stop
    // at asgn.dropoffStopIdx + 1.
    static INLINE int
    calcInitialDropoffDetour(const Assignment &asgn, const bool dropoffAtExistingStop,
                             const RouteState &routeState) {
        return calcInitialDropoffDetour(asgn.vehicle->vehicleId, asgn.dropoffStopIdx, asgn.distToDropoff,
                                        asgn.distFromDropoff, dropoffAtExistingStop, routeState);
    }

    static INLINE int
    calcInitialDropoffDetour(const AssignmentWithTransfer &asgn, const bool dropoffAtExistingStop, const RouteState &routeState) {
        return calcInitialDropoffDetour(asgn.dVeh->vehicleId, asgn.dropoffIdx, asgn.distToDropoff, asgn.distFromDropoff, dropoffAtExistingStop, routeState);
    }

    static INLINE int
    calcInitialTransferDetourPVeh(const AssignmentWithTransfer &asgn, const bool transferAtExistingStop, const RouteState routeState) {
        if (transferAtExistingStop) return 0;

        const auto lengthOfReplacedLeg = calcLengthOfLegStartingAt(asgn.transferIdxPVeh, asgn.pVeh->vehicleId, routeState);
        return asgn.distToTransferPVeh + InputConfig::getInstance().stopTime + asgn.distFromTransferPVeh - lengthOfReplacedLeg;
    }

    static INLINE int
    calcDetourRightAfterDropoff(const int vehId, const int pickupIndex, const int dropoffIndex,
                                const int initialPickupDetour, const int initialDropoffDetour,
                                const RouteState &routeState) {
        const auto detour =
                calcResidualPickupDetour(vehId, pickupIndex, dropoffIndex + 1, initialPickupDetour, routeState) +
                initialDropoffDetour;
        assert(detour >= 0 || pickupIndex == dropoffIndex);
        return std::max(detour, 0);
    }

    static INLINE int
    calcDetourRightAfterDropoff(const Assignment &asgn, const int initialPickupDetour, const int initialDropoffDetour,
                                const RouteState &routeState) {
        return calcDetourRightAfterDropoff(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                           initialPickupDetour, initialDropoffDetour, routeState);
    }

    static INLINE int
    calcDetourRightAfterDropoff(const AssignmentWithTransfer &asgn, const int initialTransferDetour, const int initalDropoffDetour, const RouteState &routeState) {
        return calcDetourRightAfterDropoff(asgn.dVeh->vehicleId, asgn.transferIdxDVeh, asgn.dropoffIdx, initialTransferDetour, initalDropoffDetour, routeState);
    }

    static INLINE int
    calcDetourRightAfterTransferPVeh(const AssignmentWithTransfer &asgn, const int initialPickupDetour, const int initialTransferDetour, const RouteState &routeState) {
        const auto detour = calcResidualPickupDetour(asgn.pVeh->vehicleId, asgn.pickupIdx, asgn.transferIdxPVeh + 1, initialPickupDetour, routeState) + initialTransferDetour;
        
        assert(detour >= 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
        return std::max(detour, 0);
    }

    static INLINE int
    calcResidualTotalDetourForStopAfterDropoff(const int vehId, const int dropoffIndex, const int toIndex,
                                               const int detourRightAfterDropoff, const RouteState &routeState) {
        assert(toIndex >= dropoffIndex);
        const auto vehWaitTime = getTotalVehWaitTimeInInterval(vehId, dropoffIndex, toIndex - 1, routeState);
        return std::max(detourRightAfterDropoff - vehWaitTime, 0);
    }

    static INLINE int
    calcResidualTotalDetour(const int vehId, const int pickupIndex, const int dropoffIndex, const int toIndex,
                            const int initialPickupDetour, const int detourRightAfterDropoff,
                            const RouteState &routeState) {
        assert(toIndex >= pickupIndex);
        if (toIndex <= dropoffIndex) {
            return calcResidualPickupDetour(vehId, pickupIndex, toIndex, initialPickupDetour, routeState);
        }

        return calcResidualTotalDetourForStopAfterDropoff(vehId, dropoffIndex, toIndex, detourRightAfterDropoff,
                                                          routeState);
    }

    static INLINE int calcResidualTotalDetour(const Assignment &asgn, const int toIndex, const int initialPickupDetour,
                                              const int detourRightAfterDropoff, const RouteState &routeState) {
        return calcResidualTotalDetour(asgn.vehicle->vehicleId, asgn.pickupStopIdx, asgn.dropoffStopIdx,
                                       toIndex, initialPickupDetour, detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeInInterval(const int vehId, const int fromIndex, const int toIndex, const int detourAtFromIndex,
                                const RouteState &routeState) {

        assert(detourAtFromIndex >= 0);
        if (detourAtFromIndex == 0 || fromIndex == toIndex) {
            return 0;
        }
        const auto &vehWaitTimePrefixSums = routeState.vehWaitTimesPrefixSumFor(vehId);
        const auto &vehWaitTimesAtDropoffsPrefixSums = routeState.vehWaitTimesUntilDropoffsPrefixSumsFor(vehId);
        const auto &numDropoffsPrefixSums = routeState.numDropoffsPrefixSumFor(vehId);

        // We consider the interval sum of vehicle wait times between fromIndex and an existing dropoff of a
        // request r as a buffer to the added trip time for r since those vehicle wait times will now be used for
        // driving the detour instead.
        // Let toIndex := initial given value of resDetourUntilIdx.
        // Let d be an existing dropoff between fromIndex and toIndex. If the buffer from fromIndex to d is smaller
        // than the detour at fromIndex, then there is a residual detour that the passenger of d experiences as
        // added trip time. If the buffer between fromIndex and d is large enough to counteract the whole detour, then
        // there is no added trip time.
        // We find the largest index resDetourUntilIdx that still has a residual detour by decrementing
        // starting at toIndex. Then, all dropoffs later than resDetourUntilIdx experience no added trip time and all
        // dropoffs between fromIndex and resDetourUntilIdx do.
        // For the dropoffs before resDetourUntilIdx, we can then simply subtract the sum of the buffers that each
        // dropoff can utilize from the sum of detours (i.e. the given detour * the number of affected dropoffs).
        // This only works if all affected dropoffs experience a positive residual detour since otherwise in the sum a
        // "negative" detour at a later dropoff would be able to reduce the added trip time of an earlier dropoff which
        // we have to prevent.
        //
        // In most cases, we expect a vehicle to only have a small total wait time so resDetourUntilIdx will likely
        // simply be equal to toIndex. Thus, in most cases, we can compute the added trip time in constant time.
        int resDetourUntilIdx = toIndex;
        while (resDetourUntilIdx > fromIndex &&
               getTotalVehWaitTimeInInterval(vehId, fromIndex, resDetourUntilIdx - 1, routeState) >=
               detourAtFromIndex) {
            --resDetourUntilIdx;
        }
        assert(getTotalVehWaitTimeInInterval(vehId, fromIndex, resDetourUntilIdx - 1, routeState) < detourAtFromIndex);

        const auto numDropoffsInInterval = numDropoffsPrefixSums[resDetourUntilIdx] - numDropoffsPrefixSums[fromIndex];
        const auto sumOfBuffersOfDropoffsInInterval =
                vehWaitTimesAtDropoffsPrefixSums[resDetourUntilIdx] - vehWaitTimesAtDropoffsPrefixSums[fromIndex] -
                numDropoffsInInterval * vehWaitTimePrefixSums[fromIndex];
        return numDropoffsInInterval * detourAtFromIndex - sumOfBuffersOfDropoffsInInterval;
    }


    static INLINE int calcAddedTripTimeAffectedByPickupAndDropoff(const int vehId, const int newDropoffIndex,
                                                                  const int detourRightAfterDropoff,
                                                                  const RouteState &routeState) {
        assert(detourRightAfterDropoff >= 0);
        const auto &numStops = routeState.numStopsOf(vehId);
        assert(newDropoffIndex < numStops);
        return calcAddedTripTimeInInterval(vehId, newDropoffIndex, numStops - 1, detourRightAfterDropoff, routeState);
    }

    static INLINE int calcAddedTripTimeAffectedByTransferAndDropoff(const int vehId, const int newDropoffIndex, const int detourRightAfterDropoff, const RouteState &routeState) {
        assert(detourRightAfterDropoff >= 0);
        const auto numStops = routeState.numStopsOf(vehId);
        assert(newDropoffIndex < numStops);

        return calcAddedTripTimeInInterval(vehId, newDropoffIndex, numStops - 1, detourRightAfterDropoff, routeState);
    }

    static INLINE int calcAddedTripTimeAffectedByTransferAndDropoff(const AssignmentWithTransfer &asgn, const int detourRightAfterDropoff, const RouteState &routeState) {
        return calcAddedTripTimeAffectedByTransferAndDropoff(asgn.dVeh->vehicleId, asgn.pickupIdx, detourRightAfterDropoff, routeState);
    }

    static INLINE int
    calcAddedTripTimeAffectedByPickupAndDropoff(const Assignment &asgn, const int detourRightAfterDropoff,
                                                const RouteState &routeState) {
        return calcAddedTripTimeAffectedByPickupAndDropoff(asgn.vehicle->vehicleId, asgn.dropoffStopIdx,
                                                           detourRightAfterDropoff, routeState);
    }

    static INLINE int calcAddedTripTimeAffectedByPickupAndTransfer(const AssignmentWithTransfer &asgn, const int detourRightAfterTransfer, const RouteState &routeState) {
        return calcAddedTripTimeAffectedByPickupAndDropoff(asgn.pVeh->vehicleId, asgn.transferIdxPVeh, detourRightAfterTransfer, routeState);
    }

    template<typename RequestContext>
    static INLINE bool isAnyHardConstraintViolated(const Vehicle &veh, const int pickupIndex, const int dropoffIndex,
                                                   const RequestContext &context, const int initialPickupDetour,
                                                   const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                                   const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);
        const auto &occupancies = routeState.occupanciesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request if the vehicle is currently
        // idling) is moved past the end of the service time by the total detour, the assignment violates the service
        // time constraint.
        if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (pickupIndex + 1 == numStops)
            return false;

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (pickupIndex != dropoffIndex && initialPickupDetour != 0 &&
            minArrTimes[pickupIndex + 1] + initialPickupDetour > maxArrTimes[pickupIndex + 1])
            return true;

        // If somewhere between pickup and dropoff the vehicle is already full, we cannot insert another passenger.
        for (int i = pickupIndex; i < dropoffIndex; ++i)
            if (occupancies[i] + context.originalRequest.numRiders > veh.capacity)
                return true;
        if (!dropoffAtExistingStop && occupancies[dropoffIndex] + context.originalRequest.numRiders > veh.capacity)
            return true;

        // If the dropoff is inserted at/after the last stop, the service time constraint is kept and the pickup does
        // not violate the trip time or wait time constraints, the assignment is ok.
        if (dropoffIndex + 1 == numStops)
            return false;

        // If the total detour moves the planned arrival time at the stop after the dropoff past the latest permissible
        // arrival time, this assignment violates some trip time or wait time constraint.
        if (detourRightAfterDropoff != 0 &&
            minArrTimes[dropoffIndex + 1] + detourRightAfterDropoff > maxArrTimes[dropoffIndex + 1])
            return true;

        return false;
    }

    template<typename RequestContext>
    static INLINE bool
    isAnyHardConstraintViolated(const AssignmentWithTransfer &asgn, const RequestContext &context, const int initialPickupDetour,
                                const int detourRightAfterTransfer, const int residualDetourAtEnd,
                                const bool transferAtExistingStop, const RouteState &routeState) {
        return isAnyHardConstraintViolated(*asgn.pVeh, asgn.pickupIdx, asgn.transferIdxPVeh, context, initialPickupDetour, detourRightAfterTransfer, residualDetourAtEnd, transferAtExistingStop, routeState);                                    
    }

    template<typename RequestContext>
    static INLINE bool
    isAnyHardConstraintViolated(const Assignment &asgn, const RequestContext &context, const int initialPickupDetour,
                                const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                const bool dropoffAtExistingStop, const RouteState &routeState) {
        return isAnyHardConstraintViolated(*asgn.vehicle, asgn.pickupStopIdx, asgn.dropoffStopIdx, context,
                                           initialPickupDetour, detourRightAfterDropoff, residualDetourAtEnd,
                                           dropoffAtExistingStop, routeState);
    }

    template<typename RequestContext>
    static INLINE bool isAnyHardConstraintViolated(const int arrAtTransfer, const Vehicle &dVeh, const int transferIdx, const int dropoffIdx,
                                                   const RequestContext &context, const int initialTransferDetour,
                                                   const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                                   const bool dropoffAtExistingStop, const RouteState &routeState) {
        const auto vehId = dVeh.vehicleId;
        const auto endServiceTime = dVeh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);
        const auto &occupancies = routeState.occupanciesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request if the vehicle is currently
        // idling) is moved past the end of the service time by the total detour, the assignment violates the service
        // time constraint.
        if (std::max(minDepTimes[numStops - 1], arrAtTransfer) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (transferIdx + 1 == numStops)
            return false;

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (transferIdx != dropoffIdx && initialTransferDetour != 0 &&
            minArrTimes[transferIdx + 1] + initialTransferDetour > maxArrTimes[transferIdx + 1])
            return true;

        // If somewhere between pickup and dropoff the vehicle is already full, we cannot insert another passenger.
        for (int i = transferIdx; i < transferIdx; ++i)
            if (occupancies[i] + context.originalRequest.numRiders > dVeh.capacity)
                return true;
        if (!dropoffAtExistingStop && occupancies[transferIdx] + context.originalRequest.numRiders > dVeh.capacity)
            return true;

        // If the dropoff is inserted at/after the last stop, the service time constraint is kept and the pickup does
        // not violate the trip time or wait time constraints, the assignment is ok.
        if (transferIdx + 1 == numStops)
            return false;

        // If the total detour moves the planned arrival time at the stop after the dropoff past the latest permissible
        // arrival time, this assignment violates some trip time or wait time constraint.
        if (detourRightAfterDropoff != 0 &&
            minArrTimes[dropoffIdx + 1] + detourRightAfterDropoff > maxArrTimes[dropoffIdx + 1])
            return true;

        return false;
    }

    template<typename RequestContext>
    static INLINE bool isAnyHardConstraintViolated(const AssignmentWithTransfer &asgn, RequestContext &context,
                                                   const int initalTransferDetour, const int detourRightAfterDropoff, const int residualDetourAtEnd,
                                                   const bool dropoffAtExistingStop, const RouteState &routeState) {
        return isAnyHardConstraintViolated(asgn.arrAtTransferPoint, *asgn.dVeh, asgn.transferIdxDVeh, asgn.dropoffIdx, context,
                                           initalTransferDetour, detourRightAfterDropoff, residualDetourAtEnd, dropoffAtExistingStop, routeState);
    }

    template<typename RequestContext>
    static INLINE bool
    isServiceTimeConstraintViolated(const Vehicle &veh, const RequestContext &context, const int residualDetourAtEnd,
                                    const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);

        return std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
               endServiceTime;
    }

    template<typename RequestContext>
    static INLINE bool
    doesPickupDetourViolateHardConstraints(const Vehicle &veh, const RequestContext &context, const int pickupIndex,
                                           const int initialPickupDetour, const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request) is moved past
        // the end of the service time by the pickup detour, the assignment violates the service time constraint.
        const auto residualDetourAtEnd = calcResidualPickupDetour(vehId, pickupIndex, numStops - 1, initialPickupDetour,
                                                                  routeState);
        if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the pickup is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (pickupIndex + 1 == numStops)
            return false;

        // If the pickup detour moves the planned arrival time at the stop after the pickup past the latest permissible
        // arrival time, this assignment violates some trip time or wait time hard constraint.
        if (initialPickupDetour != 0 &&
            minArrTimes[pickupIndex + 1] + initialPickupDetour > maxArrTimes[pickupIndex + 1])
            return true;

        return false;
    }

    template<typename RequestContext>
    static INLINE bool
    doesDropoffDetourViolateHardConstraints(const Vehicle &veh, const RequestContext &context, const int dropoffIndex,
                                            const int initialDropoffDetour, const RouteState &routeState) {
        const auto vehId = veh.vehicleId;
        const auto endServiceTime = veh.endOfServiceTime;
        const auto numStops = routeState.numStopsOf(vehId);
        const auto &minDepTimes = routeState.schedDepTimesFor(vehId);
        const auto &minArrTimes = routeState.schedArrTimesFor(vehId);
        const auto &maxArrTimes = routeState.maxArrTimesFor(vehId);

        // If departure time at the last stop (which may be the time of issuing this request) is moved past
        // the end of the service time by the dropoff detour, the assignment violates the service time constraint.
        const auto residualDetourAtEnd = calcResidualTotalDetourForStopAfterDropoff(vehId, dropoffIndex, numStops - 1,
                                                                                    initialDropoffDetour, routeState);
        if (std::max(minDepTimes[numStops - 1], context.originalRequest.requestTime) + residualDetourAtEnd >
            endServiceTime)
            return true;

        // If the dropoff is inserted at/after the last stop and the service time constraint is not violated, the
        // assignment is ok.
        if (dropoffIndex + 1 == numStops)
            return false;

        // If the dropoff detour moves the planned arrival time at the stop after the dropoff past the latest
        // permissible arrival time, this assignment violates some trip time or wait time hard constraint.
        if (initialDropoffDetour != 0 &&
            minArrTimes[dropoffIndex + 1] + initialDropoffDetour > maxArrTimes[dropoffIndex + 1])
            return true;

        return false;
    }

    static INLINE int calcDetourForTransferPoint(const TransferPoint &tp, const RouteState &routeState) {

        auto pVeh = tp.pVeh;
        auto dVeh = tp.dVeh;

        assert(routeState.numStopsOf(pVeh->vehicleId) - 1 != tp.dropoffAtTransferStopIdx);
        assert(0 != tp.pickupFromTransferStopIdx);

        // Length in 10th seconds
        const auto lengthOfReplacedLegPVeh = calcLengthOfLegStartingAt(tp.dropoffAtTransferStopIdx, pVeh->vehicleId, routeState);
        const auto lengthOfReplacedLegDVeh = calcLengthOfLegStartingAt(tp.pickupFromTransferStopIdx, dVeh->vehicleId, routeState);

        assert(lengthOfReplacedLegPVeh > 0);
        assert(lengthOfReplacedLegDVeh > 0);
        // int sum = 0;

        int sumPVeh = tp.distancePVehToTransfer + tp.distancePVehFromTransfer + InputConfig::getInstance().stopTime - lengthOfReplacedLegPVeh;
        int sumDVeh = tp.distanceDVehToTransfer + tp.distanceDVehFromTransfer + InputConfig::getInstance().stopTime - lengthOfReplacedLegDVeh;
        
        return sumPVeh + sumDVeh;


    }

} // end namespace