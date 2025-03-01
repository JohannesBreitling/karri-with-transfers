/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include <stack>

#include "Tools/Constants.h"
#include "DataStructures/Utilities/DynamicRagged2DArrays.h"
#include "DataStructures/Containers/BitVector.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"

namespace karri {

// Represents the state of all vehicle routes including the stop locations and schedules.
    class RouteState {

    public:
        RouteState(const Fleet &fleet)
                : pos(fleet.size()),
                  stopIds(fleet.size()),
                  stopLocations(fleet.size()),
                  schedArrTimes(fleet.size()),
                  schedDepTimes(fleet.size()),
                  maxArrTimes(fleet.size()),
                  occupancies(fleet.size()),
                  vehWaitTimesPrefixSum(fleet.size()),
                  vehWaitTimesUntilDropoffsPrefixSum(fleet.size()),
                  numDropoffsPrefixSum(fleet.size()),
                  stopIdToIdOfPrevStop(fleet.size(), INVALID_ID),
                  stopIdToPosition(fleet.size(), 0),
                  stopIdToLeeway(fleet.size(), 0),
                  stopIdToVehicleId(fleet.size(), INVALID_ID),
                  rangeOfRequestsPickedUpAtStop(fleet.size()),
                  requestsPickedUpAtStop(),
                  rangeOfRequestsDroppedOffAtStop(fleet.size()),
                  requestsDroppedOffAtStop(),
                  maxLeeway(0),
                  stopIdOfMaxLeeway(INVALID_ID),
                  maxLegLength(0),
                  stopIdOfMaxLegLength(INVALID_ID),
                  unusedStopIds(),
                  nextUnusedStopId(fleet.size()),
                  maxStopId(fleet.size() - 1) {
            for (auto i = 0; i < fleet.size(); ++i) {
                pos[i].start = i;
                pos[i].end = i + 1;
                stopIds[i] = i;
                stopIdToVehicleId[i] = i;
                stopLocations[i] = fleet[i].initialLocation;
                schedArrTimes[i] = fleet[i].startOfServiceTime;
                schedDepTimes[i] = fleet[i].startOfServiceTime;
                vehWaitTimesPrefixSum[i] = 0;
                occupancies[i] = 0;
                numDropoffsPrefixSum[i] = 0;
                vehWaitTimesUntilDropoffsPrefixSum[i] = 0;
                maxArrTimes[i] = INFTY;
            }
        }

        const int &getMaxStopId() const {
            return maxStopId;
        }

        int numStopsOf(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            return pos[vehId].end - pos[vehId].start;
        }

        // Range containing the ids of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopIdsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopIds.begin() + start, stopIds.begin() + end};
        }

        // Range containing the locations (= edges) of the currently scheduled stops of vehicle with given ID.
        ConstantVectorRange<int> stopLocationsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {stopLocations.begin() + start, stopLocations.begin() + end};
        }

        // Range containing the scheduled arrival times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedArrTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedArrTimes.begin() + start, schedArrTimes.begin() + end};
        }

        // Range containing the scheduled departure times of vehicle with given ID at its stops.
        ConstantVectorRange<int> schedDepTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {schedDepTimes.begin() + start, schedDepTimes.begin() + end};
        }

        // Range containing the latest possible arrival times of vehicle with given ID at its stops s.t. all hard
        // constraints of the vehicle and its passengers are still satisfied.
        ConstantVectorRange<int> maxArrTimesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {maxArrTimes.begin() + start, maxArrTimes.begin() + end};
        }

        // Range containing a prefix sum over the wait times of the vehicle with given ID at its stops. A vehicle
        // wait time means any amount of time that a vehicle has to wait for a passenger to arrive at a stop (excluding
        // the minimum duration stopTime of each stop).
        ConstantVectorRange<int> vehWaitTimesPrefixSumFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {vehWaitTimesPrefixSum.begin() + start, vehWaitTimesPrefixSum.begin() + end};
        }

        // Range containing the occupancies of each leg of the currently scheduled route of the vehicle with given ID.
        // occupanciesFor(vehId)[i] means the number of passengers that travel in the vehicle between stops i and i+1.
        ConstantVectorRange<int> occupanciesFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {occupancies.begin() + start, occupancies.begin() + end};
        }

        // Range containing a prefix sum over the number of dropoffs that the vehicle with given ID is scheduled to
        // make at each of its stops.
        ConstantVectorRange<int> numDropoffsPrefixSumFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {numDropoffsPrefixSum.begin() + start, numDropoffsPrefixSum.begin() + end};
        }

        // For any vehicle and its i-th stop, this value is the sum of the prefix sums of vehicle wait times up to dropoff
        // d for each dropoff d before the i-th stop.
        // Let N_d(l) be the number of dropoffs at stop l.
        // Then vehWaitTimesUntilDropoffsPrefixSumsFor(vehId)[i] = \sum_{z = 0}^{i} N_d(z) * vehWaitTimesPrefixSumFor(vehId)[z - 1]
        ConstantVectorRange<int> vehWaitTimesUntilDropoffsPrefixSumsFor(const int vehId) const {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            return {vehWaitTimesUntilDropoffsPrefixSum.begin() + start,
                    vehWaitTimesUntilDropoffsPrefixSum.begin() + end};
        }

        // Returns the id of the vehicle whose route the stop with the given ID is currently part of.
        int vehicleIdOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToVehicleId[stopId];
        }

        // Returns the id of the stop that comes before the stop with the given ID in the route of its vehicle.
        int idOfPreviousStopOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToIdOfPrevStop.size());
            return stopIdToIdOfPrevStop[stopId];
        }

        int stopPositionOf(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToPosition.size());
            return stopIdToPosition[stopId];
        }

        int leewayOfLegStartingAt(const int stopId) const {
            assert(stopId >= 0 && stopId < stopIdToLeeway.size());
            return stopIdToLeeway[stopId];
        }

        const int &getMaxLeeway() const {
            return maxLeeway;
        }

        const int &getMaxLegLength() const {
            return maxLegLength;
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insert(const Assignment &asgn, const RequestStateT &requestState) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &pickup = *asgn.pickup;
            const auto &dropoff = *asgn.dropoff;
            const int now = requestState.originalRequest.requestTime;
            const int numRiders = requestState.originalRequest.numRiders;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto pickupIndex = asgn.pickupStopIdx;
            auto dropoffIndex = asgn.dropoffStopIdx;

            assert(pickupIndex >= 0);
            assert(pickupIndex < end - start);
            assert(dropoffIndex >= 0);
            assert(dropoffIndex < end - start);

            bool pickupInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;

            if ((pickupIndex > 0 || schedDepTimes[start] > now) && pickup.loc == stopLocations[start + pickupIndex]) {
                assert(start + pickupIndex == end - 1 || pickupIndex == dropoffIndex ||
                       asgn.distFromPickup ==
                       schedArrTimes[start + pickupIndex + 1] - schedDepTimes[start + pickupIndex]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + pickupIndex] = std::max(schedDepTimes[start + pickupIndex],
                                                              requestState.getPassengerArrAtPickup(pickup.id));

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                
                // Original solution
                // const int psgMaxDepTime = std::max(requestState.getMaxDepTimeAtPickup(), requestState.getPassengerArrAtPickup(pickup.id)); // TODO We restrict the karri dispatcher to enable compareability
                // maxArrTimes[start + pickupIndex] = std::min(maxArrTimes[start + pickupIndex], psgMaxDepTime - InputConfig::getInstance().stopTime); // TODO Just for test reasons
                
                // Test solution
                const int psgMaxDepTime = schedDepTimes[start + pickupIndex]; // TODO Not optimal!!
                maxArrTimes[start + pickupIndex] = std::min(maxArrTimes[start + pickupIndex], psgMaxDepTime - InputConfig::getInstance().stopTime); // TODO Just for test reasons 
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.originalRequest.requestTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++pickupIndex;
                ++dropoffIndex;
                stableInsertion(vehId, pickupIndex, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + pickupIndex] = pickup.loc;
                schedArrTimes[start + pickupIndex] = schedDepTimes[start + pickupIndex - 1] + asgn.distToPickup;
                schedDepTimes[start + pickupIndex] = std::max(schedArrTimes[start + pickupIndex] + InputConfig::getInstance().stopTime,
                                                              requestState.getPassengerArrAtPickup(pickup.id));
                
                
                // Original solution
                // maxArrTimes[start + pickupIndex] = requestState.getMaxDepTimeAtPickup() - InputConfig::getInstance().stopTime;
                
                maxArrTimes[start + pickupIndex] = schedDepTimes[start + pickupIndex] - InputConfig::getInstance().stopTime; // TODO Not optimal!!!

                occupancies[start + pickupIndex] = occupancies[start + pickupIndex - 1];
                numDropoffsPrefixSum[start + pickupIndex] = numDropoffsPrefixSum[start + pickupIndex - 1];
                pickupInsertedAsNewStop = true;
            }

            if (pickupIndex != dropoffIndex) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                assert(asgn.distFromPickup > 0);
                propagateSchedArrAndDepForward(start + pickupIndex + 1, start + dropoffIndex, asgn.distFromPickup);
            }

            if (pickup.loc != dropoff.loc && dropoff.loc == stopLocations[start + dropoffIndex]) {
                maxArrTimes[start + dropoffIndex] = std::min(maxArrTimes[start + dropoffIndex],
                                                             requestState.getMaxArrTimeAtDropoff(pickup.id,
                                                                                                 dropoff.id));
            } else {
                ++dropoffIndex;
                stableInsertion(vehId, dropoffIndex, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + dropoffIndex] = dropoff.loc;
                schedArrTimes[start + dropoffIndex] =
                        schedDepTimes[start + dropoffIndex - 1] + asgn.distToDropoff;
                schedDepTimes[start + dropoffIndex] = schedArrTimes[start + dropoffIndex] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + dropoffIndex] = requestState.getMaxArrTimeAtDropoff(pickup.id, dropoff.id);
                occupancies[start + dropoffIndex] = occupancies[start + dropoffIndex - 1];
                numDropoffsPrefixSum[start + dropoffIndex] = numDropoffsPrefixSum[start + dropoffIndex - 1];
                dropoffInsertedAsNewStop = true;
            }

            if (start + dropoffIndex < end - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                propagateSchedArrAndDepForward(start + dropoffIndex + 1, end - 1, asgn.distFromDropoff);

                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                propagateMaxArrTimeBackward(start + dropoffIndex, start + pickupIndex);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                propagateMaxArrTimeBackward(start + dropoffIndex - 1, start + pickupIndex);
            }
            propagateMaxArrTimeBackward(start + pickupIndex - 1, start);

            // Update occupancies and prefix sums
            for (int idx = start + pickupIndex; idx < start + dropoffIndex; ++idx) {
                occupancies[idx] += numRiders;
                assert(occupancies[idx] <= asgn.vehicle->capacity);
            }

            for (int idx = start + dropoffIndex; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            const int lastUnchangedPrefixSum = pickupIndex > 0 ? vehWaitTimesPrefixSum[start + pickupIndex - 1] : 0;
            recalculateVehWaitTimesPrefixSum(start + pickupIndex, end - 1, lastUnchangedPrefixSum);
            const int lastUnchangedAtDropoffsPrefixSum =
                    pickupIndex > 0 ? vehWaitTimesUntilDropoffsPrefixSum[start + pickupIndex - 1] : 0;
            recalculateVehWaitTimesAtDropoffsPrefixSum(start + pickupIndex, end - 1, lastUnchangedAtDropoffsPrefixSum);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + pickupIndex], stopIds[start + dropoffIndex]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            assert(start == pos[vehId].start && end == pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                assert(pickupIndex >= 1 && start + pickupIndex < end - 1);
                stopIdToVehicleId[stopIds[start + pickupIndex]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex]] = stopIds[start + pickupIndex - 1];
                stopIdToIdOfPrevStop[stopIds[start + pickupIndex + 1]] = stopIds[start + pickupIndex];
            }
            if (dropoffInsertedAsNewStop) {
                assert(dropoffIndex > pickupIndex && start + dropoffIndex < end);
                stopIdToVehicleId[stopIds[start + dropoffIndex]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + dropoffIndex]] = stopIds[start + dropoffIndex - 1];
                if (start + dropoffIndex != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + dropoffIndex + 1]] = stopIds[start + dropoffIndex];
            }

            if (pickupInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = start + pickupIndex; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            updateLeeways(vehId);
            updateMaxLegLength(vehId, pickupIndex, dropoffIndex);


            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + pickupIndex], requestState.originalRequest.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + dropoffIndex], requestState.originalRequest.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {pickupIndex, dropoffIndex};
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insertPVeh(const AssignmentWithTransfer &asgn, const RequestStateT &requestState) {
            const auto vehId = asgn.pVeh->vehicleId;
            const auto &pickup = *asgn.pickup;
            const auto transfer = asgn.transfer;
            const int now = requestState.originalRequest.requestTime;
            const int numRiders = requestState.originalRequest.numRiders;
            
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto pickupIdx = asgn.pickupIdx;
            auto transferIdx = asgn.transferIdxPVeh;

            assert(pickupIdx >= 0);
            assert(pickupIdx < end - start);
            assert(transferIdx >= 0);
            assert(transferIdx < end - start);

            bool pickupInsertedAsNewStop = false;
            bool transferInsertedAsNewStop = false;

            const bool pickupNotInsertedAsNewStopCond = (pickupIdx > 0 || schedDepTimes[start] > now) && pickup.loc == stopLocations[start + pickupIdx];
            if (pickupNotInsertedAsNewStopCond) {
                assert(start + pickupIdx == end - 1 // Pickup is at the last stop
                    || pickupIdx == transferIdx // Pickup paired
                    || asgn.distFromPickup == schedArrTimes[start + pickupIdx + 1] - schedDepTimes[start + pickupIdx]); // Distance from pickup is the distance to the next stop

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + pickupIdx] = std::max(schedDepTimes[start + pickupIdx], requestState.getPassengerArrAtPickup(pickup.id));
            
                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime = std::max(requestState.getMaxDepTimeAtPickup(), requestState.getPassengerArrAtPickup(pickup.id));
                maxArrTimes[start + pickupIdx] = std::min(maxArrTimes[start + pickupIdx], psgMaxDepTime - InputConfig::getInstance().stopTime);
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.originalRequest.requestTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++pickupIdx;
                ++transferIdx;
                
                stableInsertion(vehId, pickupIdx, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

                stopLocations[start + pickupIdx] = pickup.loc;
                schedArrTimes[start + pickupIdx] = schedDepTimes[start + pickupIdx - 1] + asgn.distToPickup;
                schedDepTimes[start + pickupIdx] = std::max(schedArrTimes[start + pickupIdx] + InputConfig::getInstance().stopTime,
                                                              requestState.getPassengerArrAtPickup(pickup.id));
                maxArrTimes[start + pickupIdx] = requestState.getMaxDepTimeAtPickup() - InputConfig::getInstance().stopTime;
                occupancies[start + pickupIdx] = occupancies[start + pickupIdx - 1];

                numDropoffsPrefixSum[start + pickupIdx] = numDropoffsPrefixSum[start + pickupIdx - 1];
                pickupInsertedAsNewStop = true;
            }

            if (pickupIdx != transferIdx) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                assert(asgn.distFromPickup > 0);
                propagateSchedArrAndDepForward(start + pickupIdx + 1, start + transferIdx, asgn.distFromPickup);
            }

            const int actualStopLocationTransfer = stopLocations[start + transferIdx];
            const bool conditionTransferNotNewStop = pickup.loc != transfer.loc && transfer.loc == actualStopLocationTransfer;
            if (conditionTransferNotNewStop) {
                assert(schedDepTimes[start + transferIdx] > asgn.arrAtTransferPoint);
                maxArrTimes[start + transferIdx] = std::min(maxArrTimes[start + transferIdx], asgn.arrAtTransferPoint); // TODO This is not optimal, In this case we set the maxArrTime so, that the arrival can not be delayed, otherwise we would have to propagate the maxArrTime to the dropoffVehicle
            } else {
                // Insert transfer as new stop
                ++transferIdx;
                stableInsertion(vehId, transferIdx, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + transferIdx] = transfer.loc;
                schedArrTimes[start + transferIdx] = schedDepTimes[start + transferIdx - 1] + asgn.distToTransferPVeh;
                schedDepTimes[start + transferIdx] = schedArrTimes[start + transferIdx] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + transferIdx] = asgn.arrAtTransferPoint; // TODO Not optimal, In this case we set the maxArrTime so, that the arrival can not be delayed, otherwise we would have to propagate the maxArrTime to the dropoffVehicle
                occupancies[start + transferIdx] = occupancies[start + transferIdx - 1];
                numDropoffsPrefixSum[start + transferIdx] = numDropoffsPrefixSum[start + transferIdx - 1];
                transferInsertedAsNewStop = true;
            }

            // Propagate updated scheduled arrival and departure times as well as latest permissible arrival times.
            if (start + transferIdx < end - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                propagateSchedArrAndDepForward(start + transferIdx + 1, end - 1, asgn.distFromTransferPVeh);
 
                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                propagateMaxArrTimeBackward(start + transferIdx, start + pickupIdx);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                propagateMaxArrTimeBackward(start + transferIdx - 1, start + pickupIdx);
            }
            propagateMaxArrTimeBackward(start + pickupIdx - 1, start);

            // Update occupancies and prefix sums
            for (int idx = start + pickupIdx; idx < start + transferIdx; ++idx) {
                occupancies[idx] += numRiders;
                assert(occupancies[idx] <= asgn.pVeh->capacity);
            }

            for (int idx = start + transferIdx; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            const int lastUnchangedPrefixSum = pickupIdx > 0 ? vehWaitTimesPrefixSum[start + pickupIdx - 1] : 0;
            recalculateVehWaitTimesPrefixSum(start + pickupIdx, end - 1, lastUnchangedPrefixSum);
            const int lastUnchangedAtDropoffsPrefixSum =
                    pickupIdx > 0 ? vehWaitTimesUntilDropoffsPrefixSum[start + pickupIdx - 1] : 0;
            recalculateVehWaitTimesAtDropoffsPrefixSum(start + pickupIdx, end - 1, lastUnchangedAtDropoffsPrefixSum);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + pickupIdx], stopIds[start + transferIdx]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            assert(start == pos[vehId].start && end == pos[vehId].end);
            if (pickupInsertedAsNewStop) {
                assert(pickupIdx >= 1 && start + pickupIdx < end - 1);
                stopIdToVehicleId[stopIds[start + pickupIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + pickupIdx]] = stopIds[start + pickupIdx - 1];
                stopIdToIdOfPrevStop[stopIds[start + pickupIdx + 1]] = stopIds[start + pickupIdx];
            }
            if (transferInsertedAsNewStop) {
                assert(transferIdx > pickupIdx && start + transferIdx < end);
                stopIdToVehicleId[stopIds[start + transferIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + transferIdx]] = stopIds[start + transferIdx - 1];
                if (start + transferIdx != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + transferIdx + 1]] = stopIds[start + transferIdx];
            }

            if (pickupInsertedAsNewStop || transferInsertedAsNewStop) {
                for (int i = start + pickupIdx; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            updateLeeways(vehId);
            updateMaxLegLength(vehId, pickupIdx, transferIdx);

            for (int i = start; i < end; i++) {
                assert(stopIdToVehicleId[stopIds[i]] == vehId);
            }
            
            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + pickupIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + transferIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {pickupIdx, transferIdx};
        }

        template<typename RequestStateT>
        std::pair<int, int>
        insertDVeh(const AssignmentWithTransfer &asgn, const RequestStateT &requestState) {
            const auto vehId = asgn.dVeh->vehicleId;
            const auto transfer = asgn.transfer;
            const auto &dropoff = *asgn.dropoff;
            const int now = requestState.originalRequest.requestTime;
            const int numRiders = requestState.originalRequest.numRiders;
            const auto &start = pos[vehId].start;
            const auto &end = pos[vehId].end;
            auto transferIdx = asgn.transferIdxDVeh;
            auto dropoffIdx = asgn.dropoffIdx;

            assert(transferIdx >= 0);
            assert(transferIdx < end - start);
            assert(dropoffIdx >= 0);
            assert(dropoffIdx < end - start);

            bool transferInsertedAsNewStop = false;
            bool dropoffInsertedAsNewStop = false;


            if ((transferIdx > 0 || schedDepTimes[start] > now) && transfer.loc == stopLocations[start + transferIdx]) {
                assert(start + transferIdx == end - 1 || transferIdx == dropoffIdx || asgn.distFromTransferDVeh == schedArrTimes[start + transferIdx + 1] - schedDepTimes[start + transferIdx]);

                // Pickup at existing stop
                // For pickup at existing stop we don't count another stopTime. The vehicle can depart at the earliest
                // moment when vehicle and passenger are at the location.
                schedDepTimes[start + transferIdx] = std::max(schedDepTimes[start + transferIdx],
                                                              asgn.arrAtTransferPoint);

                // If we allow pickupRadius > waitTime, then the passenger may arrive at the pickup location after
                // the regular max dep time of requestTime + waitTime. In this case, the new latest permissible arrival
                // time is defined by the passenger arrival time at the pickup, not the maximum wait time.
                const int psgMaxDepTime = std::max(requestState.getMaxDepTimeAtTransfer(asgn), asgn.arrAtTransferPoint);
                maxArrTimes[start + transferIdx] = std::min(maxArrTimes[start + transferIdx], psgMaxDepTime - InputConfig::getInstance().stopTime);
            } else {
                // If vehicle is currently idle, the vehicle can leave its current stop at the earliest when the
                // request is made. In that case, we update the arrival time to count the idling as one stopTime.
                schedDepTimes[end - 1] = std::max(schedDepTimes[end - 1], requestState.originalRequest.requestTime);
                schedArrTimes[end - 1] = schedDepTimes[end - 1] - InputConfig::getInstance().stopTime;
                ++transferIdx;
                ++dropoffIdx;
                stableInsertion(vehId, transferIdx, getUnusedStopId(), pos, stopIds, stopLocations,
                                schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                                numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + transferIdx] = transfer.loc;
                schedArrTimes[start + transferIdx] = schedDepTimes[start + transferIdx - 1] + asgn.distToTransferDVeh;
                schedDepTimes[start + transferIdx] = std::max(schedArrTimes[start + transferIdx] + InputConfig::getInstance().stopTime,
                                                              asgn.arrAtTransferPoint);
                maxArrTimes[start + transferIdx] = requestState.getMaxDepTimeAtTransfer(asgn) - InputConfig::getInstance().stopTime;
                occupancies[start + transferIdx] = occupancies[start + transferIdx - 1];
                numDropoffsPrefixSum[start + transferIdx] = numDropoffsPrefixSum[start + transferIdx - 1];
                transferInsertedAsNewStop = true;
            }

            if (transferIdx != dropoffIdx) {
                // Propagate changes to minArrTime/minDepTime forward from inserted pickup stop until dropoff stop
                assert(asgn.distFromTransferDVeh > 0);
                propagateSchedArrAndDepForward(start + transferIdx + 1, start + dropoffIdx, asgn.distFromTransferDVeh);
            }


            if (transfer.loc != dropoff.loc && dropoff.loc == stopLocations[start + dropoffIdx]) {
                maxArrTimes[start + dropoffIdx] = std::min(maxArrTimes[start + dropoffIdx],
                                                             requestState.getMaxArrTimeAtDropoff(asgn.pickup->id,
                                                                                                 dropoff.id));
            } else {
                ++dropoffIdx;
                stableInsertion(vehId, dropoffIdx, getUnusedStopId(),
                                pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                                maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
                stopLocations[start + dropoffIdx] = dropoff.loc;
                schedArrTimes[start + dropoffIdx] =
                        schedDepTimes[start + dropoffIdx - 1] + asgn.distToDropoff;
                schedDepTimes[start + dropoffIdx] = schedArrTimes[start + dropoffIdx] + InputConfig::getInstance().stopTime;
                // compare maxVehArrTime to next stop later
                maxArrTimes[start + dropoffIdx] = requestState.getMaxArrTimeAtDropoff(asgn.pickup->id, dropoff.id);
                occupancies[start + dropoffIdx] = occupancies[start + dropoffIdx - 1];
                numDropoffsPrefixSum[start + dropoffIdx] = numDropoffsPrefixSum[start + dropoffIdx - 1];
                dropoffInsertedAsNewStop = true;
            }

            // Propagate updated scheduled arrival and departure times as well as latest permissible arrival times.
            if (start + dropoffIdx < end - 1) {
                // At this point minDepTimes[start + dropoffIndex] is correct. If dropoff has been inserted not as the last
                // stop, propagate the changes to minDep and minArr times forward until the last stop.
                propagateSchedArrAndDepForward(start + dropoffIdx + 1, end - 1, asgn.distFromDropoff);

                // If there are stops after the dropoff, consider them for propagating changes to the maxArrTimes
                propagateMaxArrTimeBackward(start + dropoffIdx, start + transferIdx);
            } else {
                // If there are no stops after the dropoff, propagate maxArrTimes backwards not including dropoff
                propagateMaxArrTimeBackward(start + dropoffIdx - 1, start + transferIdx);
            }
            propagateMaxArrTimeBackward(start + transferIdx - 1, start);

            // Update occupancies and prefix sums
            for (int idx = start + transferIdx; idx < start + dropoffIdx; ++idx) {
                occupancies[idx] += numRiders;
                assert(occupancies[idx] <= asgn.dVeh->capacity);
            }

            for (int idx = start + dropoffIdx; idx < end; ++idx) {
                ++numDropoffsPrefixSum[idx];
            }

            const int lastUnchangedPrefixSum = transferIdx > 0 ? vehWaitTimesPrefixSum[start + transferIdx - 1] : 0;
            recalculateVehWaitTimesPrefixSum(start + transferIdx, end - 1, lastUnchangedPrefixSum);
            const int lastUnchangedAtDropoffsPrefixSum =
                    transferIdx > 0 ? vehWaitTimesUntilDropoffsPrefixSum[start + transferIdx - 1] : 0;
            recalculateVehWaitTimesAtDropoffsPrefixSum(start + transferIdx, end - 1, lastUnchangedAtDropoffsPrefixSum);

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const auto newMinSize = std::max(stopIds[start + transferIdx], stopIds[start + dropoffIdx]) + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            assert(start == pos[vehId].start && end == pos[vehId].end);
            if (transferInsertedAsNewStop) {
                assert(transferIdx >= 1 && start + transferIdx < end - 1);
                stopIdToVehicleId[stopIds[start + transferIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + transferIdx]] = stopIds[start + transferIdx - 1];
                stopIdToIdOfPrevStop[stopIds[start + transferIdx + 1]] = stopIds[start + transferIdx];
            }
            if (dropoffInsertedAsNewStop) {
                assert(dropoffIdx > transferIdx && start + dropoffIdx < end);
                stopIdToVehicleId[stopIds[start + dropoffIdx]] = vehId;
                stopIdToIdOfPrevStop[stopIds[start + dropoffIdx]] = stopIds[start + dropoffIdx - 1];
                if (start + dropoffIdx != end - 1)
                    stopIdToIdOfPrevStop[stopIds[start + dropoffIdx + 1]] = stopIds[start + dropoffIdx];
            }

            if (transferInsertedAsNewStop || dropoffInsertedAsNewStop) {
                for (int i = start + transferIdx; i < end; ++i) {
                    stopIdToPosition[stopIds[i]] = i - start;
                }
            }

            updateLeeways(vehId);
            updateMaxLegLength(vehId, transferIdx, dropoffIdx);

            for (int i = start; i < end; i++) {
                assert(stopIdToVehicleId[stopIds[i]] == vehId);
            }


            // Remember that request is picked up and dropped of at respective stops:
            insertion(stopIds[start + transferIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            insertion(stopIds[start + dropoffIdx], requestState.originalRequest.requestId,
                      rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);

            return {transferIdx, dropoffIdx};
        }

        void removeStartOfCurrentLeg(const int vehId) {
            assert(vehId >= 0);
            assert(vehId < pos.size());
            const auto &start = pos[vehId].start;
            assert(pos[vehId].end - start > 0);
            const bool haveToRecomputeMaxLeeway = stopIds[start] == stopIdOfMaxLeeway;
            stopIdToVehicleId[stopIds[start]] = INVALID_ID;
            stopIdToLeeway[stopIds[start]] = 0;
            stopIdToPosition[stopIds[start]] = INVALID_INDEX;
            removalOfAllCols(stopIds[start], rangeOfRequestsPickedUpAtStop, requestsPickedUpAtStop);
            removalOfAllCols(stopIds[start], rangeOfRequestsDroppedOffAtStop, requestsDroppedOffAtStop);
            unusedStopIds.push(stopIds[start]);
            assert(stopIdToIdOfPrevStop[stopIds[start]] == INVALID_ID);
            if (numStopsOf(vehId) > 1) {
                stopIdToIdOfPrevStop[stopIds[start + 1]] = INVALID_ID;
            }

            const auto numDropoffsAtStart = numDropoffsPrefixSum[start];
            stableRemoval(vehId, 0,
                          pos, stopIds, stopLocations, schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum,
                          maxArrTimes, occupancies, numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);

            const auto &startAfterRemoval = pos[vehId].start;
            const auto &endAfterRemoval = pos[vehId].end;
            for (int i = startAfterRemoval; i < endAfterRemoval; ++i) {
                numDropoffsPrefixSum[i] -= numDropoffsAtStart;
                --stopIdToPosition[stopIds[i]];
                assert(stopIdToPosition[stopIds[i]] == i - startAfterRemoval);
            }

            if (haveToRecomputeMaxLeeway)
                recomputeMaxLeeway();
        }

        // Creates an intermediate stop between stop 0 and stop 1 for a vehicle reroute at the given location.
        void createIntermediateStopForReroute(const int vehId, const int location, const int now, const int depTime) {
            KASSERT(vehId >= 0);
            KASSERT(vehId < pos.size());
            KASSERT(pos[vehId].end - pos[vehId].start > 0);
            KASSERT(depTime >= now);
            stableInsertion(vehId, 1, getUnusedStopId(), pos, stopIds, stopLocations,
                            schedArrTimes, schedDepTimes, vehWaitTimesPrefixSum, maxArrTimes, occupancies,
                            numDropoffsPrefixSum, vehWaitTimesUntilDropoffsPrefixSum);
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;
            stopLocations[start + 1] = location;
            schedArrTimes[start + 1] = now;
            schedDepTimes[start + 1] = depTime;
            maxArrTimes[start + 1] = now;
            occupancies[start + 1] = occupancies[start];
            numDropoffsPrefixSum[start + 1] = numDropoffsPrefixSum[start];
            vehWaitTimesPrefixSum[start + 1] = vehWaitTimesPrefixSum[start];
            vehWaitTimesUntilDropoffsPrefixSum[start + 1] = vehWaitTimesUntilDropoffsPrefixSum[start];

            // Update mappings from the stop ids to ids of previous stop, to position in the route, to the leeway and
            // to the vehicle id.
            const int newStopId = stopIds[start + 1];
            const auto newMinSize = newStopId + 1;
            if (stopIdToIdOfPrevStop.size() < newMinSize) {
                stopIdToIdOfPrevStop.resize(newMinSize, INVALID_ID);
                stopIdToPosition.resize(newMinSize, INVALID_INDEX);
                stopIdToLeeway.resize(newMinSize, 0);
                stopIdToVehicleId.resize(newMinSize, INVALID_ID);
                rangeOfRequestsPickedUpAtStop.resize(newMinSize);
                rangeOfRequestsDroppedOffAtStop.resize(newMinSize);
            }
            KASSERT(start == pos[vehId].start && end == pos[vehId].end);
            stopIdToVehicleId[newStopId] = vehId;
            stopIdToIdOfPrevStop[newStopId] = stopIds[start];
            stopIdToIdOfPrevStop[stopIds[start + 2]] = newStopId;
            for (int i = start; i < end; ++i) {
                stopIdToPosition[stopIds[i]] = i - start;
            }

            const auto leeway =
                    std::max(maxArrTimes[start + 2], schedDepTimes[start + 2]) - schedDepTimes[start + 1] - InputConfig::getInstance().stopTime;
            assert(leeway >= 0);
            stopIdToLeeway[newStopId] = leeway;

            updateMaxLegLength(vehId, 1, 1);
        }

        //* Utility methods to test if the insertion was correct
        bool assertRoutePVeh(const AssignmentWithTransfer &asgn) {
            const auto numStopsPVeh = numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = stopLocationsFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesPVeh = schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimesPVeh = schedArrTimesFor(asgn.pVeh->vehicleId);
            
            // Find the pickup and transfer indices
            int pickupIdx = asgn.pickupIdx;
            const bool pickupBNS = asgn.pickupIdx == 0;
            while (stopLocationsPVeh[pickupIdx] != asgn.pickup->loc || schedDepTimesPVeh[pickupIdx] <= asgn.requestTime) {
                pickupIdx++;
            }

            assert(stopLocationsPVeh[pickupIdx] != asgn.pickup->loc || pickupIdx > 0 || schedDepTimesPVeh[pickupIdx] >= asgn.requestTime);
            if(!(stopLocationsPVeh[pickupIdx] != asgn.pickup->loc || pickupIdx > 0 || schedDepTimesPVeh[pickupIdx] >= asgn.requestTime))
                return false;

            const bool pickupAsNewStop = asgn.pickupIdx != pickupIdx;
            const int pickupLaterShifted = pickupIdx - asgn.pickupIdx;
            int transferIdxPVeh = std::max(pickupIdx + 1, asgn.transferIdxPVeh + pickupLaterShifted);
            while (stopLocationsPVeh[transferIdxPVeh] != asgn.transfer.loc) {
                ++transferIdxPVeh;
            }
            
            const int correctedTransferIdx = asgn.transferIdxPVeh + pickupLaterShifted;
            const bool transferAsNewStop = correctedTransferIdx != transferIdxPVeh;

            assert(pickupBNS || pickupIdx == asgn.pickupIdx + pickupAsNewStop);
            if (!(pickupBNS || pickupIdx == asgn.pickupIdx + pickupAsNewStop))
               
            assert(correctedTransferIdx + transferAsNewStop == transferIdxPVeh);
            if (!(correctedTransferIdx + transferAsNewStop == transferIdxPVeh))
                return false;

            // Assert that the departure at the pickup is later than the arrival at the pickup
            assert(schedDepTimesPVeh[pickupIdx] >= asgn.requestTime + asgn.pickup->walkingDist);
            if (!(schedDepTimesPVeh[pickupIdx] >= asgn.requestTime + asgn.pickup->walkingDist))
                return false;

            const int schedDepAtPickup = schedDepTimesPVeh[pickupIdx];
            assert(schedDepAtPickup == asgn.depAtPickup);
            if (!(schedDepAtPickup == asgn.depAtPickup))
                return false;

            // Assert that it is recognized correctly, when a transfer is not a new stop
            assert(asgn.transferAtStopPVeh == !transferAsNewStop);
            if (!(asgn.transferAtStopPVeh == !transferAsNewStop))
                return false;
            

            if (pickupAsNewStop && !assertPickupNew(asgn, schedDepTimesPVeh, schedArrTimesPVeh, pickupIdx, transferIdxPVeh))
                return false;

            if (transferAsNewStop && !assertTransferNewPVeh(asgn, schedDepTimesPVeh, schedArrTimesPVeh, transferIdxPVeh, numStopsPVeh))
                return false;

            // Assert the the arrival at the transfer point (dropoff for passenger) is correct
            assert(transferIdxPVeh == 0 || !transferAsNewStop || schedDepTimesPVeh[transferIdxPVeh - 1] + asgn.distToTransferPVeh == asgn.arrAtTransferPoint);
            if (transferIdxPVeh > 0 && transferAsNewStop && !(schedDepTimesPVeh[transferIdxPVeh - 1] + asgn.distToTransferPVeh == asgn.arrAtTransferPoint)) { 
                return false;
            }

            if (!transferAsNewStop && !(schedArrTimesPVeh[transferIdxPVeh] == asgn.arrAtTransferPoint))
                return false;

            if (!(schedArrTimesPVeh[transferIdxPVeh] == asgn.arrAtTransferPoint)
             || !(schedArrTimesPVeh[pickupIdx] + InputConfig::getInstance().stopTime <= schedDepTimesPVeh[pickupIdx])
             || !(schedArrTimesPVeh[transferIdxPVeh] + InputConfig::getInstance().stopTime <= schedDepTimesPVeh[transferIdxPVeh]))
                return false;
            
            return true;
        }

        bool assertRouteDVeh(const AssignmentWithTransfer &asgn) {
            const auto numStopsDVeh = numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsDVeh = stopLocationsFor(asgn.dVeh->vehicleId);
            const auto schedDepTimesDVeh = schedDepTimesFor(asgn.dVeh->vehicleId);
            const auto schedArrTimesDVeh = schedArrTimesFor(asgn.dVeh->vehicleId);
            // const auto maxArrTimesDVeh = maxArrTimesFor(asgn.dVeh->vehicleId);
            
            // Find the pickup and transfer indices
            int transferIdxDVeh = asgn.transferIdxDVeh;
            const bool transferBNS = asgn.transferIdxDVeh == 0;
            while (stopLocationsDVeh[transferIdxDVeh] != asgn.transfer.loc || schedDepTimesDVeh[transferIdxDVeh] <= asgn.requestTime) {
                ++transferIdxDVeh;
            }

            const int transferLaterShifted = transferIdxDVeh - asgn.transferIdxDVeh;
            int dropoffIdx = std::max(transferIdxDVeh + 1, asgn.dropoffIdx + transferLaterShifted);
            while (stopLocationsDVeh[dropoffIdx] != asgn.dropoff->loc) {
                ++dropoffIdx;
            }

            const bool transferAsNewStop = asgn.transferIdxDVeh != transferIdxDVeh;
            const int correctedDropoffIdx = asgn.dropoffIdx + transferLaterShifted;
            const bool dropoffAsNewStop = correctedDropoffIdx != dropoffIdx;

            const int schedDepAtTransfer = schedDepTimesDVeh[transferIdxDVeh];
             // Assert that the arrival at the dropoff is corrent
            const int schedArrAtDropoff = schedArrTimesDVeh[dropoffIdx];

            if (!(transferBNS || transferIdxDVeh == asgn.transferIdxDVeh + transferAsNewStop)
             || !(correctedDropoffIdx + dropoffAsNewStop == dropoffIdx)
             // Assert that the scheduled departure at the transfer is later than the arrival at the transfer
             || !(schedDepAtTransfer >= asgn.arrAtTransferPoint && schedDepAtTransfer == asgn.depAtTransfer)
             || !(schedArrAtDropoff == asgn.arrAtDropoff))
                return false;

            // Assert that the trip time of the dVeh is corret
            const int waitingTimeAtTransfer = schedDepTimesDVeh[transferIdxDVeh] - asgn.arrAtTransferPoint;
            const int actualTripTime = schedArrTimesDVeh[dropoffIdx] - schedDepTimesDVeh[transferIdxDVeh] + asgn.dropoff->walkingDist + waitingTimeAtTransfer;
            
            
            if (!(actualTripTime == asgn.tripTimeDVeh))
                return false;

            if (transferAsNewStop && !assertTransferNewDVeh(asgn, schedDepTimesDVeh, schedArrTimesDVeh, transferIdxDVeh, dropoffIdx))
                return false;

            if (dropoffAsNewStop && !assertDropoffNew(asgn, schedDepTimesDVeh, schedArrTimesDVeh, dropoffIdx, numStopsDVeh))
                return false;

            if (!(schedArrTimesDVeh[transferIdxDVeh] + InputConfig::getInstance().stopTime <= schedDepTimesDVeh[transferIdxDVeh])
             || !(schedArrTimesDVeh[dropoffIdx] + InputConfig::getInstance().stopTime <= schedDepTimesDVeh[dropoffIdx]))
                return false;

            return true;
        }

        bool assertPickupNew(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesPVeh, ConstantVectorRange<int> schedArrTimesPVeh, const int pickupIdx, const int transferIdxPVeh) {
            const bool bns = asgn.pickupIdx == 0;
            const bool paired = asgn.pickupIdx == asgn.transferIdxPVeh;

            if (!(!paired || pickupIdx + 1 == transferIdxPVeh))
                return false;

            if (!bns && !(pickupIdx > 1 || schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[pickupIdx - 1] == asgn.distToPickup)) {
                // Pickup is not first stop
                return false;
            } else if (!(pickupIdx >= 0 && pickupIdx <= 2) || !(schedArrTimesPVeh[pickupIdx] - schedDepTimesPVeh[0] == asgn.distToPickup)) {
                return false;
            }

            if (!paired && !(schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] == asgn.distFromPickup)) {
                return false;
            }

            return true;
        }

        bool assertTransferNewPVeh(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesPVeh, ConstantVectorRange<int> schedArrTimesPVeh, const int transferIdxPVeh, const int numStopsPVeh) {
            if (!(transferIdxPVeh > 0)
             || !(schedArrTimesPVeh[transferIdxPVeh] - schedDepTimesPVeh[transferIdxPVeh - 1] == asgn.distToTransferPVeh))
                return false;
            
            // If the transfer is not als, assert the distance to the next stop
            if (transferIdxPVeh < numStopsPVeh - 1
            && !(schedArrTimesPVeh[transferIdxPVeh + 1] - schedDepTimesPVeh[transferIdxPVeh] == asgn.distFromTransferPVeh))
                return false;

            return true;
        }

        bool assertTransferNewDVeh(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesDVeh, ConstantVectorRange<int> schedArrTimesDVeh, const int transferIdxDVeh, const int dropoffIdx) {
            const bool bns = asgn.transferIdxDVeh == 0;
            const bool paired = asgn.transferIdxDVeh == asgn.dropoffIdx;
            
            if (!(!paired || transferIdxDVeh + 1 == dropoffIdx))
                return false;
            
            if (!bns && (!(transferIdxDVeh > 1) || !(schedArrTimesDVeh[transferIdxDVeh] - schedDepTimesDVeh[transferIdxDVeh - 1] == asgn.distToTransferDVeh)))
                return false;

            if (bns && (!(transferIdxDVeh >= 0 && transferIdxDVeh <= 2) || !(schedArrTimesDVeh[transferIdxDVeh] - schedDepTimesDVeh[0] == asgn.distToTransferDVeh)))
                return false;

            if (!paired && !(schedArrTimesDVeh[transferIdxDVeh + 1] - schedDepTimesDVeh[transferIdxDVeh] == asgn.distFromTransferDVeh))
                return false;

            return true;
        }

        bool assertDropoffNew(const AssignmentWithTransfer &asgn, ConstantVectorRange<int> schedDepTimesDVeh, ConstantVectorRange<int> schedArrTimesDVeh, const int dropoffIdx, const int numStopsDVeh) {
            if (!(dropoffIdx > 0)
             // Assert the distance to the dropoff
             || !(schedArrTimesDVeh[dropoffIdx] - schedDepTimesDVeh[dropoffIdx - 1] == asgn.distToDropoff))
                return false;

            // If the dropoff is not als, assert the distance to the next stop
            if (dropoffIdx < numStopsDVeh - 1 && !(schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx] == asgn.distFromDropoff)) {
                // Dropoff is not the last stop
                // Assert the distance from the dropoff
                return false;
            }

            return true;
        }

        // Scheduled stop interface for event simulation
        struct ScheduledStop {
            int stopId;
            int arrTime;
            int depTime;
            int occupancyInFollowingLeg;
            ConstantVectorRange<int> requestsPickedUpHere;
            ConstantVectorRange<int> requestsDroppedOffHere;
        };

        bool hasNextScheduledStop(const int vehId) const {
            return numStopsOf(vehId) > 1;
        }

        ScheduledStop getNextScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 1);
        }

        ScheduledStop getCurrentOrPrevScheduledStop(const int vehId) const {
            return getScheduledStop(vehId, 0);
        }

    private:

        ScheduledStop getScheduledStop(const int vehId, const int stopIndex) const {
            assert(numStopsOf(vehId) > stopIndex);
            const auto id = stopIdsFor(vehId)[stopIndex];
            const auto arrTime = schedArrTimesFor(vehId)[stopIndex];
            const auto depTime = schedDepTimesFor(vehId)[stopIndex];
            const auto occ = occupanciesFor(vehId)[stopIndex];
            const auto pickupsRange = rangeOfRequestsPickedUpAtStop[id];
            const ConstantVectorRange<int> pickups = {requestsPickedUpAtStop.begin() + pickupsRange.start,
                                                      requestsPickedUpAtStop.begin() + pickupsRange.end};
            const auto dropoffsRange = rangeOfRequestsDroppedOffAtStop[id];
            const ConstantVectorRange<int> dropoffs = {requestsDroppedOffAtStop.begin() + dropoffsRange.start,
                                                       requestsDroppedOffAtStop.begin() + dropoffsRange.end};
            return {id, arrTime, depTime, occ, pickups, dropoffs};
        }

        int getUnusedStopId() {
            if (!unusedStopIds.empty()) {
                const auto id = unusedStopIds.top();
                unusedStopIds.pop();
                assert(stopIdToVehicleId[id] == INVALID_ID);
                return id;
            }
            ++maxStopId;
            return nextUnusedStopId++;
        }


        // Standard forward propagation of changes to minVehArrTime and minVehDepTime from fromIdx to toIdx (both inclusive)
        // caused by inserting a pickup stop. Needs distance from stop at fromIdx - 1 to stop at fromIdx because that
        // distance cannot be inferred. Indices are direct indices in the 2D arrays.
        void propagateSchedArrAndDepForward(const int fromIdx, const int toIdx, const int distFromPrevOfFromIdx) {
            assert(distFromPrevOfFromIdx > 0);
            int distPrevToCurrent = distFromPrevOfFromIdx;
            for (int l = fromIdx; l <= toIdx; ++l) {
                schedArrTimes[l] = schedDepTimes[l - 1] + distPrevToCurrent;

                // If the planned departure time is already later than the new arrival time demands, then the planned
                // departure time remains unaffected and subsequent arrival/departure times will not change either.
                if (schedDepTimes[l] >= schedArrTimes[l] + InputConfig::getInstance().stopTime) {
                    break;
                }

                const auto oldMinDepTime = schedDepTimes[l];
                schedDepTimes[l] = schedArrTimes[l] + InputConfig::getInstance().stopTime; // = max(schedDepTimes[l], schedArrTimes[l] + stopTime);
                if (l < toIdx) distPrevToCurrent = schedArrTimes[l + 1] - oldMinDepTime;
            }
        }

        void propagateSchedArrAndDepForward(const int fromIdx, const int toIdx) {
            int distPrevToCurrent = 0;
            
            for (int l = fromIdx; l <= toIdx; ++l) {
                schedArrTimes[l] = schedDepTimes[l - 1] + distPrevToCurrent;

                // If the planned departure time is already later than the new arrival time demands, then the planned
                // departure time remains unaffected and subsequent arrival/departure times will not change either.
                if (schedDepTimes[l] >= schedArrTimes[l] + InputConfig::getInstance().stopTime) {
                    break;
                }

                const auto oldMinDepTime = schedDepTimes[l];
                schedDepTimes[l] = schedArrTimes[l] + InputConfig::getInstance().stopTime; // = max(schedDepTimes[l], schedArrTimes[l] + stopTime);
                if (l < toIdx) distPrevToCurrent = schedArrTimes[l + 1] - oldMinDepTime;
            }
        }

        // Backwards propagation of changes to maxArrTimes from fromIdx down to toIdx
        void propagateMaxArrTimeBackward(const int fromIdx, const int toIdx) {
            for (int l = fromIdx; l >= toIdx; --l) {
                const auto distToNext = schedArrTimes[l + 1] - schedDepTimes[l];
                const auto propagatedMaxArrTime = maxArrTimes[l + 1] - distToNext - InputConfig::getInstance().stopTime;
                if (maxArrTimes[l] <= propagatedMaxArrTime)
                    break; // Stop propagating if known maxArrTime at l is stricter already
                maxArrTimes[l] = propagatedMaxArrTime;
            }
        }

        void updateLeeways(const int vehId) {
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;

            for (int idx = start; idx < end - 1; ++idx) {
                const auto &stopId = stopIds[idx];

                // Set leeway of stop, possibly update max leeway
                const auto leeway =
                        std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1]) - schedDepTimes[idx] - InputConfig::getInstance().stopTime;
                assert(leeway >= 0);
                stopIdToLeeway[stopId] = leeway;

                if (leeway > maxLeeway) {
                    maxLeeway = leeway;
                    stopIdOfMaxLeeway = stopId;
                }
            }

            if (stopIdToLeeway[stopIdOfMaxLeeway] < maxLeeway) {
                // Leeway of stop that previously had the max leeway has decreased s.t. it is no longer the stop with
                // the largest leeway, so we recompute the largest leeway from scratch.
                recomputeMaxLeeway();
            }
        }

        void updateMaxLegLength(const int vehId, const int pickupIndex, const int dropoffIndex) {
            const auto start = pos[vehId].start;
            const auto end = pos[vehId].end;

            bool lengthOfFormerLongestChanged = false;
            bool maxLengthIncreased = false;

            auto updateLegLengthAt = [&](const int stopIdx) {
                const auto toPickup = schedArrTimes[start + stopIdx + 1] - schedDepTimes[start + stopIdx];
                lengthOfFormerLongestChanged |= stopIds[start + stopIdx] == stopIdOfMaxLegLength;
                if (toPickup >= maxLegLength) {
                    maxLegLength = toPickup;
                    stopIdOfMaxLegLength = stopIds[start + stopIdx];
                    maxLengthIncreased = true;
                }
            };

            if (pickupIndex > 0)
                updateLegLengthAt(pickupIndex - 1);
            if (pickupIndex + 1 != dropoffIndex)
                updateLegLengthAt(pickupIndex);
            updateLegLengthAt(dropoffIndex - 1);
            if (start + dropoffIndex < end - 1)
                updateLegLengthAt(dropoffIndex);

            // If we did not find a new maximum leg length but the length of the formerly longest leg changed, we need
            // to recompute the max leg length from scratch.
            if (!maxLengthIncreased && lengthOfFormerLongestChanged)
                recomputeMaxLegLength();
        }

        // Recalculate the prefix sum of vehicle wait times from fromIdx up to toIdx (both inclusive) based on current
        // minVehArrTime and minVehDepTime values. Takes a sum for the element before fromIdx as baseline.
        void recalculateVehWaitTimesPrefixSum(const int fromIdx, const int toIdx, const int baseline) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto stopLength = schedDepTimes[l] - InputConfig::getInstance().stopTime - schedArrTimes[l];
                assert(stopLength >= 0);
                vehWaitTimesPrefixSum[l] = prevSum + stopLength;
                prevSum = vehWaitTimesPrefixSum[l];
            }
        }

        void recalculateVehWaitTimesAtDropoffsPrefixSum(const int fromIdx, const int toIdx, const int baseline) {
            int prevSum = baseline;
            for (int l = fromIdx; l <= toIdx; ++l) {
                const auto numDropoffs = numDropoffsPrefixSum[l] - (l == 0 ? 0 : numDropoffsPrefixSum[l - 1]);
                const auto waitPrefixSum = l == 0 ? 0 : vehWaitTimesPrefixSum[l - 1];
                vehWaitTimesUntilDropoffsPrefixSum[l] = prevSum + numDropoffs * waitPrefixSum;
                prevSum = vehWaitTimesUntilDropoffsPrefixSum[l];
            }
        }

        void recomputeMaxLeeway() {
            maxLeeway = 0;
            stopIdOfMaxLeeway = INVALID_ID;
            for (const auto &[start, end]: pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto leeway = std::max(maxArrTimes[idx + 1], schedDepTimes[idx + 1])
                                        - schedDepTimes[idx] - InputConfig::getInstance().stopTime;
                    if (leeway > maxLeeway) {
                        maxLeeway = leeway;
                        stopIdOfMaxLeeway = stopIds[idx];
                    }
                }
            }
        }

        void recomputeMaxLegLength() {
            maxLegLength = 0;
            stopIdOfMaxLegLength = INVALID_ID;
            for (const auto &[start, end]: pos) {
                for (int idx = start; idx < end - 1; ++idx) {
                    const auto legLength = schedArrTimes[idx + 1] - schedDepTimes[idx];
                    if (legLength > maxLegLength) {
                        maxLegLength = legLength;
                        stopIdOfMaxLegLength = stopIds[idx];
                    }
                }
            }
        }

        void printStopLocations(const int vehId) {
            std::string sep = "";
            for (int i = 0; i < stopLocationsFor(vehId).size(); ++i) {
                std::cout << sep << i << ": " << stopLocationsFor(vehId)[i];
                sep = ", ";
            }
            std::cout << std::endl;
        }

        // Index Array:

        // For a vehicle with ID vehId, the according entries in each value array lie in the index interval
        // [pos[vehId].start, pos[vehId].end).
        std::vector<ValueBlockPosition> pos;

        // Value Arrays:

        // Unique ID for each stop (IDs can be reused after stops are finished, just unique for any point in time)
        std::vector<int> stopIds;

        // Locations of stops (edges in vehicle road network)
        std::vector<int> stopLocations;

        // Scheduled arrival time of vehicle for each stop
        std::vector<int> schedArrTimes;

        // Scheduled departure time of vehicle for each stop
        std::vector<int> schedDepTimes;

        // Latest permissible arrival time of vehicle at each stop in order to adhere to hard constraints of existing
        // passengers.
        std::vector<int> maxArrTimes;

        // Occupancies in route leg immediately following each stop
        std::vector<int> occupancies;

        // Index-shifted prefix sum of vehicle wait times not including the minimum stop length stopTime for any stop.
        // vehWaitTimesPrefixSum[pos[vehId].start + i] is the sum of all wait times up till and including stop i for the
        // vehicle with ID vehId.
        std::vector<int> vehWaitTimesPrefixSum;

        // For any vehicle and its i-th stop, this value is the sum of the prefix sums of vehicle wait times up to dropoff
        // d for each dropoff d before the i-th stop.
        // Let N_d(l) be the number of dropoffs at stop l.
        // Then vehWaitTimesUntilDropoffsPrefixSum[i] = \sum_{z = 0}^{i} N_d(z) * vehWaitTimesPrefixSum[z - 1]
        std::vector<int> vehWaitTimesUntilDropoffsPrefixSum;

        // Prefix sum of the number of dropoffs scheduled for the vehicle up to a stop.
        // numDropoffsPrefixSum[pos[vehId].start + i] is the number of dropoffs before stop i plus the number of dropoffs
        // at stop i for the vehicle with ID vehId.
        std::vector<int> numDropoffsPrefixSum;



        // Mappings of stop ids to other aspects of the respective vehicle route:

        // Maps each stop id to the id of the stop before it in the route of its vehicle.
        std::vector<int> stopIdToIdOfPrevStop;

        // Maps each stop id to its position in the route of its vehicle.
        std::vector<int> stopIdToPosition;

        // stopIdToLeeway[id] is the current leeway in the leg starting at the stop with stopId id.
        std::vector<int> stopIdToLeeway;

        // stopIdToVehicleId[stopId] is the id of the vehicle that the stop with stopId is currently part of the route of.
        std::vector<int> stopIdToVehicleId;

        // Pickups and dropoffs per request as dynamic ragged 2D-arrays.
        // The range requestsPickedUpAtStop[rangeOfRequestsPickedUpAtStop[stopId].start ... rangeOfRequestsPickedUpAtStop[stopId].end]
        // stores the IDs of all requests that are picked up at stop with ID stopId. (Analogous for dropoffs.)
        std::vector<ValueBlockPosition> rangeOfRequestsPickedUpAtStop;
        std::vector<int> requestsPickedUpAtStop;
        std::vector<ValueBlockPosition> rangeOfRequestsDroppedOffAtStop;
        std::vector<int> requestsDroppedOffAtStop;


        // Other data:

        int maxLeeway;
        int stopIdOfMaxLeeway;

        int maxLegLength;
        int stopIdOfMaxLegLength;

        std::stack<int, std::vector<int>> unusedStopIds;
        int nextUnusedStopId;
        int maxStopId;
    };
}