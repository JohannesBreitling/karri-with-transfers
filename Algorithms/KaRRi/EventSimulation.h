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

#include "DataStructures/Queues/AddressableKHeap.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Tools/Workarounds.h"
#include "Tools/Logging/LogManager.h"
#include "Tools/Timer.h"

namespace karri {


    template<typename AssignmentFinderT,
            typename SystemStateUpdaterT,
            typename ScheduledStopsT>
    class EventSimulation {

        enum VehicleState {
            OUT_OF_SERVICE,
            IDLING,
            DRIVING,
            STOPPING
        };

        enum RequestState {
            NOT_RECEIVED,
            ASSIGNED_TO_PVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_DVEH, // Assignment with transfer, request is assigned to the pickup vehicle
            ASSIGNED_TO_VEH, // Assigned to vehicle or assigned to dropoff vehicle if the request will be satisfied including a transfer
            WALKING_TO_DEST,
            FINISHED
        };

        // Stores information about assignment and departure time of a request needed for logging on arrival of the
        // request.
        struct RequestData {
            int depTime; // Departure at pickup
            int walkingTimeToPickup;
            int walkingTimeFromDropoff;
            int assignmentCost;

            // Indicate if the request is satisfied using assignment with transfer
            bool usingTransfer;
            
            // Arrival and departure at transfer
            int arrAtTransferPoint;
            int depTimeAtTransfer;
            int arrAtDropoff;
            int depTimeAtPickup;
        };

    public:


        EventSimulation(
                const Fleet &fleet, const std::vector<Request> &requests,
                AssignmentFinderT &assignmentFinder, SystemStateUpdaterT &systemStateUpdater,
                const ScheduledStopsT &scheduledStops,
                const bool verbose = true)
                : fleet(fleet),
                  requests(requests),
                  assignmentFinder(assignmentFinder),
                  systemStateUpdater(systemStateUpdater),
                  scheduledStops(scheduledStops),
                  vehicleEvents(fleet.size()),
                  requestEvents(requests.size()),
                  vehicleState(fleet.size(), OUT_OF_SERVICE),
                  requestState(requests.size(), NOT_RECEIVED),
                  requestData(requests.size(), RequestData()),
                  eventSimulationStatsLogger(LogManager<std::ofstream>::getLogger("eventsimulationstats.csv",
                                                                                  "occurrence_time,"
                                                                                  "type,"
                                                                                  "running_time\n")),
                  assignmentQualityStatsLogger(LogManager<std::ofstream>::getLogger("assignmentquality.csv",
                                                                              "request_id,"
                                                                              "using_transfer,"
                                                                              "arr_time,"
                                                                              "wait_time,"
                                                                              "ride_time,"
                                                                              "trip_time,"
                                                                              "walk_to_pickup_time,"
                                                                              "walk_to_dropoff_time,"
                                                                              "cost\n")),
                  legStatsLogger(LogManager<std::ofstream>::getLogger("legstats.csv",
                                                                      "vehicle_id,"
                                                                      "stop_time,"
                                                                      "dep_time,"
                                                                      "arr_time,"
                                                                      "drive_time,"
                                                                      "occupancy\n")),
                  progressBar(requests.size(), verbose) {
            progressBar.setDotOutputInterval(1);
            progressBar.setPercentageOutputInterval(5);
            for (const auto &veh: fleet)
                vehicleEvents.insert(veh.vehicleId, veh.startOfServiceTime);
            for (const auto &req: requests)
                requestEvents.insert(req.requestId, req.requestTime);
        }

        void run() {

            while (!(vehicleEvents.empty() && requestEvents.empty())) {
                // Pop next event from either queue. Request event has precedence if at the same time as vehicle event.
                int id, occTime;

                if (requestEvents.empty()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.empty()) {
                    requestEvents.min(id, occTime);
                    handleRequestEvent(id, occTime);
                    continue;
                }

                if (vehicleEvents.minKey() < requestEvents.minKey()) {
                    vehicleEvents.min(id, occTime);
                    handleVehicleEvent(id, occTime);
                    continue;
                }

                requestEvents.min(id, occTime);
                handleRequestEvent(id, occTime);
            }
        }

    private:

        void handleVehicleEvent(const int vehId, const int occTime) {
            switch (vehicleState[vehId]) {
                case OUT_OF_SERVICE:
                    handleVehicleStartup(vehId, occTime);
                    break;
                case IDLING:
                    handleVehicleShutdown(vehId, occTime);
                    break;
                case DRIVING:
                    handleVehicleArrivalAtStop(vehId, occTime);
                    break;
                case STOPPING:
                    handleVehicleDepartureFromStop(vehId, occTime);
                    break;
                default:
                    break;
            }
        }

        void handleRequestEvent(const int reqId, const int occTime) {
            switch (requestState[reqId]) {
                case NOT_RECEIVED:
                    handleRequestReceipt(reqId, occTime);
                    break;
                
                case ASSIGNED_TO_VEH:
                case ASSIGNED_TO_DVEH:
                case ASSIGNED_TO_PVEH:
                    // When assigned to a vehicle, there should be no request event until the dropoff.
                    // At that point the request state becomes WALKING_TO_DEST.
                    assert(false);
                    break;
                case WALKING_TO_DEST:
                    handleWalkingArrivalAtDest(reqId, occTime);
                    break;
                case FINISHED:
                    assert(false);
                    break;
                default:
                    break;
            }
        }

        void handleVehicleStartup(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == OUT_OF_SERVICE);
            assert(fleet[vehId].startOfServiceTime == occTime);
            unused(occTime);
            Timer timer;

            // Vehicle may have already been assigned stops. In this case it will start driving right away:
            if (scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            } else {
                vehicleState[vehId] = IDLING;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleStartup," << time << '\n';
        }

        void handleVehicleShutdown(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == IDLING);
            assert(fleet[vehId].endOfServiceTime == occTime);
            unused(occTime);
            assert(!scheduledStops.hasNextScheduledStop(vehId));
            Timer timer;

            vehicleState[vehId] = OUT_OF_SERVICE;

            int id, key;
            vehicleEvents.deleteMin(id, key);
            assert(id == vehId && key == occTime);
            systemStateUpdater.notifyVehicleReachedEndOfServiceTime(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleShutdown," << time << '\n';
        }

        void handleVehicleArrivalAtStop(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == DRIVING);
            assert(scheduledStops.getNextScheduledStop(vehId).arrTime == occTime);
            Timer timer;

            const auto prevStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
            legStatsLogger << vehId << ','
                           << prevStop.depTime - prevStop.arrTime << ','
                           << prevStop.depTime << ','
                           << occTime << ','
                           << occTime - prevStop.depTime << ','
                           << prevStop.occupancyInFollowingLeg << '\n';

            vehicleState[vehId] = STOPPING;
            const auto reachedStop = scheduledStops.getNextScheduledStop(vehId);

            // Handle dropoffs at reached stop: Insert walking arrival event at the time when passenger will arrive at
            // destination. Thus, all requests are logged in the order of the arrival at their destination.
            for (const auto &reqId: reachedStop.requestsDroppedOffHere) {
                const auto &reqData = requestData[reqId];

                if (requestState[reqId] == ASSIGNED_TO_PVEH) {
                    handleArrivalAtTransferPoint(reqId, occTime);
                    requestState[reqId] = ASSIGNED_TO_DVEH;
                } else {
                    requestState[reqId] = WALKING_TO_DEST;
                    requestEvents.insert(reqId, occTime + reqData.walkingTimeFromDropoff);
                }
            }

            // Next event for this vehicle is the departure at this stop:
            vehicleEvents.increaseKey(vehId, reachedStop.depTime);
            systemStateUpdater.notifyStopStarted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleArrival," << time << '\n';
        }

        void handleVehicleDepartureFromStop(const int vehId, const int occTime) {
            assert(vehicleState[vehId] == STOPPING);
            assert(scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime == occTime);
            Timer timer;

            if (!scheduledStops.hasNextScheduledStop(vehId)) {
                vehicleState[vehId] = IDLING;
                vehicleEvents.increaseKey(vehId, fleet[vehId].endOfServiceTime);
            } else {
                // Remember departure time for all requests picked up at this stop:
                const auto curStop = scheduledStops.getCurrentOrPrevScheduledStop(vehId);
                for (const auto &reqId: curStop.requestsPickedUpHere) {
                    if (!requestData[reqId].usingTransfer) {
                        requestData[reqId].depTime = occTime;
                    }
                }
                vehicleState[vehId] = DRIVING;
                vehicleEvents.increaseKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
            }

            systemStateUpdater.notifyStopCompleted(fleet[vehId]);

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",VehicleDeparture," << time << '\n';
        }

        void handleRequestReceipt(const int reqId, const int occTime) {
            ++progressBar;
            assert(requestState[reqId] == NOT_RECEIVED);
            assert(requests[reqId].requestTime == occTime);
            Timer timer;

            const auto &request = requests[reqId];
            const auto &asgnFinderResponse = assignmentFinder.findBestAssignment(request);
            systemStateUpdater.writeBestAssignmentToLogger();

            if (asgnFinderResponse.improvementThroughTransfer()) {
                applyAssignmentWithTransfer(asgnFinderResponse.getBestAssignmentWithTransfer(), asgnFinderResponse.getBestCostWithTransfer(), reqId);
            } else {
                applyAssignment(asgnFinderResponse, reqId, occTime);
            }

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestReceipt," << time << '\n';
        }

        void handleArrivalAtTransferPoint(const int reqId, const int occTime) {
            unused(reqId, occTime);
            assert(requestState[reqId] == ASSIGNED_TO_PVEH);
        }

        template<typename AssignmentWithTransferT>
        void applyAssignmentWithTransfer(const AssignmentWithTransferT &asgn, const int& cost, const int reqId) {
            if (!asgn.dVeh || !asgn.pVeh || !asgn.pickup || !asgn.dropoff) {
                requestState[reqId] = FINISHED;
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            int id, key;
            requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff

            requestState[reqId] = ASSIGNED_TO_PVEH;
            requestData[reqId].walkingTimeToPickup = asgn.pickup->walkingDist;
            requestData[reqId].walkingTimeFromDropoff = asgn.dropoff->walkingDist;
            requestData[reqId].assignmentCost = cost;
            requestData[reqId].usingTransfer = true;

            requestData[reqId].arrAtTransferPoint = asgn.arrAtTransferPoint;
            requestData[reqId].depTimeAtTransfer = asgn.depAtTransfer;
            requestData[reqId].depTimeAtPickup = asgn.depAtPickup;
            requestData[reqId].arrAtDropoff = asgn.arrAtDropoff;
            
            int pickupStopId, transferStopIdPVeh, transferStopIdDVeh, dropoffStopId; 
            systemStateUpdater.insertBestAssignmentWithTransfer(asgn, pickupStopId, transferStopIdPVeh, transferStopIdDVeh, dropoffStopId);
            systemStateUpdater.writePerformanceLogs();

            const auto pVehId = asgn.pVeh->vehicleId;
            const auto dVehId = asgn.dVeh->vehicleId;

            switch (vehicleState[pVehId]) {
                case STOPPING:
                    // Update event time to departure time at current stop since it may have changed
                    vehicleEvents.updateKey(pVehId, scheduledStops.getCurrentOrPrevScheduledStop(pVehId).depTime);
                    break;
                case IDLING:
                    vehicleState[pVehId] = VehicleState::DRIVING;
                    [[fallthrough]];
                case DRIVING:
                    // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                    vehicleEvents.updateKey(pVehId, scheduledStops.getNextScheduledStop(pVehId).arrTime);
                    [[fallthrough]];
                default:
                    break;
            }

            switch (vehicleState[dVehId]) {
                case STOPPING:
                    // Update event time to departure time at current stop since it may have changed
                    vehicleEvents.updateKey(dVehId, scheduledStops.getCurrentOrPrevScheduledStop(dVehId).depTime);
                    break;
                case IDLING:
                    vehicleState[dVehId] = VehicleState::DRIVING;
                    [[fallthrough]];
                case DRIVING:
                    // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                    vehicleEvents.updateKey(dVehId, scheduledStops.getNextScheduledStop(dVehId).arrTime);
                    [[fallthrough]];
                default:
                    break;
            }
        }

        template<typename AssignmentFinderResponseT>
        void applyAssignment(const AssignmentFinderResponseT &asgnFinderResponse, const int reqId, const int occTime) {
            if (asgnFinderResponse.isNotUsingVehicleBest()) {
                requestState[reqId] = WALKING_TO_DEST;
                requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();
                requestData[reqId].depTime = occTime;
                requestData[reqId].walkingTimeToPickup = 0;
                requestData[reqId].walkingTimeFromDropoff = asgnFinderResponse.getNotUsingVehicleDist();
                requestData[reqId].usingTransfer = false;
                requestData[reqId].arrAtTransferPoint = -1;
                requestData[reqId].depTimeAtTransfer = -1;
                requestEvents.increaseKey(reqId, occTime + asgnFinderResponse.getNotUsingVehicleDist());
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            int id, key;
            requestEvents.deleteMin(id, key); // event for walking arrival at dest inserted at dropoff
            assert(id == reqId && key == occTime);

            const auto &bestAsgn = asgnFinderResponse.getBestAssignment();
            if (!bestAsgn.vehicle || !bestAsgn.pickup || !bestAsgn.dropoff) {
                requestState[reqId] = FINISHED;
                systemStateUpdater.writePerformanceLogs();
                return;
            }

            requestState[reqId] = ASSIGNED_TO_VEH;
            requestData[reqId].walkingTimeToPickup = bestAsgn.pickup->walkingDist;
            requestData[reqId].walkingTimeFromDropoff = bestAsgn.dropoff->walkingDist;
            requestData[reqId].assignmentCost = asgnFinderResponse.getBestCost();

            int pickupStopId, dropoffStopId;
            systemStateUpdater.insertBestAssignment(pickupStopId, dropoffStopId);
            systemStateUpdater.writePerformanceLogs();
            assert(pickupStopId >= 0 && dropoffStopId >= 0);

            const auto vehId = bestAsgn.vehicle->vehicleId;
            switch (vehicleState[vehId]) {
                case STOPPING:
                    // Update event time to departure time at current stop since it may have changed
                    vehicleEvents.updateKey(vehId, scheduledStops.getCurrentOrPrevScheduledStop(vehId).depTime);
                    break;
                case IDLING:
                    vehicleState[vehId] = VehicleState::DRIVING;
                    [[fallthrough]];
                case DRIVING:
                    // Update event time to arrival time at next stop since it may have changed (also for case of idling).
                    vehicleEvents.updateKey(vehId, scheduledStops.getNextScheduledStop(vehId).arrTime);
                    [[fallthrough]];
                default:
                    break;
            }
        }

        void handleWalkingArrivalAtDest(const int reqId, const int occTime) {
            assert(requestState[reqId] == WALKING_TO_DEST);
            Timer timer;

            const auto &reqData = requestData[reqId];
            requestState[reqId] = FINISHED;
            int id, key;
            requestEvents.deleteMin(id, key);
            assert(id == reqId && key == occTime);

            int waitTime;
            int arrTime;
            int rideTime;
            int tripTime;
            if (!reqData.usingTransfer) {
                arrTime = occTime;
                tripTime = arrTime - requests[reqId].requestTime;
                waitTime = reqData.depTime - requests[reqId].requestTime;
                rideTime = occTime - reqData.walkingTimeFromDropoff - reqData.depTime;
            } else {
                // Calculate the values if the assignment consists of two vehicles
                arrTime = reqData.arrAtDropoff + reqData.walkingTimeFromDropoff;
                tripTime = arrTime - requests[reqId].requestTime;
                int waitAtTransfer = reqData.depTimeAtTransfer - reqData.arrAtTransferPoint;
                assert(waitAtTransfer >= 0);
                waitTime = reqData.depTimeAtPickup - requests[reqId].requestTime + waitAtTransfer;
                rideTime = arrTime - reqData.walkingTimeFromDropoff - reqData.depTimeAtPickup - waitAtTransfer;
            }

            assert(waitTime >= 0 && rideTime >= 0);
            
            assignmentQualityStatsLogger << reqId << ','
                                   << reqData.usingTransfer << ","
                                   << arrTime << ','
                                   << waitTime << ','
                                   << rideTime << ','
                                   << tripTime << ','
                                   << reqData.walkingTimeToPickup << ','
                                   << reqData.walkingTimeFromDropoff << ','
                                   << reqData.assignmentCost << '\n';

            const auto time = timer.elapsed<std::chrono::nanoseconds>();
            eventSimulationStatsLogger << occTime << ",RequestWalkingArrival," << time << '\n';
        }


        const Fleet &fleet;
        const std::vector<Request> &requests;
        AssignmentFinderT &assignmentFinder;
        SystemStateUpdaterT &systemStateUpdater;
        const ScheduledStopsT &scheduledStops;

        AddressableQuadHeap vehicleEvents;
        AddressableQuadHeap requestEvents;

        std::vector<VehicleState> vehicleState;
        std::vector<RequestState> requestState;

        std::vector<RequestData> requestData;

        std::ofstream &eventSimulationStatsLogger;
        std::ofstream &assignmentQualityStatsLogger;
        std::ofstream &legStatsLogger;
        ProgressBar progressBar;

    };
}