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

#include "Algorithms/KaRRi/RouteState.h"
#include "Algorithms/KaRRi/LastStopSearches/OnlyLastStopsAtVerticesBucketSubstitute.h"
#include "PathTracker.h"

namespace karri {

    // Updates the system state consisting of the route state (schedules of vehicles and additional information about
    // stops) as well as the bucket state (precomputed information for fast shortest path queries to vehicle stops).
    template<typename InputGraphT,
            typename EllipticBucketsEnvT,
            typename LastStopBucketsEnvT,
            typename CurVehLocsT,
            typename PathTrackerT,
            typename LoggerT = NullLogger>
    class SystemStateUpdater {

    public:

        SystemStateUpdater(const InputGraphT &inputGraph, const CH &vehCh, RequestState &requestState,
                           const CurVehLocsT &curVehLocs,
                           PathTrackerT &pathTracker,
                           RouteState &routeState, EllipticBucketsEnvT &ellipticBucketsEnv,
                           LastStopBucketsEnvT &lastStopBucketsEnv)
                : inputGraph(inputGraph),
                  vehCh(vehCh),
                  requestState(requestState),
                  curVehLocs(curVehLocs),
                  pathTracker(pathTracker),
                  routeState(routeState),
                  ellipticBucketsEnv(ellipticBucketsEnv),
                  lastStopBucketsEnv(lastStopBucketsEnv),
                  bestAssignmentsOverallLogger(LogManager<LoggerT>::getLogger("bestassignmentsoverall.csv",
                                                                              "request_id,"
                                                                              "request_time,"
                                                                              "direct_od_dist,"
                                                                              "number_of_legs,"
                                                                              "cost\n")),
                  bestAssignmentsWithoutUsingVehicleLogger(LogManager<LoggerT>::getLogger("bestassignmentswithoutvehicle.csv",
                                                                                          "request_id,"
                                                                                          "request_time,"
                                                                                          "direct_walking_dist,"
                                                                                          "cost\n")),
                  bestAssignmentsWithoutTransferLogger(LogManager<LoggerT>::getLogger("bestassignmentswithouttransfer.csv",
                                                                       "request_id,"
                                                                       "request_time,"
                                                                       "direct_od_dist,"
                                                                       "vehicle_id,"
                                                                       "pickup_insertion_point,"
                                                                       "dropoff_insertion_point,"
                                                                       "dist_to_pickup,"
                                                                       "dist_from_pickup,"
                                                                       "dist_to_dropoff,"
                                                                       "dist_from_dropoff,"
                                                                       "pickup_id,"
                                                                       "pickup_walking_dist,"
                                                                       "dropoff_id,"
                                                                       "dropoff_walking_dist,"
                                                                       "num_stops,"
                                                                       "veh_dep_time_at_stop_before_pickup,"
                                                                       "veh_dep_time_at_stop_before_dropoff,"
                                                                       "cost\n")),
                  bestAssignmentsWithTransferLogger(LogManager<LoggerT>::getLogger("bestassignmentswithtransfer.csv",
                                                                                   "request_id,"
                                                                                   "request_time,"
                                                                                   "direct_od_dist,"
                                                                                   "pickup_vehicle_id,"
                                                                                   "dropoff_vehicle_id,"
                                                                                   "pickup_insertion_point,"
                                                                                   "transfer_pveh_insertion_point,"
                                                                                   "transfer_dveh_insertion_point,"
                                                                                   "dropoff_insertion_point,"
                                                                                   "dist_to_pickup,"
                                                                                   "dist_from_pickup,"
                                                                                   "dist_to_transfer_pveh,"
                                                                                   "dist_from_transfer_pveh,"
                                                                                   "dist_to_transfer_dveh,"
                                                                                   "dist_from_transfer_dveh,"
                                                                                   "dist_to_dropoff,"
                                                                                   "dist_from_dropoff,"
                                                                                   "pickup_type,"
                                                                                   "transfer_type_pveh,"
                                                                                   "transfer_type_dveh,"
                                                                                   "dropoff_type,"
                                                                                   "pickup_id,"
                                                                                   "pickup_walking_dist,"
                                                                                   "dropoff_id,"
                                                                                   "dropoff_walking_dist,"
                                                                                   "num_stops_pveh,"
                                                                                   "num_stops_dveh,"
                                                                                   "veh_dep_time_at_stop_before_pickup,"
                                                                                   "veh_dep_time_at_stop_before_transfer_pveh,"
                                                                                   "veh_dep_time_at_stop_before_transfer_dveh,"
                                                                                   "veh_dep_time_at_stop_before_dropoff,"
                                                                                   "cost\n")),
                  overallPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::DispatchingPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::DispatchingPerformanceStats::LOGGER_COLS))),
                  initializationPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::InitializationPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::InitializationPerformanceStats::LOGGER_COLS))),
                  ellipticBchPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::EllipticBCHPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::EllipticBCHPerformanceStats::LOGGER_COLS))),
                  pdDistancesPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PDDistancesPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::PDDistancesPerformanceStats::LOGGER_COLS))),
                  ordPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::OrdAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::OrdAssignmentsPerformanceStats::LOGGER_COLS))),
                  pbnsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PbnsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PbnsAssignmentsPerformanceStats::LOGGER_COLS))),
                  palsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::PalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::PalsAssignmentsPerformanceStats::LOGGER_COLS))),
                  dalsPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::DalsAssignmentsPerformanceStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(
                                                                 stats::DalsAssignmentsPerformanceStats::LOGGER_COLS))),
                  updatePerfLogger(LogManager<LoggerT>::getLogger(stats::UpdatePerformanceStats::LOGGER_NAME,
                                                                  "request_id, " +
                                                                  std::string(
                                                                          stats::UpdatePerformanceStats::LOGGER_COLS))),
                  ellipseReconstructionPerfLogger(
                          LogManager<LoggerT>::getLogger(stats::EllipseReconstructionStats::LOGGER_NAME,
                                                         "request_id, " +
                                                         std::string(stats::EllipseReconstructionStats::LOGGER_COLS))),
                  ordinaryTransferPerfLogger(LogManager<LoggerT>::getLogger(
                          stats::AssignmentsWithOrdinaryTransferPerformanceStats::LOGGER_NAME,
                          "request_id," +
                          std::string(stats::AssignmentsWithOrdinaryTransferPerformanceStats::LOGGER_COLS))),

                  transferALSPVehPerfLogger(LogManager<LoggerT>::getLogger(
                          stats::AssignmentsWithTransferALSPVehPerformanceStats::LOGGER_NAME,
                          "request_id," +
                          std::string(stats::AssignmentsWithTransferALSPVehPerformanceStats::LOGGER_COLS))),

                  transferALSDVehPerfLogger(LogManager<LoggerT>::getLogger(
                          stats::AssignmentsWithTransferALSDVehPerformanceStats::LOGGER_NAME,
                          "request_id," +
                          std::string(stats::AssignmentsWithTransferALSDVehPerformanceStats::LOGGER_COLS))),

                  assignmentsCostLogger(LogManager<LoggerT>::getLogger(stats::AssignmentCostStats::LOGGER_NAME,
                                                                       "request_id," +
                                                                       std::string(
                                                                               stats::AssignmentCostStats::LOGGER_COLS))),
                  ordinaryTransferRanksLogger(LogManager<LoggerT>::getLogger("ordinarytransfer_ranks.csv",
                                                                             "request_id,"
                                                                             "pickup_vehicle_id,"
                                                                             "dropoff_vehicle_id,"
                                                                             "transfer_rank,"
                                                                             "pveh_prev_stop_rank,"
                                                                             "pveh_next_stop_rank,"
                                                                             "dveh_prev_stop_rank,"
                                                                             "dveh_next_stop_rank\n")) {}


        void
        insertBestAssignmentWithTransfer(const AssignmentWithTransfer &asgn, int &pickupStopId, int &transferStopIdPVeh,
                                         int &transferStopIdDVeh, int &dropoffStopId) {
            Timer timer;

            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.pickup->loc));
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff->loc));
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.transfer.loc));
            assert(asgn.pVeh != nullptr);
            assert(asgn.dVeh != nullptr);

            const auto pVehId = asgn.pVeh->vehicleId;
            const auto dVehId = asgn.dVeh->vehicleId;

            const auto numStopsBeforePVeh = routeState.numStopsOf(pVehId);
            const auto numStopsBeforeDVeh = routeState.numStopsOf(dVehId);

            const auto depTimeAtLastStopBeforePVeh = routeState.schedDepTimesFor(pVehId)[numStopsBeforePVeh - 1];
            const auto depTimeAtLastStopBeforeDVeh = routeState.schedDepTimesFor(dVehId)[numStopsBeforeDVeh - 1];

            timer.restart();

            auto [pIdxPVeh, dIdxPVeh] = routeState.insertPVeh(asgn, requestState);
            updateBucketStatePVeh(asgn, pIdxPVeh, dIdxPVeh, depTimeAtLastStopBeforePVeh);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting.
            if (asgn.pickupIdx == 0 && numStopsBeforePVeh > 1 &&
                routeState.schedDepTimesFor(pVehId)[0] < requestState.originalRequest.requestTime) {
                createIntermediateStopStopAtCurrentLocationForReroute(*asgn.pVeh,
                                                                      requestState.originalRequest.requestTime);
                assert(routeState.vehicleIdOf(routeState.stopIdsFor(pVehId)[1]) == pVehId);

                ++pIdxPVeh;
                ++dIdxPVeh;
            }

            assert(routeState.assertRoutePVeh(asgn));

            auto [pIdxDVeh, dIdxDVeh] = routeState.insertDVeh(asgn, requestState);
            updateBucketStateDVeh(asgn, pIdxDVeh, dIdxDVeh, depTimeAtLastStopBeforeDVeh);

            if (asgn.transferIdxDVeh == 0 && numStopsBeforeDVeh > 1 &&
                routeState.schedDepTimesFor(dVehId)[0] < requestState.originalRequest.requestTime) {
                createIntermediateStopStopAtCurrentLocationForReroute(*asgn.dVeh,
                                                                      requestState.originalRequest.requestTime);
                assert(routeState.vehicleIdOf(routeState.stopIdsFor(dVehId)[1]) == dVehId);

                ++pIdxDVeh;
                ++dIdxDVeh;
            }

            assert(routeState.assertRouteDVeh(asgn));

            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            pickupStopId = routeState.stopIdsFor(pVehId)[pIdxPVeh];
            transferStopIdPVeh = routeState.stopIdsFor(pVehId)[dIdxPVeh];
            transferStopIdDVeh = routeState.stopIdsFor(dVehId)[pIdxDVeh];
            dropoffStopId = routeState.stopIdsFor(dVehId)[dIdxDVeh];

            assert(routeState.vehicleIdOf(pickupStopId) == pVehId);
            assert(routeState.vehicleIdOf(transferStopIdPVeh) == pVehId);
            assert(routeState.vehicleIdOf(transferStopIdDVeh) == dVehId);
            assert(routeState.vehicleIdOf(dropoffStopId) == dVehId);

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, transferStopIdPVeh);
            pathTracker.registerPdEventsForBestAssignment(transferStopIdDVeh, dropoffStopId);
        }

        void insertBestAssignment(int &pickupStopId, int &dropoffStopId) {
            Timer timer;

            if (requestState.isNotUsingVehicleBest()) {
                pickupStopId = -1;
                dropoffStopId = -1;
                return;
            }

            const auto &asgn = requestState.getBestAssignmentWithoutTransfer();
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.pickup->loc));
            requestState.chosenPDLocsRoadCategoryStats().incCountForCat(inputGraph.osmRoadCategory(asgn.dropoff->loc));
            assert(asgn.vehicle != nullptr);

            const auto vehId = asgn.vehicle->vehicleId;
            const auto numStopsBefore = routeState.numStopsOf(vehId);
            const auto depTimeAtLastStopBefore = routeState.schedDepTimesFor(vehId)[numStopsBefore - 1];

            timer.restart();
            auto [pickupIndex, dropoffIndex] = routeState.insert(asgn, requestState);
            const auto routeUpdateTime = timer.elapsed<std::chrono::nanoseconds>();
            requestState.stats().updateStats.updateRoutesTime += routeUpdateTime;

            updateBucketState(asgn, pickupIndex, dropoffIndex, depTimeAtLastStopBefore);

            // If the vehicle has to be rerouted at its current location for a PBNS assignment, we introduce an
            // intermediate stop at its current location representing the rerouting.
            if (asgn.pickupStopIdx == 0 && numStopsBefore > 1 && routeState.schedDepTimesFor(vehId)[0] <
                                                                 requestState.originalRequest.requestTime) {
                createIntermediateStopStopAtCurrentLocationForReroute(*asgn.vehicle,
                                                                      requestState.originalRequest.requestTime);
                ++pickupIndex;
                ++dropoffIndex;
            }

            pickupStopId = routeState.stopIdsFor(vehId)[pickupIndex];
            dropoffStopId = routeState.stopIdsFor(vehId)[dropoffIndex];

            // Register the inserted pickup and dropoff with the path data
            pathTracker.registerPdEventsForBestAssignment(pickupStopId, dropoffStopId);
        }

        void notifyStopStarted(const Vehicle &veh) {

            // Update buckets and route state
            ellipticBucketsEnv.deleteSourceBucketEntries(veh, 0);
            ellipticBucketsEnv.deleteTargetBucketEntries(veh, 1);
            routeState.removeStartOfCurrentLeg(veh.vehicleId);

            // If vehicle has become idle, update last stop bucket entries
            if (routeState.numStopsOf(veh.vehicleId) == 1) {
                lastStopBucketsEnv.updateBucketEntries(veh, 0);
            }
        }


        void notifyStopCompleted(const Vehicle &veh) {
            pathTracker.logCompletedStop(veh);
        }

        void notifyVehicleReachedEndOfServiceTime(const Vehicle &veh) {
            const auto vehId = veh.vehicleId;
            assert(routeState.numStopsOf(vehId) == 1);

            lastStopBucketsEnv.removeIdleBucketEntries(veh, 0);

            routeState.removeStartOfCurrentLeg(vehId);
        }


        void writeBestAssignmentToLogger() {
            // Set up the best assignment for logging
            int numLegs = -1;
            switch (requestState.getBestAsgnType()) {
                case RequestState::BestAsgnType::NOT_USING_VEHICLE:
                    numLegs = 0; // Nothing to log
                    break;
                case RequestState::BestAsgnType::ONE_LEG:
                    numLegs = 1;
                    break;
                case RequestState::BestAsgnType::TWO_LEGS:
                    numLegs = 2;
                    break;
                default:
                    numLegs = 0;
            };
            bestAssignmentsOverallLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", "
                    << numLegs << ", "
                    << requestState.getBestCost() << "\n";

            bestAssignmentsWithoutUsingVehicleLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", ";

            bestAssignmentsWithoutTransferLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", ";

            bestAssignmentsWithTransferLogger
                    << requestState.originalRequest.requestId << ", "
                    << requestState.originalRequest.requestTime << ", "
                    << requestState.originalReqDirectDist << ", ";

            // Log the cost as well
            assignmentsCostLogger << requestState.originalRequest.requestId << ", "
                                  << requestState.stats().costStats.getLoggerRow() << "\n";

            const int costWithoutVehicle = requestState.getBestCostWithoutUsingVehicle();
            const int costWOT = requestState.getCostObjectWithoutTransfer().total;
            const int costWT = requestState.getCostObjectWithTransfer().total;

            if (costWithoutVehicle >= INFTY) {
                bestAssignmentsWithoutUsingVehicleLogger << "-1,inf\n";
            }

            if (costWOT >= INFTY) {
                bestAssignmentsWithoutTransferLogger << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
            }

            if (costWT >= INFTY) {
                bestAssignmentsWithTransferLogger
                        << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,inf\n";
            }

            using time_utils::getVehDepTimeAtStopForRequest;
            if (costWithoutVehicle < INFTY) {
                bestAssignmentsWithoutUsingVehicleLogger << requestState.getNotUsingVehicleDist() << ", "
                                                         << requestState.getBestCostWithoutUsingVehicle() << "\n";
            }

            if (costWOT < INFTY) {
                const auto &bestAsgn = requestState.getBestAssignmentWithoutTransfer();

                const auto &vehId = bestAsgn.vehicle->vehicleId;
                const auto &numStops = routeState.numStopsOf(vehId);
                const auto &vehDepTimeBeforePickup = getVehDepTimeAtStopForRequest(vehId, bestAsgn.pickupStopIdx,
                                                                                   requestState, routeState);
                const auto &vehDepTimeBeforeDropoff = getVehDepTimeAtStopForRequest(vehId, bestAsgn.dropoffStopIdx,
                                                                                    requestState, routeState);
                bestAssignmentsWithoutTransferLogger
                        << vehId << ", "
                        << bestAsgn.pickupStopIdx << ", "
                        << bestAsgn.dropoffStopIdx << ", "
                        << bestAsgn.distToPickup << ", "
                        << bestAsgn.distFromPickup << ", "
                        << bestAsgn.distToDropoff << ", "
                        << bestAsgn.distFromDropoff << ", "
                        << bestAsgn.pickup->id << ", "
                        << bestAsgn.pickup->walkingDist << ", "
                        << bestAsgn.dropoff->id << ", "
                        << bestAsgn.dropoff->walkingDist << ", "
                        << numStops << ", "
                        << vehDepTimeBeforePickup << ", "
                        << vehDepTimeBeforeDropoff << ", "
                        << requestState.getBestCostWithoutTransfer() << "\n";
            }

            if (costWT < INFTY) {
                const auto &bestAsgnWT = requestState.getBestAssignmentWithTransfer();
                const int pVehId = bestAsgnWT.pVeh->vehicleId;
                const int dVehId = bestAsgnWT.dVeh->vehicleId;
                const int pickupIdx = bestAsgnWT.pickupIdx;
                const int transferIdxPVeh = bestAsgnWT.transferIdxPVeh;
                const int transferIdxDVeh = bestAsgnWT.transferIdxDVeh;
                const int dropoffIdx = bestAsgnWT.dropoffIdx;

                const auto &numStopsPVeh = routeState.numStopsOf(pVehId);
                const auto &numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto &vehDepTimeBeforePickupWT = getVehDepTimeAtStopForRequest(pVehId, pickupIdx,
                                                                                     requestState, routeState);
                const auto &vehDepTimeBeforeTransferPVeh = getVehDepTimeAtStopForRequest(pVehId, transferIdxPVeh,
                                                                                         requestState, routeState);
                const auto &vehDepTimeBeforeTransferDVeh = getVehDepTimeAtStopForRequest(dVehId, transferIdxDVeh,
                                                                                         requestState, routeState);
                const auto &vehDepTimeBeforeDropoffWT = getVehDepTimeAtStopForRequest(dVehId, dropoffIdx,
                                                                                      requestState, routeState);

                const std::array<std::string, 4> types = {"NOT_SET", "BNS", "ORD", "ALS"};

                const auto pickupType = types[bestAsgnWT.pickupType];
                const auto transferTypePVeh = types[bestAsgnWT.transferTypePVeh];
                const auto transferTypeDVeh = types[bestAsgnWT.transferTypeDVeh];
                const auto dropoffType = types[bestAsgnWT.dropoffType];

                if (bestAsgnWT.transferTypePVeh == ORDINARY && bestAsgnWT.transferTypeDVeh == ORDINARY) {
                    // Log CH ranks of transfer and stop pairs for ordinary transfer
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);
                    const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                    ordinaryTransferRanksLogger
                            << requestState.originalRequest.requestId << ","
                            << pVehId << ","
                            << dVehId << ","
                            << vehCh.rank(inputGraph.edgeHead(bestAsgnWT.transfer.loc)) << ","
                            << vehCh.rank(inputGraph.edgeHead(stopLocationsPVeh[transferIdxPVeh])) << ","
                            << vehCh.rank(inputGraph.edgeHead(stopLocationsPVeh[transferIdxPVeh + 1])) << ","
                            << vehCh.rank(inputGraph.edgeHead(stopLocationsDVeh[transferIdxDVeh])) << ","
                            << vehCh.rank(inputGraph.edgeHead(stopLocationsDVeh[transferIdxDVeh + 1])) << "\n"
                            << std::flush;
                }


                bestAssignmentsWithTransferLogger
                        << pVehId << ", "
                        << dVehId << ", "
                        << pickupIdx << ", "
                        << transferIdxPVeh << ", "
                        << transferIdxDVeh << ", "
                        << dropoffIdx << ", "
                        << bestAsgnWT.distToPickup << ", "
                        << bestAsgnWT.distFromPickup << ", "
                        << bestAsgnWT.distToTransferPVeh << ", "
                        << bestAsgnWT.distFromTransferPVeh << ", "
                        << bestAsgnWT.distToTransferDVeh << ", "
                        << bestAsgnWT.distFromTransferDVeh << ", "
                        << bestAsgnWT.distToDropoff << ", "
                        << bestAsgnWT.distFromDropoff << ", "
                        << pickupType << ", "
                        << transferTypePVeh << ", "
                        << transferTypeDVeh << ", "
                        << dropoffType << ", "
                        << bestAsgnWT.pickup->id << ", "
                        << bestAsgnWT.pickup->walkingDist << ", "
                        << bestAsgnWT.dropoff->id << ", "
                        << bestAsgnWT.dropoff->walkingDist << ", "
                        << numStopsPVeh << ", "
                        << numStopsDVeh << ", "
                        << vehDepTimeBeforePickupWT << ", "
                        << vehDepTimeBeforeTransferPVeh << ", "
                        << vehDepTimeBeforeTransferDVeh << ", "
                        << vehDepTimeBeforeDropoffWT << ", "
                        << requestState.getBestCostWithTransfer() << "\n";
            }
        }

        void writePerformanceLogs() {
            overallPerfLogger << requestState.originalRequest.requestId << ", "
                              << requestState.stats().getLoggerRow() << "\n";
            initializationPerfLogger << requestState.originalRequest.requestId << ", "
                                     << requestState.stats().initializationStats.getLoggerRow() << "\n";
            ellipticBchPerfLogger << requestState.originalRequest.requestId << ", "
                                  << requestState.stats().ellipticBchStats.getLoggerRow() << "\n";
            pdDistancesPerfLogger << requestState.originalRequest.requestId << ", "
                                  << requestState.stats().pdDistancesStats.getLoggerRow() << "\n";
            ordPerfLogger << requestState.originalRequest.requestId << ", "
                          << requestState.stats().ordAssignmentsStats.getLoggerRow() << "\n";
            pbnsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().pbnsAssignmentsStats.getLoggerRow() << "\n";
            palsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().palsAssignmentsStats.getLoggerRow() << "\n";
            dalsPerfLogger << requestState.originalRequest.requestId << ", "
                           << requestState.stats().dalsAssignmentsStats.getLoggerRow() << "\n";
            updatePerfLogger << requestState.originalRequest.requestId << ", "
                             << requestState.stats().updateStats.getLoggerRow() << "\n";
            ellipseReconstructionPerfLogger << requestState.originalRequest.requestId << ", "
                                            << requestState.stats().ellipseReconstructionStats.getLoggerRow() << "\n";
            ordinaryTransferPerfLogger << requestState.originalRequest.requestId << ", "
                                       << requestState.stats().ordinaryTransferStats.getLoggerRow() << "\n";
            transferALSPVehPerfLogger << requestState.originalRequest.requestId << ", "
                                      << requestState.stats().transferALSPVehStats.getLoggerRow() << "\n";
            transferALSDVehPerfLogger << requestState.originalRequest.requestId << ", "
                                      << requestState.stats().transferALSDVehStats.getLoggerRow() << "\n";
        }

    private:

        // If vehicle is rerouted from its current position to a newly inserted stop (PBNS assignment), create new
        // intermediate stop at the vehicle's current position to maintain the invariant of the schedule for the
        // first stop, i.e. dist(s[i], s[i+1]) = schedArrTime(s[i+1]) - schedDepTime(s[i]).
        // Intermediate stop gets an arrival time equal to the request time so the stop is reached immediately,
        // making it the new stop 0. Thus, we do not need to compute target bucket entries for the stop.
        void createIntermediateStopStopAtCurrentLocationForReroute(const Vehicle &veh, const int now) {
            assert(curVehLocs.knowsCurrentLocationOf(veh.vehicleId));
            auto loc = curVehLocs.getCurrentLocationOf(veh.vehicleId);
            LIGHT_KASSERT(loc.depTimeAtHead >= now);
            routeState.createIntermediateStopForReroute(veh.vehicleId, loc.location, now, loc.depTimeAtHead);
            ellipticBucketsEnv.generateSourceBucketEntries(veh, 1);
        }


        void updateBucketStatePVeh(const AssignmentWithTransfer &asgn, const int pickupIdx, const int transferIdxPVeh,
                                   const int depTimeAtLastBeforePVeh) {
            // generateBucketStateForNewStops(asgn, pickupIdx, transferIdxPVeh, transferIdxDVeh, dropoffIdx);

            generateBucketStateForNewStopsPVeh(asgn, pickupIdx, transferIdxPVeh);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.pVeh);
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.pVeh);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const int pVehId = asgn.pVeh->vehicleId;
            const int numStopsAfterPVeh = routeState.numStopsOf(pVehId);

            const bool pickupAtExistingStop = pickupIdx == asgn.pickupIdx;
            const bool transferAtExistingStopPVeh = transferIdxPVeh == asgn.transferIdxPVeh + !pickupAtExistingStop;
            const auto depTimeAtLastStopAfterPVeh = routeState.schedDepTimesFor(pVehId)[numStopsAfterPVeh - 1];
            const bool depTimeAtLastChangedPVeh = depTimeAtLastStopAfterPVeh != depTimeAtLastBeforePVeh;

            if ((transferAtExistingStopPVeh || transferIdxPVeh < numStopsAfterPVeh - 1) && depTimeAtLastChangedPVeh) {
                lastStopBucketsEnv.updateBucketEntries(*asgn.pVeh, numStopsAfterPVeh - 1);
            }
        }

        void updateBucketStateDVeh(const AssignmentWithTransfer &asgn, const int transferIdxDVeh, const int dropoffIdx,
                                   const int depTimeAtLastBeforeDVeh) {
            // generateBucketStateForNewStops(asgn, pickupIdx, transferIdxPVeh, transferIdxDVeh, dropoffIdx);
            generateBucketStateForNewStopsDVeh(asgn, transferIdxDVeh, dropoffIdx);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.dVeh);
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.dVeh);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const int dVehId = asgn.dVeh->vehicleId;

            const int numStopsAfterDVeh = routeState.numStopsOf(dVehId);
            const bool transferAtExistingStopDVeh = transferIdxDVeh == asgn.transferIdxDVeh;
            const bool dropoffAtExistingStop = dropoffIdx == asgn.dropoffIdx + !transferAtExistingStopDVeh;

            const auto depTimeAtLastStopAfterDVeh = routeState.schedDepTimesFor(dVehId)[numStopsAfterDVeh - 1];
            const bool depTimeAtLastChangedDVeh = depTimeAtLastStopAfterDVeh != depTimeAtLastBeforeDVeh;

            if ((dropoffAtExistingStop || dropoffIdx < numStopsAfterDVeh - 1) && depTimeAtLastChangedDVeh) {
                lastStopBucketsEnv.updateBucketEntries(*asgn.dVeh, numStopsAfterDVeh - 1);
            }
        }


        // Updates the bucket state (elliptic buckets, last stop buckets, lastStopsAtVertices structure) given an
        // assignment that has already been inserted into routeState as well as the stop index of the pickup and
        // dropoff after the insertion.
        void updateBucketState(const Assignment &asgn,
                               const int pickupIndex, const int dropoffIndex,
                               const int depTimeAtLastStopBefore) {

            generateBucketStateForNewStops(asgn, pickupIndex, dropoffIndex);

            // If we use buckets sorted by remaining leeway, we have to update the leeway of all
            // entries for stops of this vehicle.
            if constexpr (EllipticBucketsEnvT::SORTED_BY_REM_LEEWAY) {
                ellipticBucketsEnv.updateLeewayInSourceBucketsForAllStopsOf(*asgn.vehicle);
                ellipticBucketsEnv.updateLeewayInTargetBucketsForAllStopsOf(*asgn.vehicle);
            }

            // If last stop does not change but departure time at last stop does change, update last stop bucket entries
            // accordingly.
            const int vehId = asgn.vehicle->vehicleId;
            const auto numStopsAfter = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;
            const auto depTimeAtLastStopAfter = routeState.schedDepTimesFor(vehId)[numStopsAfter - 1];
            const bool depTimeAtLastChanged = depTimeAtLastStopAfter != depTimeAtLastStopBefore;

            if ((dropoffAtExistingStop || dropoffIndex < numStopsAfter - 1) && depTimeAtLastChanged) {
                lastStopBucketsEnv.updateBucketEntries(*asgn.vehicle, numStopsAfter - 1);
            }
        }

        void generateBucketStateForNewStopsPVeh(const AssignmentWithTransfer &asgn, const int pickupIdx,
                                                const int transferIdxPVeh) {
            const auto pVehId = asgn.pVeh->vehicleId;
            const auto &numStopsPVeh = routeState.numStopsOf(pVehId);

            const bool pickupAtExistingStop = pickupIdx == asgn.pickupIdx;
            const bool transferAtExistingStopPVeh = transferIdxPVeh == asgn.transferIdxPVeh + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                assert(pickupIdx > 0);
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.pVeh, pickupIdx);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.pVeh, pickupIdx);
            }

            if (!transferAtExistingStopPVeh) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.pVeh, transferIdxPVeh);

                // If dropoff is not the new last stop, we generate elliptic source buckets for it.
                if (transferIdxPVeh < numStopsPVeh - 1) {
                    ellipticBucketsEnv.generateSourceBucketEntries(*asgn.pVeh, transferIdxPVeh);
                    return;
                }

                // If dropoff is the new last stop, the former last stop becomes a regular stop:
                // Generate elliptic source bucket entries for former last stop
                const auto pickupAtEnd = pickupIdx + 1 == transferIdxPVeh && pickupIdx > asgn.pickupIdx;
                const int formerLastStopIdx = transferIdxPVeh - pickupAtEnd - 1;
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.pVeh, formerLastStopIdx);

                // Remove last stop bucket entries for former last stop and generate them for dropoff
                if (formerLastStopIdx == 0) {
                    lastStopBucketsEnv.removeIdleBucketEntries(*asgn.pVeh, formerLastStopIdx);
                } else {
                    lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.pVeh, formerLastStopIdx);
                }
                lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.pVeh);
            }
        }

        void generateBucketStateForNewStopsDVeh(const AssignmentWithTransfer &asgn, const int transferIdxDVeh,
                                                const int dropoffIdx) {
            const auto dVehId = asgn.dVeh->vehicleId;
            const auto &numStopsDVeh = routeState.numStopsOf(dVehId);

            const bool transferAtExistingStopDVeh = transferIdxDVeh == asgn.transferIdxDVeh;
            const bool dropoffAtExistingStop = dropoffIdx == asgn.dropoffIdx + !transferAtExistingStopDVeh;

            if (!transferAtExistingStopDVeh) {
                assert(transferIdxDVeh > 0);
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.dVeh, transferIdxDVeh);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.dVeh, transferIdxDVeh);
            }

            if (!dropoffAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.dVeh, dropoffIdx);

                // If dropoff is not the new last stop, we generate elliptic source buckets for it.
                if (dropoffIdx < numStopsDVeh - 1) {
                    ellipticBucketsEnv.generateSourceBucketEntries(*asgn.dVeh, dropoffIdx);
                    return;
                }

                // If dropoff is the new last stop, the former last stop becomes a regular stop:
                // Generate elliptic source bucket entries for former last stop
                const auto transferAtEnd = transferIdxDVeh + 1 == dropoffIdx && transferIdxDVeh > asgn.transferIdxDVeh;
                const int formerLastStopIdx = dropoffIdx - transferAtEnd - 1;
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.dVeh, formerLastStopIdx);

                // Remove last stop bucket entries for former last stop and generate them for dropoff
                if (formerLastStopIdx == 0) {
                    lastStopBucketsEnv.removeIdleBucketEntries(*asgn.dVeh, formerLastStopIdx);
                } else {
                    lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.dVeh, formerLastStopIdx);
                }
                lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.dVeh);
            }
        }

        void generateBucketStateForNewStops(const Assignment &asgn, const int pickupIndex, const int dropoffIndex) {
            const auto vehId = asgn.vehicle->vehicleId;
            const auto &numStops = routeState.numStopsOf(vehId);
            const bool pickupAtExistingStop = pickupIndex == asgn.pickupStopIdx;
            const bool dropoffAtExistingStop = dropoffIndex == asgn.dropoffStopIdx + !pickupAtExistingStop;

            if (!pickupAtExistingStop) {
                ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, pickupIndex);
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, pickupIndex);
            }

            // If no new stop was inserted for the pickup, we do not need to generate any new entries for it.
            if (dropoffAtExistingStop)
                return;

            ellipticBucketsEnv.generateTargetBucketEntries(*asgn.vehicle, dropoffIndex);

            // If dropoff is not the new last stop, we generate elliptic source buckets for it.
            if (dropoffIndex < numStops - 1) {
                ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, dropoffIndex);
                return;
            }

            // If dropoff is the new last stop, the former last stop becomes a regular stop:
            // Generate elliptic source bucket entries for former last stop
            const auto pickupAtEnd = pickupIndex + 1 == dropoffIndex && pickupIndex > asgn.pickupStopIdx;
            const int formerLastStopIdx = dropoffIndex - pickupAtEnd - 1;
            ellipticBucketsEnv.generateSourceBucketEntries(*asgn.vehicle, formerLastStopIdx);

            // Remove last stop bucket entries for former last stop and generate them for dropoff
            if (formerLastStopIdx == 0) {
                lastStopBucketsEnv.removeIdleBucketEntries(*asgn.vehicle, formerLastStopIdx);
            } else {
                lastStopBucketsEnv.removeNonIdleBucketEntries(*asgn.vehicle, formerLastStopIdx);
            }
            lastStopBucketsEnv.generateNonIdleBucketEntries(*asgn.vehicle);
        }

        const InputGraphT &inputGraph;
        const CH &vehCh;
        RequestState &requestState;
        const CurVehLocsT &curVehLocs;
        PathTrackerT &pathTracker;

        // Route state
        RouteState &routeState;

        // Bucket state
        EllipticBucketsEnvT &ellipticBucketsEnv;
        LastStopBucketsEnvT &lastStopBucketsEnv;

        // Performance Loggers
        LoggerT &bestAssignmentsOverallLogger;
        LoggerT &bestAssignmentsWithoutUsingVehicleLogger;
        LoggerT &bestAssignmentsWithoutTransferLogger;
        LoggerT &bestAssignmentsWithTransferLogger;
        LoggerT &overallPerfLogger;
        LoggerT &initializationPerfLogger;
        LoggerT &ellipticBchPerfLogger;
        LoggerT &pdDistancesPerfLogger;
        LoggerT &ordPerfLogger;
        LoggerT &pbnsPerfLogger;
        LoggerT &palsPerfLogger;
        LoggerT &dalsPerfLogger;
        LoggerT &updatePerfLogger;

        LoggerT &ellipseReconstructionPerfLogger;
        LoggerT &ordinaryTransferPerfLogger;
        LoggerT &transferALSPVehPerfLogger;
        LoggerT &transferALSDVehPerfLogger;

        LoggerT &assignmentsCostLogger;

        LoggerT &ordinaryTransferRanksLogger;

    };
}