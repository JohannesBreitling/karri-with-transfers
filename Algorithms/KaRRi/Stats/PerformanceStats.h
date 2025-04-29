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

#include <cstdint>

namespace karri::stats {

    struct InitializationPerformanceStats {
        int64_t findPDLocsInRadiusTime;
        int64_t findVehicleToPdLocsDistancesTime;
        int64_t notUsingVehicleTime;
        int64_t computeODDistanceTime;

        int64_t getTotalTime() const {
            return findPDLocsInRadiusTime + findVehicleToPdLocsDistancesTime + notUsingVehicleTime +
                   computeODDistanceTime;
        }

        void clear() {
            findPDLocsInRadiusTime = 0;
            findVehicleToPdLocsDistancesTime = 0;
            notUsingVehicleTime = 0;
            computeODDistanceTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_initreq.csv";
        static constexpr auto LOGGER_COLS =
                "find_pd_locs_in_radius_time,"
                "find_vehicle_to_pd_locs_distances_time,"
                "not_using_veh_time,"
                "compute_od_distance_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << findPDLocsInRadiusTime << ", "
               << findVehicleToPdLocsDistancesTime << ", "
               << notUsingVehicleTime << ", "
               << computeODDistanceTime << ", "
               << getTotalTime();
            return ss.str();
        }
    };


    struct EllipticBCHPerformanceStats {
        int64_t initializationTime;
        int64_t pickupTime;
        int64_t dropoffTime;
        int64_t pickupNumEdgeRelaxations;
        int64_t pickupNumVerticesSettled;
        int64_t pickupNumEntriesScanned;
        int64_t dropoffNumEdgeRelaxations;
        int64_t dropoffNumVerticesSettled;
        int64_t dropoffNumEntriesScanned;

        int64_t getTotalTime() const {
            return initializationTime + pickupTime + dropoffTime;
        }

        void clear() {
            initializationTime = 0;
            pickupTime = 0;
            dropoffTime = 0;
            pickupNumEdgeRelaxations = 0;
            pickupNumVerticesSettled = 0;
            pickupNumEntriesScanned = 0;
            dropoffNumEdgeRelaxations = 0;
            dropoffNumVerticesSettled = 0;
            dropoffNumEntriesScanned = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_ellipticbch.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "pickup_time,"
                "dropoff_time,"
                "pickup_num_edge_relaxations,"
                "pickup_num_vertices_settled,"
                "pickup_num_entries_scanned,"
                "dropoff_num_edge_relaxations,"
                "dropoff_num_vertices_settled,"
                "dropoff_num_entries_scanned,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << pickupTime << ", "
               << dropoffTime << ", "
               << pickupNumEdgeRelaxations << ", "
               << pickupNumVerticesSettled << ", "
               << pickupNumEntriesScanned << ", "
               << dropoffNumEdgeRelaxations << ", "
               << dropoffNumVerticesSettled << ", "
               << dropoffNumEntriesScanned << ", "
               << getTotalTime();
            return ss.str();
        }
    };

    struct PDDistancesPerformanceStats {
        int64_t initializationTime;
        int64_t dropoffBucketEntryGenTime;
        int64_t pickupBchSearchTime;

        int64_t getTotalTime() const {
            return initializationTime + dropoffBucketEntryGenTime + pickupBchSearchTime;
        }

        void clear() {
            initializationTime = 0;
            dropoffBucketEntryGenTime = 0;
            pickupBchSearchTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_pddistances.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "dropoff_bucket_entry_gen_time,"
                "pickup_bch_search_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << dropoffBucketEntryGenTime << ", "
               << pickupBchSearchTime << ", "
               << getTotalTime();
            return ss.str();
        }
    };

    struct OrdAssignmentsPerformanceStats {
        int64_t initializationTime;

        int64_t numRelevantStopsForPickups;
        int64_t numRelevantStopsForDropoffs;
        int64_t filterRelevantPDLocsTime;

        int64_t numCandidateVehicles;
        int64_t numAssignmentsTried;
        int64_t tryNonPairedAssignmentsTime;
        int64_t tryPairedAssignmentsTime;

        int64_t getTotalTime() const {
            return initializationTime + filterRelevantPDLocsTime + tryNonPairedAssignmentsTime +
                   tryPairedAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;
            numRelevantStopsForPickups = 0;
            numRelevantStopsForDropoffs = 0;
            filterRelevantPDLocsTime = 0;

            numCandidateVehicles = 0;
            numAssignmentsTried = 0;
            tryNonPairedAssignmentsTime = 0;
            tryPairedAssignmentsTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_ord.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_relevant_stops_for_pickups,"
                "num_relevant_stops_for_dropoffs,"
                "filter_relevant_pd_locs_time,"
                "num_candidate_vehicles,"
                "num_assignments_tried,"
                "try_non_paired_assignments_time,"
                "try_paired_assignments_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << numRelevantStopsForPickups << ", "
               << numRelevantStopsForDropoffs << ", "
               << filterRelevantPDLocsTime << ", "
               << numCandidateVehicles << ", "
               << numAssignmentsTried << ", "
               << tryNonPairedAssignmentsTime << ", "
               << tryPairedAssignmentsTime << ", "
               << getTotalTime();
            return ss.str();
        }
    };

    struct PbnsAssignmentsPerformanceStats {
        int64_t initializationTime;

        int64_t numRelevantStopsForPickups;
        int64_t numRelevantStopsForDropoffs;
        int64_t filterRelevantPDLocsTime;

        int64_t locatingVehiclesTime;
        int64_t numCHSearches;
        int64_t directCHSearchTime;

        int64_t numCandidateVehicles;
        int64_t numAssignmentsTried;
        int64_t tryAssignmentsTime;

        int64_t getTotalTime() const {
            return initializationTime + filterRelevantPDLocsTime + tryAssignmentsTime + locatingVehiclesTime;
        }

        void clear() {
            initializationTime = 0;
            numRelevantStopsForPickups = 0;
            numRelevantStopsForDropoffs = 0;
            filterRelevantPDLocsTime = 0;

            locatingVehiclesTime = 0;
            numCHSearches = 0;
            directCHSearchTime = 0;

            numCandidateVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_pbns.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_relevant_stops_for_pickups,"
                "num_relevant_stops_for_dropoffs,"
                "filter_relevant_pd_locs_time,"
                "locating_vehicles_time,"
                "num_ch_searches,"
                "direct_ch_search_time,"
                "num_candidate_vehicles,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << numRelevantStopsForPickups << ", "
               << numRelevantStopsForDropoffs << ", "
               << filterRelevantPDLocsTime << ", "
               << locatingVehiclesTime << ", "
               << numCHSearches << ", "
               << directCHSearchTime << ", "
               << numCandidateVehicles << ", "
               << numAssignmentsTried << ", "
               << tryAssignmentsTime << ", "
               << getTotalTime();
            return ss.str();
        }
    };

    struct AssignmentsWithOrdinaryTransferPerformanceStats {

        // Overall stats
        int64_t totalTime;
        
        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;

        int64_t numCandidateVehiclesDropoffBNS;
        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;
        
        // Stats for the tried assignments
        int64_t numPartialsTriedPickupBNS;
        int64_t numPartialsTriedPickupORD;

        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;

        int64_t numAssignmentsTriedDropoffBNS;
        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;
        
        // Stats for the transfer search itself
        int64_t numStopPairs;
        int64_t numTransferPoints;
        int64_t numDijkstraSearchesRun;
        int64_t numEdgesRelaxed;
        int64_t numVerticesScanned;
        int64_t searchTime;

        int64_t ptAnyToAny_numSearchesRun;
        int64_t ptAnyToAny_sumNumSources;
        int64_t ptAnyToAny_sumNumTargets;
        int64_t ptAnyToAny_numVerticesSettled;
        int64_t ptAnyToAny_numEdgeRelaxations;
        int64_t ptAnyToAny_numResultZero;
        int64_t tdAnyToAny_numSearchesRun;
        int64_t tdAnyToAny_sumNumSources;
        int64_t tdAnyToAny_sumNumTargets;
        int64_t tdAnyToAny_numVerticesSettled;
        int64_t tdAnyToAny_numEdgeRelaxations;
        int64_t tdAnyToAny_numResultZero;

        static constexpr auto LOGGER_NAME = "perf_transf_ord.csv";
        static constexpr auto LOGGER_COLS =
                "total_time,"
                "num_vehs_pickup_bns,"
                "num_vehs_pickup_ord,"
                "num_vehs_dropoff_bns,"
                "num_vehs_dropoff_ord,"
                "num_vehs_dropoff_als,"
                "num_partials_pickup_bns,"
                "num_partials_pickup_ord,"
                "num_assignments_pickup_bns,"
                "num_assignments_pickup_ord,"
                "num_assignments_dropoff_bns,"
                "num_assignments_dropoff_ord,"
                "num_assignments_dropoff_als,"
                "try_assignments_time,"
                "num_stop_pairs,"
                "num_transfer_points,"
                "num_dijkstra_searches,"
                "num_edges_relaxed,"
                "num_vertices_settled,"
                "tp_search_time,"
                "ptAnyToAny_num_searches_run,"
                "ptAnyToAny_sum_num_sources,"
                "ptAnyToAny_sum_num_targets,"
                "ptAnyToAny_num_vertices_settled,"
                "ptAnyToAny_num_edge_relaxations,"
                "ptAnyToAny_num_result_zero,"
                "tdAnyToAny_num_searches_run,"
                "tdAnyToAny_sum_num_sources,"
                "tdAnyToAny_sum_num_targets,"
                "tdAnyToAny_num_vertices_settled,"
                "tdAnyToAny_num_edge_relaxations,"
                "tdAnyToAny_num_result_zero\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << totalTime << ", "
               << numCandidateVehiclesPickupBNS << ", "
               << numCandidateVehiclesPickupORD << ", "
               << numCandidateVehiclesDropoffBNS << ", "
               << numCandidateVehiclesDropoffORD << ", "
               << numCandidateVehiclesDropoffALS << ", "
               << numPartialsTriedPickupBNS << ", "
               << numPartialsTriedPickupORD << ", "
               << numAssignmentsTriedPickupBNS << ", "
               << numAssignmentsTriedPickupORD << ", "
               << numAssignmentsTriedDropoffBNS << ", "
               << numAssignmentsTriedDropoffORD << ", "
               << numAssignmentsTriedDropoffALS << ", "
               << tryAssignmentsTime << ", "
               << numStopPairs << ", "
               << numTransferPoints << ", "
               << numDijkstraSearchesRun << ", "
               << numEdgesRelaxed << ", "
               << numVerticesScanned << ", "
               << searchTime << ", "
               << ptAnyToAny_numSearchesRun << ", "
                << ptAnyToAny_sumNumSources << ", "
                << ptAnyToAny_sumNumTargets << ", "
                << ptAnyToAny_numVerticesSettled << ", "
                << ptAnyToAny_numEdgeRelaxations << ", "
                << ptAnyToAny_numResultZero << ", "
                << tdAnyToAny_numSearchesRun << ", "
                << tdAnyToAny_sumNumSources << ", "
                << tdAnyToAny_sumNumTargets << ", "
                << tdAnyToAny_numVerticesSettled << ", "
                << tdAnyToAny_numEdgeRelaxations << ", "
                << tdAnyToAny_numResultZero;
            return ss.str();
        }
        
        int64_t getTotalTime() const {
            return totalTime;
        }

        void clear() {
            totalTime = 0;
            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesDropoffBNS = 0;
            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;
            numPartialsTriedPickupBNS = 0;
            numPartialsTriedPickupORD = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffBNS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;
            numStopPairs = 0;
            numTransferPoints = 0;
            numDijkstraSearchesRun = 0;
            numEdgesRelaxed = 0;
            numVerticesScanned = 0;
            searchTime = 0;
            ptAnyToAny_numSearchesRun = 0;
            ptAnyToAny_sumNumSources = 0;
            ptAnyToAny_sumNumTargets = 0;
            ptAnyToAny_numVerticesSettled = 0;
            ptAnyToAny_numEdgeRelaxations = 0;
            ptAnyToAny_numResultZero = 0;
            tdAnyToAny_numSearchesRun = 0;
            tdAnyToAny_sumNumSources = 0;
            tdAnyToAny_sumNumTargets = 0;
            tdAnyToAny_numVerticesSettled = 0;
            tdAnyToAny_numEdgeRelaxations = 0;
            tdAnyToAny_numResultZero = 0;
        }
    };

    struct AssignmentsWithTransferALSPVehPerformanceStats {

        // Overall stats
        int64_t totalTime;
        
        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;
        int64_t numCandidateVehiclesPickupALS;

        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;
        
        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;
        int64_t numAssignmentsTriedPickupALS;

        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;
        
        // Stats for the transfer search itself
        int64_t numTransferPoints;
        
        // Search from last stop to all stops
        int64_t numSearchesRunLastStopToDVeh;
        int64_t numEdgesRelaxedLastStopToDVeh;
        int64_t numVerticesScannedLastStopToDVeh;
        int64_t searchTimeLastStopToDVeh;
        
        // Search from pickup to all stops
        int64_t numSearchesRunPickupToDVeh;
        int64_t numEdgesRelaxedPickupToDVeh;
        int64_t numVerticesScannedPickupToDVeh;
        int64_t searchTimePickupToDVeh;
        
        static constexpr auto LOGGER_NAME = "perf_transf_als_pveh.csv";
        static constexpr auto LOGGER_COLS =
                "total_time,"
                "num_vehs_pickup_bns,"
                "num_vehs_pickup_ord,"
                "num_vehs_pickup_als,"
                "num_vehs_dropoff_ord,"
                "num_vehs_dropoff_als,"
                "num_assignments_pickup_bns,"
                "num_assignments_pickup_ord,"
                "num_assignments_pickup_als,"
                "num_assignments_dropoff_ord,"
                "num_assignments_dropoff_als,"
                "try_assignments_time,"
                "num_transfer_points,"
                "num_searches_last_stop_to_dveh,"
                "num_edges_relaxed_last_stop_to_dveh,"
                "num_vertices_settled_last_stop_to_dveh,"
                "search_time_last_stop_to_dveh"
                "num_searches_pickup_to_dveh,"
                "num_edges_relaxed_pickup_to_dveh,"
                "num_vertices_settled_pickup_to_dveh,"
                "search_time_pickup_to_dveh\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << totalTime << ", "
               << numCandidateVehiclesPickupBNS << ", "
               << numCandidateVehiclesPickupORD << ", "
               << numCandidateVehiclesPickupALS << ", "
               << numCandidateVehiclesDropoffORD << ", "
               << numCandidateVehiclesDropoffALS << ", "
               << numAssignmentsTriedPickupBNS << ", "
               << numAssignmentsTriedPickupORD << ", "
               << numAssignmentsTriedPickupALS << ", "
               << numAssignmentsTriedDropoffORD << ", "
               << numAssignmentsTriedDropoffALS << ", "
               << tryAssignmentsTime << ", "
               << numTransferPoints << ", "
               << numSearchesRunLastStopToDVeh << ", "
               << numEdgesRelaxedLastStopToDVeh << ", "
               << numVerticesScannedLastStopToDVeh << ", "
               << searchTimeLastStopToDVeh << ", "
               << numSearchesRunLastStopToDVeh << ", "
               << numEdgesRelaxedPickupToDVeh << ", "
               << numVerticesScannedPickupToDVeh << ", "
               << searchTimePickupToDVeh;
            return ss.str();
        }

        int64_t getTotalTime() const {
            return totalTime;
        }

        void clear() {
            totalTime = 0;
            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesPickupALS = 0;
            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedPickupALS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;
            numTransferPoints = 0;
            numSearchesRunLastStopToDVeh = 0;
            numEdgesRelaxedLastStopToDVeh = 0;
            numVerticesScannedLastStopToDVeh = 0;
            searchTimeLastStopToDVeh = 0;
            numSearchesRunPickupToDVeh = 0;
            numEdgesRelaxedPickupToDVeh = 0;
            numVerticesScannedPickupToDVeh = 0;
            searchTimePickupToDVeh = 0;
        }
    };

    struct AssignmentsWithTransferALSDVehPerformanceStats {
        
        // Overall stats
        int64_t totalTime;
        
        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;

        int64_t numCandidateVehiclesDropoffALS;
        
        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;
        
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;
        
        // Stats for the transfer search itself
        int64_t numTransferPoints;
        
        // Search from last stop to all stops
        int64_t numSearchesRunLastStopToPVeh;
        int64_t numEdgesRelaxedLastStopToPVeh;
        int64_t numVerticesScannedLastStopToPVeh;
        int64_t searchTimeLastStopToPVeh;

        static constexpr auto LOGGER_NAME = "perf_transf_als_dveh.csv";
        static constexpr auto LOGGER_COLS =
                "total_time,"
                "num_vehs_pickup_bns,"
                "num_vehs_pickup_ord,"
                "num_vehs_dropoff_als,"
                "num_assignments_pickup_bns,"
                "num_assignments_pickup_ord,"
                "num_assignments_dropoff_als,"
                "try_assignments_time,"
                "num_transfer_points,"
                "num_searches_last_stop_to_pveh,"
                "num_edges_relaxed_last_stop_to_pveh,"
                "num_vertices_settled_last_stop_to_pveh,"
                "search_time_last_stop_to_pveh\n";

        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << totalTime << ", "
               << numCandidateVehiclesPickupBNS << ", "
               << numCandidateVehiclesPickupORD << ", "
               << numCandidateVehiclesDropoffALS << ", "
               << numAssignmentsTriedPickupBNS << ", "
               << numAssignmentsTriedPickupORD << ", "
               << numAssignmentsTriedDropoffALS << ", "
               << tryAssignmentsTime << ", "
               << numTransferPoints << ", "
               << numSearchesRunLastStopToPVeh << ", "
               << numEdgesRelaxedLastStopToPVeh << ", "
               << numVerticesScannedLastStopToPVeh << ", "
               << searchTimeLastStopToPVeh;
            return ss.str();
        }

        int64_t getTotalTime() const {
            return totalTime;
        }

        void clear() {
            totalTime = 0;
            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesDropoffALS = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;
            numTransferPoints = 0;
            numSearchesRunLastStopToPVeh = 0;
            numEdgesRelaxedLastStopToPVeh = 0;
            numVerticesScannedLastStopToPVeh = 0;
            searchTimeLastStopToPVeh = 0;
        }

    };

    struct AssignmentCostStats {
        int64_t totalWOT;
        int64_t walkingCostWOT;
        int64_t tripCostWOT;
        int64_t waitTimeViolationCostWOT;
        int64_t changeInTripCostsOfOthersWOT;
        int64_t vehCostWOT;

        int64_t totalWT;
        int64_t walkingCostWT;
        int64_t tripCostWT;
        int64_t waitTimeViolationCostWT;
        int64_t changeInTripCostsOfOthersWT;
        int64_t vehCostWT;

        bool transferImproves;
        bool inftyWOT;
        bool inftyWT;

        void clear() {
            totalWOT = 0;
            walkingCostWOT = 0;
            tripCostWOT = 0;
            waitTimeViolationCostWOT = 0;
            changeInTripCostsOfOthersWOT = 0;
            vehCostWOT = 0;

            totalWT = 0;
            walkingCostWT = 0;
            tripCostWT = 0;
            waitTimeViolationCostWT = 0;
            changeInTripCostsOfOthersWT = 0;
            vehCostWT = 0;

            transferImproves = false;
            inftyWOT = true;
            inftyWT = true;
        }

        static constexpr auto LOGGER_NAME = "assignmentcost.csv";
        static constexpr auto LOGGER_COLS =
                "total_no_transfer,"
                "walking_cost_no_transfer,"
                "trip_cost_no_transfer,"
                "wait_time_violation_cost_no_transfer,"
                "change_in_trip_costs_of_others_no_transfer,"
                "veh_cost_no_transfer,"
                "total_transfer,"
                "walking_cost_transfer,"
                "trip_cost_transfer,"
                "wait_time_violation_cost_transfer,"
                "change_in_trip_costs_of_others_transfer,"
                "veh_cost_transfer,"
                "infty_no_transfer,"
                "infty_transfer,"
                "transfer_improves\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << totalWOT << ", "
               << walkingCostWOT << ", "
               << tripCostWOT << ", "
               << waitTimeViolationCostWOT << ", "
               << changeInTripCostsOfOthersWOT << ", "
               << vehCostWOT << ", "
               << totalWT << ", "
               << walkingCostWT << ", "
               << tripCostWT << ", "
               << waitTimeViolationCostWT << ", "
               << changeInTripCostsOfOthersWT << ", "
               << vehCostWT << ", "
               << inftyWOT << ", "
               << inftyWT << ", "
               << transferImproves;
            return ss.str();
        }
    };





    struct PalsAssignmentsPerformanceStats {

        int64_t initializationTime;

        int64_t numEdgeRelaxationsInSearchGraph;
        int64_t numVerticesOrLabelsSettled;
        int64_t numEntriesOrLastStopsScanned;
        int64_t searchTime;

        int64_t numCandidateVehicles;
        int64_t numAssignmentsTried;
        int64_t tryAssignmentsTime;

        // Stats about pickup coinciding with last stop (same independent of PALS strategy):
        int64_t pickupAtLastStop_numCandidateVehicles;
        int64_t pickupAtLastStop_numAssignmentsTried;
        int64_t pickupAtLastStop_tryAssignmentsTime;

        // Stats only relevant for collective PALS strategy:
        int64_t collective_pickupVehDistQueryTime;
        int64_t collective_numPromisingDropoffs;
        int64_t collective_numInitialLabelsGenerated;
        int64_t collective_numInitialLabelsNotPruned;
        int64_t collective_initializationTime;
        int64_t collective_numDominationRelationTests;
        bool collective_usedFallback;

        int64_t getTotalTime() const {
            return initializationTime + searchTime + tryAssignmentsTime + pickupAtLastStop_tryAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;
            numEdgeRelaxationsInSearchGraph = 0;
            numVerticesOrLabelsSettled = 0;
            numEntriesOrLastStopsScanned = 0;
            searchTime = 0;
            numCandidateVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
            pickupAtLastStop_numCandidateVehicles = 0;
            pickupAtLastStop_numAssignmentsTried = 0;
            pickupAtLastStop_tryAssignmentsTime = 0;
            collective_pickupVehDistQueryTime = 0;
            collective_numPromisingDropoffs = 0;
            collective_numInitialLabelsGenerated = 0;
            collective_numInitialLabelsNotPruned = 0;
            collective_initializationTime = 0;
            collective_numDominationRelationTests = 0;
            collective_usedFallback = false;
        }

        static constexpr auto LOGGER_NAME = "perf_pals.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_edge_relaxations,"
                "num_vertices_or_labels_settled,"
                "num_entries_or_last_stops_scanned,"
                "search_time,"
                "num_candidate_vehicles,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "pickup_at_last_stop.num_candidate_vehicles,"
                "pickup_at_last_stop.num_assignments_tried,"
                "pickup_at_last_stop.try_assignments_time,"
                "collective.pickup_veh_dist_query_time,"
                "collective.num_promising_dropoffs,"
                "collective.num_initial_labels_generated,"
                "collective.num_initial_labels_not_pruned,"
                "collective.initialization_time,"
                "collective.num_domination_relation_tests,"
                "collective.used_fallback,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << numEdgeRelaxationsInSearchGraph << ", "
               << numVerticesOrLabelsSettled << ", "
               << numEntriesOrLastStopsScanned << ", "
               << searchTime << ", "
               << numCandidateVehicles << ", "
               << numAssignmentsTried << ", "
               << tryAssignmentsTime << ", "
               << pickupAtLastStop_numCandidateVehicles << ", "
               << pickupAtLastStop_numAssignmentsTried << ", "
               << pickupAtLastStop_tryAssignmentsTime << ", "
               << collective_pickupVehDistQueryTime << ", "
               << collective_numPromisingDropoffs << ", "
               << collective_numInitialLabelsGenerated << ", "
               << collective_numInitialLabelsNotPruned << ", "
               << collective_initializationTime << ", "
               << collective_numDominationRelationTests << ", "
               << collective_usedFallback << ", "
               << getTotalTime();
            return ss.str();
        }


    };

    struct DalsAssignmentsPerformanceStats {

        int64_t initializationTime;

        int64_t numEdgeRelaxationsInSearchGraph;
        int64_t numVerticesOrLabelsSettled;
        int64_t numEntriesOrLastStopsScanned;
        int64_t searchTime;

        int64_t numCandidateVehicles;
        int64_t numCandidateDropoffsAcrossAllVehicles;
        int64_t numAssignmentsTried;
        int64_t tryAssignmentsTime;

        // Stats only relevant for collective DALS strategy:
        int64_t collective_numDominationRelationTests;
        bool collective_ranClosestDropoffSearch;
        int64_t collective_numDirectCHSearches;
        int64_t collective_initializationTime;

        int64_t getTotalTime() const {
            return initializationTime + searchTime + tryAssignmentsTime;
        }

        void clear() {
            initializationTime = 0;
            numEdgeRelaxationsInSearchGraph = 0;
            numVerticesOrLabelsSettled = 0;
            numEntriesOrLastStopsScanned = 0;
            searchTime = 0;
            numCandidateVehicles = 0;
            numCandidateDropoffsAcrossAllVehicles = 0;
            numAssignmentsTried = 0;
            tryAssignmentsTime = 0;
            collective_numDominationRelationTests = 0;
            collective_ranClosestDropoffSearch = false;
            collective_numDirectCHSearches = 0;
            collective_initializationTime = 0;
        }


        static constexpr auto LOGGER_NAME = "perf_dals.csv";
        static constexpr auto LOGGER_COLS =
                "initialization_time,"
                "num_edge_relaxations,"
                "num_vertices_or_labels_settled,"
                "num_entries_or_last_stops_scanned,"
                "search_time,"
                "num_candidate_vehicles,"
                "num_candidate_dropoffs,"
                "num_assignments_tried,"
                "try_assignments_time,"
                "collective.num_domination_relation_tests,"
                "collective.ran_closest_dropoff_search,"
                "collective.num_direct_ch_searches,"
                "collective.initialization_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << initializationTime << ", "
               << numEdgeRelaxationsInSearchGraph << ", "
               << numVerticesOrLabelsSettled << ", "
               << numEntriesOrLastStopsScanned << ", "
               << searchTime << ", "
               << numCandidateVehicles << ", "
               << numCandidateDropoffsAcrossAllVehicles << ", "
               << numAssignmentsTried << ", "
               << tryAssignmentsTime << ", "
               << collective_numDominationRelationTests << ", "
               << collective_ranClosestDropoffSearch << ", "
               << collective_numDirectCHSearches << ", "
               << collective_initializationTime << ", "
               << getTotalTime();
            return ss.str();
        }

    };

    struct UpdatePerformanceStats {

        int64_t elliptic_generate_numVerticesInSearchSpace;
        int64_t elliptic_generate_numEntriesInserted;
        int64_t elliptic_generate_time;

        int64_t elliptic_update_numVerticesVisited;
        int64_t elliptic_update_numEntriesScanned;
        int64_t elliptic_update_time;

        int64_t elliptic_delete_numVerticesVisited;
        int64_t elliptic_delete_numEntriesScanned;
        int64_t elliptic_delete_time;

        int64_t lastStopBucketsGenerateEntriesTime;
        int64_t lastStopBucketsUpdateEntriesTime;
        int64_t lastStopBucketsDeleteEntriesTime;

        int64_t lastStopsAtVerticesUpdateTime;

        int64_t updateRoutesTime;

        int64_t getTotalTime() const {
            return elliptic_generate_time + elliptic_update_time +
                   elliptic_delete_time + lastStopBucketsGenerateEntriesTime +
                   lastStopBucketsDeleteEntriesTime + lastStopsAtVerticesUpdateTime + updateRoutesTime;
        }

        void clear() {
            elliptic_generate_numVerticesInSearchSpace = 0;
            elliptic_generate_numEntriesInserted = 0;
            elliptic_generate_time = 0;
            elliptic_update_numVerticesVisited = 0;
            elliptic_update_numEntriesScanned = 0;
            elliptic_update_time = 0;
            elliptic_delete_numVerticesVisited = 0;
            elliptic_delete_numEntriesScanned = 0;
            elliptic_delete_time = 0;
            lastStopBucketsGenerateEntriesTime = 0;
            lastStopBucketsDeleteEntriesTime = 0;
            lastStopsAtVerticesUpdateTime = 0;
            updateRoutesTime = 0;
        }

        static constexpr auto LOGGER_NAME = "perf_update.csv";
        static constexpr auto LOGGER_COLS =
                "elliptic.generate.numVerticesInSearchSpace,"
                "elliptic.generate.numEntriesInserted,"
                "elliptic.generate.time,"
                "elliptic.update.numVerticesVisited,"
                "elliptic.update.numEntriesScanned,"
                "elliptic.update.time,"
                "elliptic.delete.numVerticesVisited,"
                "elliptic.delete.numEntriesScanned,"
                "elliptic.delete.time,"
                "last_stop_buckets_generate_entries_time,"
                "last_stop_buckets_delete_entries_time,"
                "last_stop_at_vertices_update_time,"
                "update_routes_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << elliptic_generate_numVerticesInSearchSpace << ", "
               << elliptic_generate_numEntriesInserted << ", "
               << elliptic_generate_time << ", "
               << elliptic_update_numVerticesVisited << ", "
               << elliptic_update_numEntriesScanned << ", "
               << elliptic_update_time << ", "
               << elliptic_delete_numVerticesVisited << ", "
               << elliptic_delete_numEntriesScanned << ", "
               << elliptic_delete_time << ", "
               << lastStopBucketsGenerateEntriesTime << ", "
               << lastStopBucketsDeleteEntriesTime << ", "
               << lastStopsAtVerticesUpdateTime << ", "
               << updateRoutesTime << ", "
               << getTotalTime();
            return ss.str();
        }

    };

    struct DispatchingPerformanceStats {

        int32_t numPickups;
        int32_t numDropoffs;

        InitializationPerformanceStats initializationStats;
        EllipticBCHPerformanceStats ellipticBchStats;
        PDDistancesPerformanceStats pdDistancesStats;
        OrdAssignmentsPerformanceStats ordAssignmentsStats;
        PbnsAssignmentsPerformanceStats pbnsAssignmentsStats;
        PalsAssignmentsPerformanceStats palsAssignmentsStats;
        DalsAssignmentsPerformanceStats dalsAssignmentsStats;
        UpdatePerformanceStats updateStats;

        AssignmentsWithOrdinaryTransferPerformanceStats ordinaryTransferStats;
        AssignmentsWithTransferALSPVehPerformanceStats transferALSPVehStats;
        AssignmentsWithTransferALSDVehPerformanceStats transferALSDVehStats;
        AssignmentCostStats costStats;

        int64_t getTotalTime() const {
            return initializationStats.getTotalTime() +
                   ellipticBchStats.getTotalTime() +
                   pdDistancesStats.getTotalTime() +
                   ordAssignmentsStats.getTotalTime() +
                   pbnsAssignmentsStats.getTotalTime() +
                   palsAssignmentsStats.getTotalTime() +
                   dalsAssignmentsStats.getTotalTime() +
                   updateStats.getTotalTime();
        }

        void clear() {
            numPickups = 0;
            numDropoffs = 0;
            initializationStats.clear();
            ellipticBchStats.clear();
            pdDistancesStats.clear();
            ordAssignmentsStats.clear();
            pbnsAssignmentsStats.clear();
            palsAssignmentsStats.clear();
            dalsAssignmentsStats.clear();
            updateStats.clear();
            ordinaryTransferStats.clear();
            transferALSPVehStats.clear();
            transferALSDVehStats.clear();
            costStats.clear();
        }

        static constexpr auto LOGGER_NAME = "perf_overall.csv";
        static constexpr auto LOGGER_COLS =
                "num_pickups,"
                "num_dropoffs,"
                "initialization_time,"
                "elliptic_bch_time,"
                "pd_distances_time,"
                "ord_assignments_time,"
                "pbns_assignments_time,"
                "pals_assignments_time,"
                "dals_assignments_time,"
                "transf_ord_time,"
                "transf_als_pveh_time,"
                "transf_als_dveh_time,"
                "update_time,"
                "total_time\n";


        std::string getLoggerRow() const {
            std::stringstream ss;
            ss << numPickups << ", "
               << numDropoffs << ", "
               << initializationStats.getTotalTime() << ", "
               << ellipticBchStats.getTotalTime() << ", "
               << pdDistancesStats.getTotalTime() << ", "
               << ordAssignmentsStats.getTotalTime() << ", "
               << pbnsAssignmentsStats.getTotalTime() << ", "
               << palsAssignmentsStats.getTotalTime() << ", "
               << dalsAssignmentsStats.getTotalTime() << ", "
               << ordinaryTransferStats.getTotalTime() << ", "
               << transferALSPVehStats.getTotalTime() << ", "
               << transferALSDVehStats.getTotalTime() << ", "
               << updateStats.getTotalTime() << ", "
               << getTotalTime();
            return ss.str();
        }

    };

}