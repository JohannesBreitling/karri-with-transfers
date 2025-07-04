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


#include <cassert>
#include <kassert/kassert.hpp>
#include "Tools/custom_assertion_levels.h"
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <csv.h>

#include "Tools/CommandLine/CommandLineParser.h"
#include "Tools/Logging/LogManager.h"
#include "DataStructures/Graph/Graph.h"
#include "DataStructures/Graph/Attributes/LatLngAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeIdAttribute.h"
#include "DataStructures/Graph/Attributes/OsmNodeIdAttribute.h"
#include "DataStructures/Graph/Attributes/FreeFlowSpeedAttribute.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "DataStructures/Graph/Attributes/PsgEdgeToCarEdgeAttribute.h"
#include "DataStructures/Graph/Attributes/CarEdgeToPsgEdgeAttribute.h"

#include "Algorithms/KaRRi/InputConfig.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/PbnsAssignments/VehicleLocator.h"
#include "Algorithms/KaRRi/EllipticBCH/FeasibleEllipticDistances.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBucketsEnvironment.h"
#include "Algorithms/KaRRi/EllipticBCH/EllipticBCHSearches.h"
#include "Algorithms/KaRRi/CostCalculator.h"

#include "Algorithms/KaRRi/TransferPoints/InsertionAsserter.h"

#include "Algorithms/KaRRi/PHAST/RPHASTEnvironment.h"

#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/PDDistanceQueries/PDDistances.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocsFilter.h"
#include "Algorithms/KaRRi/OrdinaryAssignments/OrdinaryAssignmentsFinder.h"
#include "Algorithms/KaRRi/PbnsAssignments/PBNSAssignmentsFinder.h"
#include "Algorithms/KaRRi/PalsAssignments/PALSAssignmentsFinder.h"
#include "Algorithms/KaRRi/DalsAssignments/DALSAssignmentsFinder.h"

#include "Algorithms/KaRRi/TransferPoints/AssignmentsWithTransferFinder.h"
#include "Algorithms/KaRRi/TransferPoints/PHASTEllipseReconstructor.h"
#include "Algorithms/KaRRi/TransferPoints/DijkstraEllipseReconstructor.h"

#include "Algorithms/KaRRi/TransferPoints/EdgeEllipseIntersector.h"
#include "Algorithms/KaRRi/TransferPoints/PickupALS/TransfersPickupALSBCHStrategy.h"
#include "Algorithms/KaRRi/TransferPoints/DropoffALS/TransfersDropoffALSBCHStrategy.h"
#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/OrdinaryTransferFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/PHASTStrategyALS.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSPVeh/TransferALSPVehFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSDVeh/TransferALSDVehFinder.h"

#include "Algorithms/KaRRi/LastStopSearches/SortedLastStopBucketsEnvironment.h"
#include "Algorithms/KaRRi/LastStopSearches/UnsortedLastStopBucketsEnvironment.h"
#include "Algorithms/KaRRi/RequestState/VehicleToPDLocQuery.h"
#include "Algorithms/KaRRi/RequestState/RequestStateInitializer.h"
#include "Algorithms/KaRRi/AssignmentFinder.h"
#include "Algorithms/KaRRi/SystemStateUpdater.h"
#include "Algorithms/KaRRi/EventSimulation.h"

#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/DirectTransferDistances/BCHDirectTransferDistancesFinder.h"
#include "Parallel/hardware_topology.h"
#include "Parallel/tbb_initializer.h"
#include "Algorithms/KaRRi/TransferPoints/OnlyAtStopEllipseReconstructor.h"

#ifdef KARRI_USE_CCHS

#include "Algorithms/KaRRi/CCHEnvironment.h"

#else

#include "Algorithms/KaRRi/CHEnvironment.h"

#endif

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT

#include "Algorithms/KaRRi/PDDistanceQueries/BCHStrategy.h"

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
#include "Algorithms/KaRRi/PDDistanceQueries/CHStrategy.h"
#endif


#if KARRI_PALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/PalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_PALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/PalsAssignments/IndividualBCHStrategy.h"

#else // KARRI_PALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/PalsAssignments/DijkstraStrategy.h"

#endif

#if KARRI_DALS_STRATEGY == KARRI_COL

#include "Algorithms/KaRRi/DalsAssignments/CollectiveBCHStrategy.h"

#elif KARRI_DALS_STRATEGY == KARRI_IND

#include "Algorithms/KaRRi/DalsAssignments//IndividualBCHStrategy.h"

#else // KARRI_DALS_STRATEGY == KARRI_DIJ

#include "Algorithms/KaRRi/DalsAssignments/DijkstraStrategy.h"

#endif


inline void printUsage() {
    std::cout <<
              "Usage: karri -veh-g <vehicle network> -psg-g <passenger network> -r <requests> -v <vehicles> -o <file>\n"
              "Runs Karlsruhe Rapid Ridesharing (KaRRi) simulation with given vehicle and passenger road networks,\n"
              "requests, and vehicles. Writes output files to specified base path."
              "  -veh-g <file>          vehicle road network in binary format.\n"
              "  -psg-g <file>          passenger road (and path) network in binary format.\n"
              "  -r <file>              requests in CSV format.\n"
              "  -v <file>              vehicles in CSV format.\n"
              "  -s <sec>               stop time (in s). (dflt: 60s)\n"
              "  -w <sec>               maximum wait time (in s). (dflt: 300s)\n"
              "  -a <factor>            model parameter alpha for max trip time = a * OD-dist + b (dflt: 1.7)\n"
              "  -b <seconds>           model parameter beta for max trip time = a * OD-dist + b (dflt: 120)\n"
              "  -p-radius <sec>        walking radius (in s) for pickup locations around origin. (dflt: 300s)\n"
              "  -d-radius <sec>        walking radius (in s) for dropoff locations around destination. (dflt: 300s)\n"
              "  -max-num-p <int>       max number of pickup locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
              "  -max-num-d <int>       max number of dropoff locations to consider, sampled from all in radius. Set to 0 for no limit (dflt).\n"
              "  -always-veh            if set, the rider is not allowed to walk to their destination without a vehicle trip.\n"
              "  -veh-h <file>          contraction hierarchy for the vehicle network in binary format.\n"
              "  -psg-h <file>          contraction hierarchy for the passenger network in binary format.\n"
              "  -veh-d <file>          separator decomposition for the vehicle network in binary format (needed for CCHs).\n"
              "  -psg-d <file>          separator decomposition for the passenger network in binary format (needed for CCHs).\n"
              "  -csv-in-LOUD-format    if set, assumes that input files are in the format used by LOUD.\n"
              "  -max-num-threads <int> set the maximum number of threads to use (dflt: 1).\n"
              "  -o <file>              generate output files at name <file> (specify name without file suffix).\n"
              "  -help                  show usage help text.\n";
}

int main(int argc, char *argv[]) {
    using namespace karri;
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Initialize TBB
        auto maxNumThreads = clp.getValue<size_t>("max-num-threads", 1);
        size_t numAvailableCpus = parallel::HardwareTopology<>::instance().num_cpus();
        if (numAvailableCpus < maxNumThreads) {
            std::cout << "There are currently only " << numAvailableCpus << " cpus available. "
                      << "Setting number of threads from " << maxNumThreads << " to " << numAvailableCpus << std::endl;
            maxNumThreads = numAvailableCpus;
        }
        parallel::TBBInitializer<parallel::HardwareTopology<>>::instance(maxNumThreads);


        // Parse the command-line options.
        InputConfig &inputConfig = InputConfig::getInstance();
        inputConfig.stopTime = clp.getValue<int>("s", 60) * 10;
        inputConfig.maxWaitTime = clp.getValue<int>("w", 300) * 10;
        inputConfig.pickupRadius = clp.getValue<int>("p-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.dropoffRadius = clp.getValue<int>("d-radius", inputConfig.maxWaitTime / 10) * 10;
        inputConfig.maxNumPickups = clp.getValue<int>("max-num-p", INFTY);
        inputConfig.maxNumDropoffs = clp.getValue<int>("max-num-d", INFTY);
        inputConfig.alwaysUseVehicle = clp.isSet("always-veh");
        if (inputConfig.maxNumPickups == 0) inputConfig.maxNumPickups = INFTY;
        if (inputConfig.maxNumDropoffs == 0) inputConfig.maxNumDropoffs = INFTY;
        inputConfig.alpha = clp.getValue<double>("a", 1.7);
        inputConfig.includeTransfers = clp.getValue<int>("trans", 1);
        inputConfig.beta = clp.getValue<int>("b", 120) * 10;
        const auto vehicleNetworkFileName = clp.getValue<std::string>("veh-g");
        const auto passengerNetworkFileName = clp.getValue<std::string>("psg-g");
        const auto vehicleFileName = clp.getValue<std::string>("v");
        const auto requestFileName = clp.getValue<std::string>("r");
        const auto vehHierarchyFileName = clp.getValue<std::string>("veh-h");
        const auto psgHierarchyFileName = clp.getValue<std::string>("psg-h");
        const auto vehSepDecompFileName = clp.getValue<std::string>("veh-d");
        const auto psgSepDecompFileName = clp.getValue<std::string>("psg-d");
        const bool csvFilesInLoudFormat = clp.isSet("csv-in-LOUD-format");
        auto outputFileName = clp.getValue<std::string>("o");
        if (endsWith(outputFileName, ".csv"))
            outputFileName = outputFileName.substr(0, outputFileName.size() - 4);


        LogManager<std::ofstream>::setBaseFileName(outputFileName + ".");

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using VehicleVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using VehicleEdgeAttributes = EdgeAttrs<
                EdgeIdAttribute, EdgeTailAttribute, FreeFlowSpeedAttribute, TravelTimeAttribute, CarEdgeToPsgEdgeAttribute, OsmRoadCategoryAttribute>;
        using VehicleInputGraph = StaticGraph<VehicleVertexAttributes, VehicleEdgeAttributes>;
        std::ifstream vehicleNetworkFile(vehicleNetworkFileName, std::ios::binary);
        if (!vehicleNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + vehicleNetworkFileName + "'");
        VehicleInputGraph vehicleInputGraph(vehicleNetworkFile);
        vehicleNetworkFile.close();
        std::vector<int32_t> vehGraphOrigIdToSeqId;
        if (vehicleInputGraph.numEdges() > 0 && vehicleInputGraph.edgeId(0) == INVALID_ID) {
            vehGraphOrigIdToSeqId.assign(vehicleInputGraph.numEdges(), INVALID_ID);
            std::iota(vehGraphOrigIdToSeqId.begin(), vehGraphOrigIdToSeqId.end(), 0);
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    assert(vehicleInputGraph.edgeId(e) == INVALID_ID);
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        } else {
            FORALL_VALID_EDGES(vehicleInputGraph, v, e) {
                    assert(vehicleInputGraph.edgeId(e) != INVALID_ID);
                    if (vehicleInputGraph.edgeId(e) >= vehGraphOrigIdToSeqId.size()) {
                        const auto numElementsToBeInserted =
                                vehicleInputGraph.edgeId(e) + 1 - vehGraphOrigIdToSeqId.size();
                        vehGraphOrigIdToSeqId.insert(vehGraphOrigIdToSeqId.end(), numElementsToBeInserted, INVALID_ID);
                    }
                    assert(vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] == INVALID_ID);
                    vehGraphOrigIdToSeqId[vehicleInputGraph.edgeId(e)] = e;
                    vehicleInputGraph.edgeTail(e) = v;
                    vehicleInputGraph.edgeId(e) = e;
                }
        }
        std::cout << "done.\n";

        // Read the passenger network from file.
        std::cout << "Reading passenger network from file... " << std::flush;
        using PsgVertexAttributes = VertexAttrs<LatLngAttribute, OsmNodeIdAttribute>;
        using PsgEdgeAttributes = EdgeAttrs<EdgeIdAttribute, EdgeTailAttribute, PsgEdgeToCarEdgeAttribute, TravelTimeAttribute>;
        using PsgInputGraph = StaticGraph<PsgVertexAttributes, PsgEdgeAttributes>;
        std::ifstream psgNetworkFile(passengerNetworkFileName, std::ios::binary);
        if (!psgNetworkFile.good())
            throw std::invalid_argument("file not found -- '" + passengerNetworkFileName + "'");
        PsgInputGraph psgInputGraph(psgNetworkFile);
        psgNetworkFile.close();
        assert(psgInputGraph.numEdges() > 0 && psgInputGraph.edgeId(0) == INVALID_ID);
        int numEdgesWithMappingToCar = 0;
        FORALL_VALID_EDGES(psgInputGraph, v, e) {
                assert(psgInputGraph.edgeId(e) == INVALID_ID);
                psgInputGraph.edgeTail(e) = v;
                psgInputGraph.edgeId(e) = e;

                const int eInVehGraph = psgInputGraph.toCarEdge(e);
                if (eInVehGraph != PsgEdgeToCarEdgeAttribute::defaultValue()) {
                    ++numEdgesWithMappingToCar;
                    assert(eInVehGraph < vehGraphOrigIdToSeqId.size());
                    psgInputGraph.toCarEdge(e) = vehGraphOrigIdToSeqId[eInVehGraph];
                    assert(psgInputGraph.toCarEdge(e) < vehicleInputGraph.numEdges());
                    vehicleInputGraph.toPsgEdge(psgInputGraph.toCarEdge(e)) = e;

                    assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).latitude() ==
                           vehicleInputGraph.latLng(vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).latitude());
                    assert(psgInputGraph.latLng(psgInputGraph.edgeHead(e)).longitude() == vehicleInputGraph.latLng(
                            vehicleInputGraph.edgeHead(psgInputGraph.toCarEdge(e))).longitude());
                }
            }
        unused(numEdgesWithMappingToCar);
        assert(numEdgesWithMappingToCar > 0);
        std::cout << "done.\n";


        // Read the vehicle data from file.
        std::cout << "Reading vehicle data from file... " << std::flush;
        Fleet fleet;
        int location, capacity, startOfServiceTime, endOfServiceTime;
        io::CSVReader<4, io::trim_chars<' '>> vehiclesFileReader(vehicleFileName);

        if (csvFilesInLoudFormat) {
            vehiclesFileReader.read_header(io::ignore_no_column, "initial_location", "seating_capacity",
                                           "start_service_time", "end_service_time");
        } else {
            vehiclesFileReader.read_header(io::ignore_no_column,
                                           "initial_location", "start_of_service_time",
                                           "end_of_service_time", "capacity");
        }

        int maxCapacity = 0;
        while ((csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, capacity, startOfServiceTime, endOfServiceTime)) ||
               (!csvFilesInLoudFormat &&
                vehiclesFileReader.read_row(location, startOfServiceTime, endOfServiceTime, capacity))) {
            if (location < 0 || location >= vehGraphOrigIdToSeqId.size() ||
                vehGraphOrigIdToSeqId[location] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(location) + "'");
            if (endOfServiceTime <= startOfServiceTime)
                throw std::invalid_argument("start of service time needs to be before end of service time");
            const int vehicleId = static_cast<int>(fleet.size());
            fleet.push_back({vehicleId, vehGraphOrigIdToSeqId[location], startOfServiceTime * 10,
                             endOfServiceTime * 10, capacity});
            maxCapacity = std::max(maxCapacity, capacity);
        }
        std::cout << "done.\n";

        // Create Route State for empty routes.
        RouteState routeState(fleet);



        // Read the request data from file.
        std::cout << "Reading request data from file... " << std::flush;
        std::vector<Request> requests;
        int origin, destination, requestTime, numRiders;
        io::CSVReader<4, io::trim_chars<' '>> reqFileReader(requestFileName);

        if (csvFilesInLoudFormat) {
            reqFileReader.read_header(io::ignore_missing_column, "pickup_spot", "dropoff_spot", "min_dep_time",
                                      "num_riders");
        } else {
            reqFileReader.read_header(io::ignore_missing_column, "origin", "destination", "req_time", "num_riders");
        }

        numRiders = -1;
        while (reqFileReader.read_row(origin, destination, requestTime, numRiders)) {
            if (origin < 0 || origin >= vehGraphOrigIdToSeqId.size() || vehGraphOrigIdToSeqId[origin] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(origin) + "'");
            if (destination < 0 || destination >= vehGraphOrigIdToSeqId.size() ||
                vehGraphOrigIdToSeqId[destination] == INVALID_ID)
                throw std::invalid_argument("invalid location -- '" + std::to_string(destination) + "'");
            if (numRiders > maxCapacity)
                throw std::invalid_argument(
                        "number of riders '" + std::to_string(numRiders) + "' is larger than max vehicle capacity (" +
                        std::to_string(maxCapacity) + ")");
            const auto originSeqId = vehGraphOrigIdToSeqId[origin];
            assert(vehicleInputGraph.toPsgEdge(originSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const auto destSeqId = vehGraphOrigIdToSeqId[destination];
            assert(vehicleInputGraph.toPsgEdge(destSeqId) != CarEdgeToPsgEdgeAttribute::defaultValue());
            const int requestId = static_cast<int>(requests.size());
            if (numRiders == -1) // If number of riders was not specified, assume one rider
                numRiders = 1;
            requests.push_back({requestId, originSeqId, destSeqId, requestTime * 10, numRiders});
            numRiders = -1;
        }
        std::cout << "done.\n";

#ifdef KARRI_USE_CCHS

        // Prepare vehicle CH environment
        using VehCHEnv = CCHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream vehSepDecompFile(vehSepDecompFileName, std::ios::binary);
            if (!vehSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + vehSepDecompFileName + "'");
            SeparatorDecomposition vehSepDecomp;
            vehSepDecomp.readFrom(vehSepDecompFile);
            vehSepDecompFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph, vehSepDecomp);
        }

        // Prepare passenger CH environment
        using PsgCHEnv = CCHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgSepDecompFileName.empty()) {
            std::cout << "Building Separator Decomposition and CCH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the separator decomposition from file, construct and customize CCH.
            std::cout << "Reading Seperator Decomposition from file and building CCH... " << std::flush;
            std::ifstream psgSepDecompFile(psgSepDecompFileName, std::ios::binary);
            if (!psgSepDecompFile.good())
                throw std::invalid_argument("file not found -- '" + psgSepDecompFileName + "'");
            SeparatorDecomposition psgSepDecomp;
            psgSepDecomp.readFrom(psgSepDecompFile);
            psgSepDecompFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph, psgSepDecomp);
        }

#else
        // Prepare vehicle CH environment
        using VehCHEnv = CHEnvironment<VehicleInputGraph, TravelTimeAttribute>;
        std::unique_ptr<VehCHEnv> vehChEnv;
        if (vehHierarchyFileName.empty()) {
            std::cout << "Building CH... " << std::flush;
            vehChEnv = std::make_unique<VehCHEnv>(vehicleInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the CH from file.
            std::cout << "Reading CH from file... " << std::flush;
            std::ifstream vehHierarchyFile(vehHierarchyFileName, std::ios::binary);
            if (!vehHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + vehHierarchyFileName + "'");
            CH vehCh(vehHierarchyFile);
            vehHierarchyFile.close();
            std::cout << "done.\n";
            vehChEnv = std::make_unique<VehCHEnv>(std::move(vehCh));
        }

        // Prepare passenger CH environment
        using PsgCHEnv = CHEnvironment<PsgInputGraph, TravelTimeAttribute>;
        std::unique_ptr<PsgCHEnv> psgChEnv;
        if (psgHierarchyFileName.empty()) {
            std::cout << "Building passenger CH... " << std::flush;
            psgChEnv = std::make_unique<PsgCHEnv>(psgInputGraph);
            std::cout << "done.\n";
        } else {
            // Read the passenger CH from file.
            std::cout << "Reading passenger CH from file... " << std::flush;
            std::ifstream psgHierarchyFile(psgHierarchyFileName, std::ios::binary);
            if (!psgHierarchyFile.good())
                throw std::invalid_argument("file not found -- '" + psgHierarchyFileName + "'");
            CH psgCh(psgHierarchyFile);
            psgHierarchyFile.close();
            std::cout << "done.\n";
            psgChEnv = std::make_unique<PsgCHEnv>(std::move(psgCh));
        }
#endif


        using VehicleLocatorImpl = VehicleLocator<VehicleInputGraph, VehCHEnv>;
        VehicleLocatorImpl locator(vehicleInputGraph, *vehChEnv, routeState);

        CostCalculator calc(routeState, fleet);

        RequestState reqState(calc);

        // Construct Elliptic BCH bucket environment:
        static constexpr bool ELLIPTIC_SORTED_BUCKETS = KARRI_ELLIPTIC_BCH_SORTED_BUCKETS;
        using EllipticBucketsEnv = EllipticBucketsEnvironment<VehicleInputGraph, VehCHEnv, ELLIPTIC_SORTED_BUCKETS>;
        EllipticBucketsEnv ellipticBucketsEnv(vehicleInputGraph, *vehChEnv, routeState,
                                              reqState.stats().updateStats);

        // If we use any BCH queries in the PALS or DALS strategies, we construct the according bucket data structure.
        // Otherwise, we use a last stop buckets substitute that only stores which vehicles' last stops are at a vertex.
#if KARRI_PALS_STRATEGY == KARRI_COL || KARRI_PALS_STRATEGY == KARRI_IND || \
    KARRI_DALS_STRATEGY == KARRI_COL || KARRI_DALS_STRATEGY == KARRI_IND

        static constexpr bool LAST_STOP_SORTED_BUCKETS = KARRI_LAST_STOP_BCH_SORTED_BUCKETS;
        using LastStopBucketsEnv = std::conditional_t<LAST_STOP_SORTED_BUCKETS,
                SortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>,
                UnsortedLastStopBucketsEnvironment<VehicleInputGraph, VehCHEnv>
        >;
        LastStopBucketsEnv lastStopBucketsEnv(vehicleInputGraph, *vehChEnv, routeState,
                                              reqState.stats().updateStats);

#else
        using LastStopBucketsEnv = OnlyLastStopsAtVerticesBucketSubstitute;
        LastStopBucketsEnv lastStopBucketsEnv(vehicleInputGraph, varRouteState, fleet.size());
#endif
        // Last stop bucket environment (or substitute) also serves as a source of information on the last stops at vertices.
        using LastStopAtVerticesInfo = LastStopBucketsEnv;

        using EllipticBCHLabelSet = std::conditional_t<KARRI_ELLIPTIC_BCH_USE_SIMD,
                SimdLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_ELLIPTIC_BCH_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using FeasibleEllipticDistancesImpl = FeasibleEllipticDistances<EllipticBCHLabelSet>;
        FeasibleEllipticDistancesImpl feasibleEllipticPickups(fleet.size(), routeState);
        FeasibleEllipticDistancesImpl feasibleEllipticDropoffs(fleet.size(), routeState);


        using EllipticBCHSearchesImpl = EllipticBCHSearches<VehicleInputGraph, VehCHEnv, CostCalculator::CostFunction,
                EllipticBucketsEnv, LastStopAtVerticesInfo, FeasibleEllipticDistancesImpl, EllipticBCHLabelSet>;
        EllipticBCHSearchesImpl ellipticSearches(vehicleInputGraph, fleet, ellipticBucketsEnv, lastStopBucketsEnv,
                                                 *vehChEnv, routeState, feasibleEllipticPickups,
                                                 feasibleEllipticDropoffs, reqState);


        // Construct remaining request state
        RelevantPDLocs relOrdinaryPickups(fleet.size());
        RelevantPDLocs relOrdinaryDropoffs(fleet.size());
        RelevantPDLocs relPickupsBeforeNextStop(fleet.size());
        RelevantPDLocs relDropoffsBeforeNextStop(fleet.size());
        using RelevantPDLocsFilterImpl = RelevantPDLocsFilter<FeasibleEllipticDistancesImpl, VehicleInputGraph, VehCHEnv>;
        RelevantPDLocsFilterImpl relevantPdLocsFilter(fleet, vehicleInputGraph, *vehChEnv, calc, reqState, routeState,
                                                      feasibleEllipticPickups, feasibleEllipticDropoffs,
                                                      relOrdinaryPickups, relOrdinaryDropoffs, relPickupsBeforeNextStop,
                                                      relDropoffsBeforeNextStop);


        const auto revVehicleGraph = vehicleInputGraph.getReverseGraph();
        using VehicleToPDLocQueryImpl = VehicleToPDLocQuery<VehicleInputGraph>;
        VehicleToPDLocQueryImpl vehicleToPdLocQuery(vehicleInputGraph, revVehicleGraph);


        // Construct PD-distance query
        using PDDistancesLabelSet = std::conditional_t<KARRI_PD_DISTANCES_USE_SIMD,
                SimdLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_PD_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using PDDistancesImpl = PDDistances<PDDistancesLabelSet>;
        PDDistancesImpl pdDistances(reqState);

#if KARRI_PD_STRATEGY == KARRI_BCH_PD_STRAT
        using PDDistanceQueryImpl = PDDistanceQueryStrategies::BCHStrategy<VehicleInputGraph, VehCHEnv, VehicleToPDLocQueryImpl, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(vehicleInputGraph, *vehChEnv, pdDistances, reqState, vehicleToPdLocQuery);

#else // KARRI_PD_STRATEGY == KARRI_CH_PD_STRAT
        using PDDistanceQueryImpl = PDDistanceQueryStrategies::CHStrategy<VehicleInputGraph, VehCHEnv, PDDistancesLabelSet>;
        PDDistanceQueryImpl pdDistanceQuery(vehicleInputGraph, *vehChEnv, pdDistances, reqState);
#endif

        // Construct ordinary assignments finder:
        using OrdinaryAssignmentsFinderImpl = OrdinaryAssignmentsFinder<PDDistancesImpl>;
        OrdinaryAssignmentsFinderImpl ordinaryInsertionsFinder(relOrdinaryPickups, relOrdinaryDropoffs,
                                                               pdDistances, fleet, calc, routeState, reqState);

        // Construct PBNS assignments finder:
        using CurVehLocToPickupLabelSet = PDDistancesLabelSet;
        using CurVehLocToPickupSearchesImpl = CurVehLocToPickupSearches<VehicleInputGraph, VehicleLocatorImpl, VehCHEnv, CurVehLocToPickupLabelSet>;
        CurVehLocToPickupSearchesImpl curVehLocToPickupSearches(vehicleInputGraph, locator, *vehChEnv, routeState,
                                                                reqState, fleet.size());


        using PBNSInsertionsFinderImpl = PBNSAssignmentsFinder<PDDistancesImpl, CurVehLocToPickupSearchesImpl>;
        PBNSInsertionsFinderImpl pbnsInsertionsFinder(relPickupsBeforeNextStop, relOrdinaryDropoffs,
                                                      relDropoffsBeforeNextStop, pdDistances, curVehLocToPickupSearches,
                                                      fleet, calc, routeState, reqState);

        // Construct PALS strategy and assignment finder:
        using PALSLabelSet = std::conditional_t<KARRI_PALS_USE_SIMD,
                SimdLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_PALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;


#if KARRI_PALS_STRATEGY == KARRI_COL
        // Use Collective-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, VehicleToPDLocQueryImpl, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv,
                                  vehicleToPdLocQuery, lastStopBucketsEnv, pdDistances, calc, routeState, reqState);
#elif KARRI_PALS_STRATEGY == KARRI_IND
        // Use Individual-BCH PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, fleet, *vehChEnv, calc, lastStopBucketsEnv, pdDistances,
                                  routeState, reqState, reqState.getBestCost());
#else // KARRI_PALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra PALS Strategy
        using PALSStrategy = PickupAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, PDDistancesImpl, PALSLabelSet>;
        PALSStrategy palsStrategy(vehicleInputGraph, revVehicleGraph, fleet, routeState, lastStopsAtVertices, calc, pdDistances, reqState);
#endif

        using PALSInsertionsFinderImpl = PALSAssignmentsFinder<VehicleInputGraph, PDDistancesImpl, PALSStrategy, LastStopAtVerticesInfo>;
        PALSInsertionsFinderImpl palsInsertionsFinder(palsStrategy, vehicleInputGraph, fleet, calc, lastStopBucketsEnv,
                                                      routeState, pdDistances, reqState);


        // Construct DALS strategy and assignment finder:

        using DALSLabelSet = std::conditional_t<KARRI_DALS_USE_SIMD,
                SimdLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_DALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;
#if KARRI_DALS_STRATEGY == KARRI_COL
        // Use Collective-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::CollectiveBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, routeState, *vehChEnv, lastStopBucketsEnv, calc,
                                  curVehLocToPickupSearches, reqState, relOrdinaryPickups, relPickupsBeforeNextStop);
#elif KARRI_DALS_STRATEGY == KARRI_IND
        // Use Individual-BCH DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::IndividualBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, fleet, *vehChEnv, calc, lastStopBucketsEnv,
                                  curVehLocToPickupSearches, routeState, reqState, relOrdinaryPickups,
                                  relPickupsBeforeNextStop);
#else // KARRI_DALS_STRATEGY == KARRI_DIJ
        // Use Dijkstra DALS Strategy
        using DALSStrategy = DropoffAfterLastStopStrategies::DijkstraStrategy<VehicleInputGraph, CurVehLocToPickupSearchesImpl , DALSLabelSet>;
        DALSStrategy dalsStrategy(vehicleInputGraph, revVehicleGraph, fleet, calc, curVehLocToPickupSearches, routeState, lastStopsAtVertices, reqState, relOrdinaryPickups, relPickupsBeforeNextStop);
#endif

        // Create RPHAST Environment
        RPHASTEnvironment rphastEnv(vehChEnv->getCH());

        using DALSInsertionsFinderImpl = DALSAssignmentsFinder<DALSStrategy>;
        DALSInsertionsFinderImpl dalsInsertionsFinder(dalsStrategy);

        using RequestStateInitializerImpl = RequestStateInitializer<VehicleInputGraph, PsgInputGraph, VehCHEnv, PsgCHEnv, VehicleToPDLocQueryImpl>;
        RequestStateInitializerImpl requestStateInitializer(vehicleInputGraph, psgInputGraph, *vehChEnv, *psgChEnv,
                                                            reqState, vehicleToPdLocQuery);

        using InsertionAsserterImpl = InsertionAsserter<VehicleInputGraph, VehCHEnv>;
        InsertionAsserterImpl asserter(routeState, vehicleInputGraph, *vehChEnv);

#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2 || KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION
        using EllipseReconstructorLabelSet = std::conditional_t<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_USE_SIMD,
                SimdLabelSet<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_LOG_K, ParentInfo::NO_PARENT_INFO>>;
#endif

#if KARRI_TRANSFER_USE_DIJKSTRA_ELLIPSE_RECONSTRUCTION
        using EllipseReconstructorImpl = DijkstraEllipseReconstructor<VehicleInputGraph, VehCHEnv, TravelTimeAttribute, EllipseReconstructorLabelSet>;
        EllipseReconstructorImpl ellipseReconstructor(vehicleInputGraph, revVehicleGraph, *vehChEnv, reqState,
                                                      routeState);
#else
#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2

        static constexpr int ELLIPSES_TOP_VERTICES_DIVISOR =
                KARRI_TRANSFER_HEURISTIC_LEVEL == 0 ? 1 : KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_TOP_VERTICES_DIVISOR;

        static constexpr bool PARALLELIZE_PHAST_DETOUR_ELLIPSES = KARRI_TRANSFER_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE;

        if (!PARALLELIZE_PHAST_DETOUR_ELLIPSES && clp.isSet("max-num-threads")) {
            std::cout
                    << "Warning: -max-num-threads is set but KARRI_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE is OFF."
                    << std::endl;
        }
        if (PARALLELIZE_PHAST_DETOUR_ELLIPSES && !clp.isSet("max-num-threads")) {
            std::cout << "Warning: KARRI_CH_ELLIPSE_RECONSTRUCTOR_PARALLELIZE is ON but -max-num-threads is not set. Queries will execute using 1 thread." << std::endl;
        }

        using EllipseReconstructorImpl = PHASTEllipseReconstructor<VehicleInputGraph, VehCHEnv, EllipticBucketsEnv, PARALLELIZE_PHAST_DETOUR_ELLIPSES, ELLIPSES_TOP_VERTICES_DIVISOR, TraversalCostAttribute, EllipseReconstructorLabelSet>;
        EllipseReconstructorImpl ellipseReconstructor(vehicleInputGraph, *vehChEnv, fleet, ellipticBucketsEnv,
                                                      reqState, routeState, calc);
#else
        using EllipseReconstructorImpl = OnlyAtStopEllipseReconstructor;
        EllipseReconstructorImpl ellipseReconstructor(routeState);
#endif
#endif

        static constexpr bool KARRIT_USE_TP_PARETO_CHECKS = KARRI_TRANSFER_USE_TRANSFER_POINT_PARETO_CHECKS;
        static constexpr bool KARRIT_USE_COST_LOWER_BOUNDS = KARRI_TRANSFER_USE_COST_LOWER_BOUNDS;

#if KARRI_TRANSFER_HEURISTIC_LEVEL < 2
        using DirectTransferDistancesLabelSet = std::conditional_t<KARRI_TRANSFER_DIRECT_DISTANCES_USE_SIMD,
                SimdLabelSet<KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_TRANSFER_DIRECT_DISTANCES_LOG_K, ParentInfo::NO_PARENT_INFO>>;
        using DirectTransferDistancesFinder = BCHDirectTransferDistancesFinder<VehCHEnv, DirectTransferDistancesLabelSet>;
        DirectTransferDistancesFinder pickupToTransferDistancesFinder(vehicleInputGraph.numVertices(), *vehChEnv,
                                                                      PDLocType::PICKUP);
        DirectTransferDistancesFinder transferToDropoffDistancesFinder(vehicleInputGraph.numVertices(), *vehChEnv,
                                                                       PDLocType::DROPOFF);

        using OrdinaryTransferInsertionsImpl = OrdinaryTransferFinder<VehicleInputGraph, VehCHEnv, CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS, KARRIT_USE_TP_PARETO_CHECKS, InsertionAsserterImpl, DirectTransferDistancesFinder>;
        OrdinaryTransferInsertionsImpl ordinaryTransferInsertions = OrdinaryTransferInsertionsImpl(
                vehicleInputGraph,
                *vehChEnv,
                curVehLocToPickupSearches,
                relOrdinaryPickups,
                relPickupsBeforeNextStop,
                relOrdinaryDropoffs,
                relDropoffsBeforeNextStop,
                pickupToTransferDistancesFinder,
                transferToDropoffDistancesFinder,
                fleet, routeState,
                reqState, calc,
                asserter
        );
#else
        using OrdinaryTransferInsertionsImpl = NoOpOrdinaryTransferFinder;
        OrdinaryTransferInsertionsImpl ordinaryTransferInsertions;
#endif

        using TransfersDropoffALSStrategy = Transfers::TransfersDropoffALSBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, DALSLabelSet>;
        TransfersDropoffALSStrategy transferDropoffALSStrategy(vehicleInputGraph, fleet,
                                                               *vehChEnv, calc,
                                                               lastStopBucketsEnv,
                                                               routeState, reqState);

        using TransfersPickupALSStrategy = Transfers::TransfersPickupALSBCHStrategy<VehicleInputGraph, VehCHEnv, LastStopBucketsEnv, PDDistancesImpl, PALSLabelSet>;
        TransfersPickupALSStrategy transferPickupALSStrategy(vehicleInputGraph, fleet,
                                                             *vehChEnv, calc,
                                                             lastStopBucketsEnv,
                                                             pdDistances, routeState,
                                                             reqState);


        using TALSLabelSet = std::conditional_t<KARRI_TRANSFER_TALS_USE_SIMD,
                SimdLabelSet<KARRI_TRANSFER_TALS_LOG_K, ParentInfo::NO_PARENT_INFO>,
                BasicLabelSet<KARRI_TRANSFER_TALS_LOG_K, ParentInfo::NO_PARENT_INFO>>;

#if KARRI_TRANSFER_TALS_STRAT == KARRI_TRANSFER_TALS_PHAST

        using TransferStrategyALSImpl = PHASTStrategyALS<VehicleInputGraph, VehCHEnv, RPHASTEnvironment, TALSLabelSet, std::ofstream>;
        TransferStrategyALSImpl transferALSStrategy(routeState, fleet, vehicleInputGraph, *vehChEnv, rphastEnv);
#else
        using TransferStrategyALSImpl = CHStrategyALS<VehicleInputGraph, VehCHEnv, TALSLabelSet>;
        TransferStrategyALSImpl transferALSStrategy(routeState, fleet, vehicleInputGraph, *vehChEnv);

#endif

        using TransferALSPVehFinderImpl = TransferALSPVehFinder<VehicleInputGraph, VehCHEnv, TransferStrategyALSImpl, TransfersPickupALSStrategy, CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS, KARRIT_USE_TP_PARETO_CHECKS, InsertionAsserterImpl>;
        TransferALSPVehFinderImpl transferALSPVehInsertions(vehicleInputGraph,
                                                            *vehChEnv,
                                                            transferALSStrategy,
                                                            transferPickupALSStrategy,
                                                            curVehLocToPickupSearches,
                                                            relOrdinaryPickups,
                                                            relPickupsBeforeNextStop,
                                                            relOrdinaryDropoffs, fleet,
                                                            routeState, reqState, calc,
                                                            asserter);

        using TransferALSDVehFinderImpl = TransferALSDVehFinder<VehicleInputGraph, VehCHEnv, TransferStrategyALSImpl, CurVehLocToPickupSearchesImpl, KARRIT_USE_COST_LOWER_BOUNDS, KARRIT_USE_TP_PARETO_CHECKS, InsertionAsserterImpl>;
        TransferALSDVehFinderImpl transferALSDVehInsertions(vehicleInputGraph,
                                                            *vehChEnv,
                                                            transferALSStrategy,
                                                            curVehLocToPickupSearches,
                                                            relOrdinaryPickups,
                                                            relPickupsBeforeNextStop,
                                                            fleet,
                                                            routeState, reqState,
                                                            calc,
                                                            asserter);


        using AssignmentsWithTransferFinderImpl = AssignmentsWithTransferFinder<OrdinaryTransferInsertionsImpl,
                TransferALSPVehFinderImpl,
                TransferALSDVehFinderImpl,
                TransfersDropoffALSStrategy,
                EllipseReconstructorImpl,
                InsertionAsserterImpl>;
        AssignmentsWithTransferFinderImpl insertionsWithTransferFinder(ordinaryTransferInsertions,
                                                                       transferALSPVehInsertions,
                                                                       transferALSDVehInsertions,
                                                                       transferDropoffALSStrategy,
                                                                       ellipseReconstructor,
                                                                       reqState, routeState, asserter);

        using InsertionFinderImpl = AssignmentFinder<RequestStateInitializerImpl,
                EllipticBCHSearchesImpl,
                PDDistanceQueryImpl,
                OrdinaryAssignmentsFinderImpl,
                PBNSInsertionsFinderImpl,
                PALSInsertionsFinderImpl,
                DALSInsertionsFinderImpl,
                RelevantPDLocsFilterImpl,
                AssignmentsWithTransferFinderImpl,
                InsertionAsserterImpl
        >;
        InsertionFinderImpl insertionFinder(reqState, requestStateInitializer, ellipticSearches, pdDistanceQuery,
                                            ordinaryInsertionsFinder, pbnsInsertionsFinder, palsInsertionsFinder,
                                            dalsInsertionsFinder, relevantPdLocsFilter,
                                            relOrdinaryPickups, relPickupsBeforeNextStop,
                                            relOrdinaryDropoffs, relDropoffsBeforeNextStop,
                                            insertionsWithTransferFinder, asserter);


#if KARRI_OUTPUT_VEHICLE_PATHS
        using VehPathTracker = PathTracker<VehicleInputGraph, VehCHEnv, std::ofstream>;
        VehPathTracker pathTracker(vehicleInputGraph, *vehChEnv, reqState, routeState, fleet);
#else
        using VehPathTracker = NoOpPathTracker;
        VehPathTracker pathTracker;
#endif


        using SystemStateUpdaterImpl = SystemStateUpdater<VehicleInputGraph, EllipticBucketsEnv, LastStopBucketsEnv, CurVehLocToPickupSearchesImpl, VehPathTracker, std::ofstream>;
        SystemStateUpdaterImpl
                systemStateUpdater(vehicleInputGraph, vehChEnv->getCH(), fleet, reqState, curVehLocToPickupSearches,
                                   pathTracker, routeState, ellipticBucketsEnv, lastStopBucketsEnv);


        // Initialize last stop state for initial locations of vehicles
        for (const auto &veh: fleet) {
            lastStopBucketsEnv.generateIdleBucketEntries(veh);
        }

        // Run simulation:
        using EventSimulationImpl = EventSimulation<InsertionFinderImpl, SystemStateUpdaterImpl, RouteState>;
        EventSimulationImpl eventSimulation(fleet, requests, insertionFinder, systemStateUpdater,
                                            routeState, true);
        eventSimulation.run();

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}