/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "Algorithms/KaRRi/TransferPoints/TransferPointFinder.h"
#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/OrdinaryTransferFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSPVeh/TransferALSPVehFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSDVeh/TransferALSDVehFinder.h"

#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

namespace karri {

    template <typename StrategyT, typename InputGraphT, typename VehCHEnvT, typename CurVehLocToPickupSearchesT, typename DijLabelSet, typename OrdinaryTransferFinderT, typename TransferALSPVehFinderT, typename TransferALSDVehFinderT>
    class AssignmentsWithTransferFinder {

    public:
        AssignmentsWithTransferFinder(
            OrdinaryTransferFinderT &ordinaryTransfers,
            TransferALSPVehFinderT &transfersALSPVeh,
            TransferALSDVehFinderT &transfersALSDVeh,
            StrategyT strategy,
            const Fleet &fleet, const RouteState &routeState,
            RequestState &requestState,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,
            CostCalculator &calc,
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs,
            TransferPointFinder<StrategyT> &tpFinder,
            std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints,
            CurVehLocToPickupSearchesT &searches)
                            : ordinaryTransfers(ordinaryTransfers),
                              transfersALSPVeh(transfersALSPVeh),
                              transfersALSDVeh(transfersALSDVeh),
                              strategy(strategy),
                              fleet(fleet),
                              routeState(routeState),
                              requestState(requestState),
                              inputGraph(inputGraph),
                              vehCh(vehChEnv.getCH()),
                              vehChQuery(vehChEnv.template getFullCHQuery<>()),
                              calc(calc),
                              pVehs(pVehs), dVehs(dVehs),
                              pickupDropoffPairs(std::vector<std::tuple<Vehicle, Vehicle>>{}),
                              tpFinder(tpFinder),
                              transferPoints(transferPoints),
                              searches(searches) {}

        void init() {
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};
            transferPoints = std::map<std::tuple<int, int>, std::vector<TransferPoint>>{};
        }

        // TODO Stats wie die Verteilung der Verbesserung der Kosten liegen und so
        // TODO Im Aufschrieb, SPP gut ausarbeiten, um das Framework gut zu erklären, um eventuelle Follow Ups zu erleichtern 
        // Method to find the best assignment with exactly one transfer, using the best found cost without transfer to prune solutions
        void findBestAssignment() {
            Timer totalTimer;

            // std::cout << "// - - - - - - - - - - - - - - - - - - - -" << std::endl;
            // std::cout << "REQUEST START" << std::endl;

            // auto &stats = requestState.stats().transferStats;
            // numPartialsTried = 0;
            // numAssignmentsTried = 0;
            // numAssignmentsWithUnkownPairedDistanceTried = 0;
            // int64_t numPickupDropoffPairs = 0;
            // int64_t numStopPairs = 0;

            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            //! transfersALSPVeh.findAssignments();
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            //? ordinaryTransfers.findAssignments();
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            
            transfersALSDVeh.findAssignments();

            // TODO PROLEME:
            // - Ein Problem ist, dass zu einem gegebenen Stop keine zugehörigen Fahrzeuge existieren

            return;


            // std::cout << "REQUEST END" << std::endl;
            // std::cout << "- - - - - - - - - - - - - - - - - - - - //" << std::endl;

            // const auto totalTime = totalTimer.elapsed<std::chrono::nanoseconds>();

            // stats.totalTime += totalTime;
            // stats.numPickupDropoffPairs += numPickupDropoffPairs;
            // stats.numStopPairs += numStopPairs;
            // stats.numPartialAssignmentsTried += numPartialsTried;
            // stats.numAssignmentsTried += numAssignmentsTried;
        }


    private:

        void printRequestCost(RequestCost cost) {
            std::cout << "TOTAL: " << cost.total << ", Walking: " << cost.walkingCost << ", Wait: " << cost.waitTimeViolationCost << ", Trip: " << cost.tripCost << ", Change: " << cost.changeInTripCostsOfOthers << ", Veh: " << cost.vehCost << std::endl;
        }

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        OrdinaryTransferFinderT &ordinaryTransfers;
        TransferALSPVehFinderT &transfersALSPVeh;
        TransferALSDVehFinderT &transfersALSDVeh;

        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        CostCalculator &calc;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
        TransferPointFinder<StrategyT> &tpFinder;
        // Stores all transferpoints between two given stop pairs
        std::map<std::tuple<int, int>, std::vector<TransferPoint>> &transferPoints;
        std::vector<Pickup> ordPickups;
        std::vector<Dropoff> ordDropoffs;
        std::vector<Pickup> bnsPickups;
        std::vector<Dropoff> bnsDropoffs;
        std::vector<Pickup> alsPickups;
        std::vector<Dropoff> alsDropoffs;

        CurVehLocToPickupSearchesT &searches;

        // int64_t numAssignmentsTried = 0;
        // int64_t numPartialsTried = 0;
        // int64_t numAssignmentsWithUnkownPairedDistanceTried = 0;
    };


}