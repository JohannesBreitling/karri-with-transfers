
#pragma once

#include "Algorithms/KaRRi/TransferPoints/TransferPointFinder.h"
#include "Algorithms/KaRRi/TimeUtils.h"

namespace karri {

    template <typename StrategyT>
    class AssignmentsWithTransferFinder {

    public:
        AssignmentsWithTransferFinder(
            StrategyT strategy,
            const Fleet &fleet, const RouteState &routeState,
            CostCalculator &calc,
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs,
            TransferPointFinder<StrategyT> &tpFinder,
            std::vector<TransferPoint> &possibleTransferPoints)
                            : strategy(strategy),
                              fleet(fleet),
                              routeState(routeState),
                              calc(calc),
                              pVehs(pVehs), dVehs(dVehs),
                              pickupDropoffPairs(std::vector<std::tuple<Vehicle, Vehicle>>{}),
                              tpFinder(tpFinder),
                              possibleTransferPoints(possibleTransferPoints) {}

        void init() {
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};
            possibleTransferPoints = std::vector<TransferPoint>{};
        }
    
        void findBestAssignment(int upperBound) {
            // Pair up the vehicles
            pairUpVehicles();

            for (auto &pickupDropoffPair : pickupDropoffPairs) {
                
                auto &pVeh = std::get<0>(pickupDropoffPair);
                auto &dVeh = std::get<1>(pickupDropoffPair);

                const auto &stopLocationsPickupVehicle = routeState.stopLocationsFor(pVeh.vehicleId);
                const auto &stopLocationsDropoffVehicle = routeState.stopLocationsFor(dVeh.vehicleId);

                for (int pIndex = 0; pIndex < routeState.numStopsOf(pVeh.vehicleId) - 1; pIndex++) {
                    for (int dIndex = 0; dIndex < routeState.numStopsOf(dVeh.vehicleId) - 1; dIndex++) {
                        // Get the start locations for the searches 
                        int pickupVehicleStop = stopLocationsPickupVehicle[pIndex];
                        int pickupVehicleNextStop = stopLocationsPickupVehicle[pIndex + 1];
                        int dropoffVehicleStop = stopLocationsDropoffVehicle[dIndex];
                        int dropoffVehicleNextStop = stopLocationsDropoffVehicle[dIndex + 1];

                        // Get the stop ids of the stops
                        int stopIdPickup = routeState.stopIdsFor(pVeh.vehicleId)[pIndex];
                        int stopIdDropoff = routeState.stopIdsFor(dVeh.vehicleId)[dIndex];

                        // Get the hard constraints for stopping criterions for the dijkstra searches
                        int maxDetourPickup = routeState.leewayOfLegStartingAt(stopIdPickup);
                        int maxDetourDropoff = routeState.leewayOfLegStartingAt(stopIdDropoff);

                        // TODO Fall mit Transfer at stop eventuell extra behandeln....
                        if (maxDetourPickup < 0 || maxDetourDropoff < 0)
                            continue;
                        
                        // Use the transfer point finder to find the possible transfer points
                        tpFinder.findTransferPoints(maxDetourPickup, maxDetourDropoff, pickupVehicleStop, pickupVehicleNextStop, dropoffVehicleStop, dropoffVehicleNextStop);

                        // Build the possible transfer points
                        for (auto &tp : possibleTransferPoints) {
                            tp.pVeh = &pVeh;
                            tp.dVeh = &dVeh;
                            tp.dropoffAtTransferStopIdx = pIndex;
                            tp.pickupFromTransferStopIdx = dIndex;
                            
                            // ! Manchmal ist die detour für das dropoff fahrzeug negativ... Eventuell sind die Werte um Faktor 10 verschoben.....
                            // Calculate the vehicle detour for the request
                            const auto detour = calc.calcDetourForTransferpoint(tp);

                            // Calculate the added trip time for the request
                            const auto addedTripTime = calc.calcAddedTripTimeForTransferPoint(tp);
                            (void) addedTripTime;
                            
                            if (detour >= upperBound) {
                                continue;
                            }

                            // std::cout << "DETOUR : " << detour << std::endl;
                        }

                        // TODO Assignments bauen und so

                        // TODO Kosten der Assignments bestimmen -> cost calculator umbauen, partielle kosten bestimmen, kosten für pickup / dropoff bestimmen

                        // TODO Bestes Assignment durchführen
                    }
                }
                
            }

            //std::cout << "Find best assignment (with transfer)" << std::endl;
            //std::cout << "L: " << possibleTransferPoints.size() << std::endl;
            
            /*
            for (auto &tp : possibleTransferPoints) {
                // Calculate detour for transfer points

                const auto detour = calc.calcDetourForTransferpoint(tp);

                std::cout << "DETOUR : " << detour << std::endl;

                // int detour = time_utils::calcDetourForTransferPoint(tp);
                
                //costCalculator.calcDetourForTransferpoint(tp);
                
                //int partialCostsTransferPoint = costCalculator.calcCostForTransferPoint();
                //std::cout << "Costs for tp : " << partialCostsTransferPoint << std::endl;
            } */
        }

    
    
    
    private:
        // Method to bulild up pairs of pickup and dropoff vehicles
        void pairUpVehicles() {
            for (const auto &pVeh : *pVehs.getVehicles()) {
                for (const auto &dVeh : *dVehs.getVehicles()) {
                    // If the pickup is the same as the dropoff vehicle, we already tested the assignment without a transfer
                    if (pVeh->vehicleId == dVeh->vehicleId)
                        continue;

                    pickupDropoffPairs.push_back(std::make_tuple(*pVeh, *dVeh));
                }
            }
        }

        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        CostCalculator &calc;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
        TransferPointFinder<StrategyT> &tpFinder;
        std::vector<TransferPoint> &possibleTransferPoints;
    };


}