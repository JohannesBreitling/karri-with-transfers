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

namespace karri {

    class SearchSpaceIntersection {
    
    public:
        SearchSpaceIntersection(const int numSearches) : numSearches(numSearches), currSearch(0), verteciesFound(std::map<int, int>{}) {}
        
        void nextSearch() {
            assert(currSearch < numSearches + 1);
            currSearch++;
        }

        void vertexFound(int v) {
            if (currSearch == 0) {
                verteciesFound[v] = 1;
                return;
            }

            if (verteciesFound.count(v)) {
                verteciesFound[v] = currSearch + 1;
                return;
            }

            verteciesFound.erase(v);
        }

        std::vector<int> getIntersection() {
            auto intersection = std::vector<int>{};

            for (auto it = verteciesFound.begin(); it != verteciesFound.end(); it++) {
                if (it->second < numSearches)
                    continue;
                
                intersection.push_back(it->first);
            }

            return intersection;
        }

    private:
        const int numSearches;
        int currSearch;
        std::map<int, int> verteciesFound;
    };
    
    template<
        typename StrategyT
    >
    class TransferPointFinder {

    public:
        TransferPointFinder(
            StrategyT &strategy,
            const Fleet &fleet,
            const RouteState &routeState,
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs
        ) : strategy(strategy),
            fleet(fleet), routeState(routeState),
            pVehs(pVehs), dVehs(dVehs), intersectedPoints(SearchSpaceIntersection(4)) {}

        void init() {
            pVehs.init();
            dVehs.init();
            pickupDropoffPairs = std::vector<std::tuple<Vehicle, Vehicle>>{};         
        }

        void findTransferPoints() {
            pairUpVehicles();
            
            // Try for all the stop pairs
            for (const auto &pickupDropoffPair : pickupDropoffPairs) {
                
                const auto &pVeh = std::get<0>(pickupDropoffPair);
                const auto &dVeh = std::get<1>(pickupDropoffPair);

                const auto &stopLocationsPickupVehicle = routeState.stopLocationsFor(pVeh.vehicleId);
                const auto &stopLocationsDropoffVehicle = routeState.stopLocationsFor(dVeh.vehicleId);


                // TODO In DijkstraTransferPointStrategy verschieben
                // Get the hard constraints for the vehicles 
                const auto &maxArrTimesPickup = routeState.maxArrTimesFor(pVeh.vehicleId);
                const auto &maxArrTimesDropoff = routeState.maxArrTimesFor(dVeh.vehicleId);
                const auto &schedDepTimesPickup = routeState.schedDepTimesFor(pVeh.vehicleId);
                const auto &schedDepTimesDropoff = routeState.schedDepTimesFor(dVeh.vehicleId); 
                
                for (int pIndex = 0; pIndex < routeState.numStopsOf(pVeh.vehicleId) - 1; pIndex++) {
                    for (int dIndex = 0; dIndex < routeState.numStopsOf(dVeh.vehicleId) - 1; dIndex++) {
                        // Get the start locations for the dijkstra searches 
                        int pickupVehicleStop = stopLocationsPickupVehicle[pIndex];
                        int pickupVehicleNextStop = stopLocationsPickupVehicle[pIndex + 1];
                        int dropoffVehicleStop = stopLocationsDropoffVehicle[dIndex];
                        int dropoffVehicleNextStop = stopLocationsDropoffVehicle[dIndex + 1];

                        // Get the hard constraints for stopping criterions for the dijkstra searches
                        int maxDetourPickup = maxArrTimesPickup[pIndex + 1] - schedDepTimesPickup[pIndex];
                        int maxDetourDropoff = maxArrTimesDropoff[dIndex + 1] - schedDepTimesDropoff[dIndex];

                        // TODO Fall mit Transfer at stop eventuell extra behandeln....
                        if (maxDetourPickup < 0 || maxDetourDropoff < 0)
                            continue;
                        
                        // Run 4 dijkstra searches to determine the intersection of the search spaces                    
                        runForwardSearch(pickupVehicleStop, maxDetourPickup);
                        runBackwardSearch(pickupVehicleNextStop, maxDetourPickup);
                        runForwardSearch(dropoffVehicleStop, maxDetourDropoff);
                        runBackwardSearch(dropoffVehicleNextStop, maxDetourDropoff);
                    }
                }




            }
        }

    private:
        void pairUpVehicles() {
            for (const auto &pVeh : *pVehs.getVehicles()) {
                for (const auto &dVeh : *dVehs.getVehicles()) {
                    // If the pickup is the same as the dropoff vehicle, we already tested the assignment without a transfer
                    if (pVeh.vehicleId == dVeh.vehicleId)
                        continue;

                    pickupDropoffPairs.push_back(std::make_tuple(pVeh, dVeh));
                }
            }
        }

        void runForwardSearch(int s, int maxRadius) {
            std::cout << "fw s: " << s << " with r : " << maxRadius << std::endl;
        }

        void runBackwardSearch(int s, int maxRadius) {
            std::cout << "bw s: " << s << " with r : " << maxRadius << std::endl;
        }

        StrategyT &strategy;
        const Fleet &fleet;
        const RouteState &routeState;
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
        std::vector<std::tuple<Vehicle, Vehicle>> pickupDropoffPairs;
        SearchSpaceIntersection intersectedPoints;
    };

}