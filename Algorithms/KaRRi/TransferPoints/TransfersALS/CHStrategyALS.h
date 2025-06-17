

#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT>    
    class CHStrategyALS {

    public:

        CHStrategyALS(
            const RouteState &routeState,
            const Fleet &fleet,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv
            ) : routeState(routeState),
                fleet(fleet),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<>()) {}

        //* NEW METHODS FOR ALS TO COMPUTE WITH FASTER ALGORITHMS
        
        // Calculate distances from last stop of every pickup vehicle to all stops of dropoff vehicles
        // Result is of form: vehLastStop - vehAllStops - stop
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromLastStopsToAllStops(const std::vector<int> &vehIdsLastStop, const std::vector<int> &vehIdsAllStops) {
            std::map<int, std::map<int, std::vector<int>>> result;
            
            if (vehIdsLastStop.size() == 0 || vehIdsAllStops.size() == 0)
                return result;
            
            numSearchesRun = 0;

            for (const auto vehIdLastStop : vehIdsLastStop) {
                const auto numStops = routeState.numStopsOf(vehIdLastStop);
                const auto lastStopLoc = routeState.stopLocationsFor(vehIdLastStop)[numStops - 1];
                const auto sourceRank = vehCh.rank(inputGraph.edgeHead(lastStopLoc));

                for (const auto vehIdAllStops : vehIdsAllStops) {
                    const auto &vehAllStops = fleet[vehIdAllStops];
                    result[vehIdLastStop][vehIdAllStops] = runFromSourceToAllStops(sourceRank, vehAllStops);
                    numSearchesRun++;
                }
            }

            return result;
        }
        

        // Calculate distances from pickup to all stops of dropoff vehicles
        // Result is of form: veh - pickup - stop
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllPickupsToAllStops(const std::vector<PDLoc> &pickups, const std::vector<int> &dVehIds) {
            std::map<int, std::map<int, std::vector<int>>> result;

            for (const auto pickup : pickups) {
                const auto pickupLoc = pickup.loc;
                const auto pickupRank = vehCh.rank(inputGraph.edgeHead(pickupLoc));
                
                for (const auto dVehId : dVehIds) {
                    const auto dVeh = &fleet[dVehId];
                    result[dVehId][pickup.id] = runFromSourceToAllStops(pickupRank, *dVeh);
                }
            }

            return result;
        }
        
        // Calculate distances from all stops of pickup vehicles to all dropoffs
        // Result is of form: veh - stop - dropoff
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllStopsToAllDropoffs(const std::vector<int>& pVehIds, const std::vector<PDLoc> &dropoffs) {
            std::map<int, std::map<int, std::vector<int>>> result;
        
            for (const auto pVehId : pVehIds) {
                // const auto &pVeh = &fleet[pVehId];
                const int numStopsPVeh = routeState.numStopsOf(pVehId);
                const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                for (const auto &dropoff : dropoffs) {
                    std::vector<int> sources = std::vector<int>{};
                    for (int i = 1; i < numStopsPVeh; i++) {
                        sources.push_back(vehCh.rank(inputGraph.edgeHead(stopLocationsPVeh[i])));
                    }

                    const int targetRank = vehCh.rank(inputGraph.edgeTail(dropoff.loc));
                    const int offset = inputGraph.travelTime(dropoff.loc);

                    numSearchesRun = sources.size();
    
                    Timer searchTimer;
                    result[pVehId][dropoff.id] = vehChQuery.runManyToOne(sources, targetRank, offset);
                    searchTime = searchTimer.elapsed<std::chrono::nanoseconds>();
                }
            }

            return result;
        }
        

        //* OLD METHODS FOR ALS
        std::vector<int> calculateDistancesFromLastStopToAllStops(const Vehicle &pVeh, const Vehicle &dVeh) {
            numSearchesRun = 0;
            const auto numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const auto lastStopLocPVeh = routeState.stopLocationsFor(pVeh.vehicleId)[numStopsPVeh - 1];
            const auto sourceRank = vehCh.rank(inputGraph.edgeHead(lastStopLocPVeh));

            return runFromSourceToAllStops(sourceRank, dVeh);
        }

        std::vector<int> calculateDistancesFromPickupToAllStops(const int pickupLoc, const Vehicle &dVeh) {
            numSearchesRun = 0;
            const auto sourceRank = vehCh.rank(inputGraph.edgeHead(pickupLoc));
            return runFromSourceToAllStops(sourceRank, dVeh);
        }

        std::vector<int> calculateDistancesFromAllStopsToLocation(const Vehicle &pVeh, const int location) {
            std::vector<int> sources = std::vector<int>{};
            const int numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);

            for (int i = 1; i < numStopsPVeh; i++) {
                sources.push_back(vehCh.rank(inputGraph.edgeHead(routeState.stopLocationsFor(pVeh.vehicleId)[i])));
            }

            const int targetRank = vehCh.rank(inputGraph.edgeTail(location));
            const int offset = inputGraph.travelTime(location);

            numSearchesRun = sources.size();

            Timer searchTimer;
            auto result = vehChQuery.runManyToOne(sources, targetRank, offset);
            searchTime = searchTimer.elapsed<std::chrono::nanoseconds>();

            return result;
        }

        int64_t getNumSearchesRun() {
            return numSearchesRun;
        }

        int64_t getSearchTime() {
            return searchTime;
        }
            

    private:
        std::vector<int> runFromSourceToAllStops(const int sourceRank, const Vehicle &dVeh) {
            const auto numStopsDVeh = routeState.numStopsOf(dVeh.vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(dVeh.vehicleId);

            if (numStopsDVeh < 2) {
                return std::vector<int>{};
            }

            auto ranks = std::vector<int>{};
            auto offsets = std::vector<int>{};

            for (int i = 1; i < numStopsDVeh; i++) {
                ranks.push_back(vehCh.rank(inputGraph.edgeTail(stopLocationsDVeh[i])));
                offsets.push_back(inputGraph.travelTime(stopLocationsDVeh[i]));
            }

            // numSearchesRun = ranks.size();
            
            
            Timer searchTimer;
            auto result = vehChQuery.runOneToMany(sourceRank, ranks, offsets);
            searchTime = searchTimer.elapsed<std::chrono::nanoseconds>();

            return result;
        }
        
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const Fleet &fleet;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        int64_t numSearchesRun;
        int64_t searchTime;
        // int64_t numEdgesRelaxed;
        // int64_t numVerticesScanned;
    
    };








}