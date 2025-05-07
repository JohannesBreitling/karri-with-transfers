#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT>    
    class CHStrategyALS {

    public:

        PHASTStrategyALS(
            const RouteState &routeState,
            const Fleet &fleet,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv
            ) : routeState(routeState),
                fleet(fleet),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<>()) {}

        // NEW METHODS FOR ALS TO COMPUTE WITH FASTER ALGORITHMS
        
        // Calculate distances from last stop of every pickup vehicle to all stops of dropoff vehicles
        // Result is of form: vehLastStop - vehAllStops - stop
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromLastStopsToAllStops(std::vector<int> vehIdsLastStop, std::vector<int> vehIdsAllStops) {
            std::map<int, std::map<int, std::vector<int>>> result;
            
            return result;
        }
        

        // Calculate distances from pickup to all stops of dropoff vehicles
        // Result is of form: veh - pickup - stop
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllPickupsToAllStops(std::vector<PDLoc> &pickups, std::vector<int> dVehIds) {
            std::map<int, std::map<int, std::vector<int>>> result;

            return result;
        }
        
        // Calculate distances from all stops of pickup vehicles to all dropoffs
        // Result is of form: veh - stop - dropoff
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllStopsToAllDropoffs(std::vector<int> pVehIds, std::vector<PDLoc> &dropoffs) {
            std::map<int, std::map<int, std::vector<int>>> result;

            return result;
        }

        int64_t getNumSearchesRun() {
            return numSearchesRun;
        }

        int64_t getSearchTime() {
            return searchTime;
        }
            

    private:
    
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