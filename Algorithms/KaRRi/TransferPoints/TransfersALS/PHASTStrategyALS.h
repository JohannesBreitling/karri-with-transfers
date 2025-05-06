#pragma once

namespace karri {

    template<typename InputGraphT, typename RPHASTEnv>    
    class PHASTStrategyALS {

    public:

        PHASTStrategyALS(
            const RouteState &routeState,
            const Fleet &fleet,
            const InputGraphT &inputGraph,
            const RPHASTEnv &rphastEnv
            //const VehCHEnvT &vehChEnv
            ) : routeState(routeState),
                fleet(fleet),
                inputGraph(inputGraph),
                // vehCh(vehChEnv.getCH()),
                // vehChQuery(vehChEnv.template getFullCHQuery<>())
                rphastEnv(rphastEnv),
                targetsSelection(rphastEnv.getTargetsSelectionPhase()),
                sourcesSelection(rphastEnv.getSourcesSelectionPhase()),
                forwardQuery(rphastEnv.getForwardRPHASTQuery()),
                reverseQuery(rphastEnv.getReverseRPHASTQuery()) {}

        // NEW METHODS FOR ALS TO COMPUTE WITH FASTER ALGORITHMS
        
        // Calculate distances from last stop of every pickup vehicle to all stops of dropoff vehicles
        // Result is of form: vehLastStop - vehAllStops - stop
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromLastStopsToAllStops(std::vector<int> vehIdsLastStop, std::vector<int> vehIdsAllStops) {
            std::map<int, std::map<int, std::vector<int>>> result;
            
            // Collect all last stops and run selection for those stops
            for (const auto vehIdLastStop : vehIdsLastStop) {
                const auto numStops = routeState.numStopsOf(vehIdsLastStop);
                const auto lastStopLoc = routeState.stopLocationsFor(vehIdsLastStop)[numStops - 1];

                
            }

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
    
        // using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const Fleet &fleet;
        const InputGraphT &inputGraph;

        using RPHASTSelection = RPHASTSelectionPhase<dij::NoCriterion>; 

        const RPHASTEnv &rphastEnv;

        RPHASTSelection targetsSelection;
        RPHASTSelection sourcesSelection;

        using Query = PHASTQuery<CH::SearchGraph, CH::Weight, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, dij::NoCriterion>;
        Query forwardQuery;
        Query reverseQuery;
        
        // const CH &vehCh;
        // VehCHQuery vehChQuery;

        

        int64_t numSearchesRun;
        int64_t searchTime;
        // int64_t numEdgesRelaxed;
        // int64_t numVerticesScanned;
    
    };








}