#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename RPHASTEnv>    
    class PHASTStrategyALS {

    public:

        PHASTStrategyALS(
            const RouteState &routeState,
            const Fleet &fleet,
            const InputGraphT &inputGraph,
            VehCHEnvT &vehChEnv,
            RPHASTEnv &rphastEnv
            ) : routeState(routeState),
                fleet(fleet),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
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

            if (vehIdsLastStop.size() == 0 || vehIdsAllStops.size() == 0)
                return result;

            std::cout << std::endl;
            std::cout << "- - Overview Runtimes for the RPAHST Strategy\n";

            // Collect all target stops and run selection for those stops
            std::vector<int> targets;
            std::vector<int> offsets;
            for (const auto vehIdAllStops : vehIdsAllStops) {
                const auto stopLocations = routeState.stopLocationsFor(vehIdAllStops);

                for (int i = 1; i < stopLocations.size(); i++) {
                    targets.push_back(vehCh.rank(inputGraph.edgeTail(stopLocations[i])));
                }
            }

            Timer selectionTimer;
            RPHASTSelection allStopsSelection = targetsSelection.run(targets);
            const auto selectionTime = selectionTimer.elapsed<std::chrono::milliseconds>();
            std::cout << "Selection Time:        " << selectionTime << "ms\n";

            // Run one PHAST query for every last stop and populate map
            int queryNumber = 1;
            for (const auto vehIdLastStop : vehIdsLastStop) { // TODO : Do the search for k last stops at the same time
                const auto numStops = routeState.numStopsOf(vehIdLastStop);
                const auto lastStopLoc = routeState.stopLocationsFor(vehIdLastStop)[numStops - 1];
                const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
                const auto lastStopRank = vehCh.rank(lastStopVertex);

                Timer queryTimer;
                forwardQuery.run(allStopsSelection, lastStopRank);
                std::cout << "Query " << queryNumber << " Time:          " << queryTimer.elapsed<std::chrono::milliseconds>() << "ms\n";
                
                // Construct the result
                for (const auto vehIdAllStops : vehIdsAllStops) {
                    const auto stopLocations = routeState.stopLocationsFor(vehIdAllStops);
                    std::vector<int> distances;


                    for (int i = 1; i < stopLocations.size(); i++) {
                        const auto vertex = inputGraph.edgeTail(stopLocations[i]);
                        const auto rank = vehCh.rank(vertex);
                        const auto rankInSelection = allStopsSelection.fullToSubMapping[rank];
                        
                        const auto distance = forwardQuery.getDistance(rankInSelection);
                        const int offset = inputGraph.travelTime(stopLocations[i]);
                        distances.push_back(distance + offset);
                    }

                    result[vehIdLastStop][vehIdAllStops] = distances;
                }
                
                queryNumber++;
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

        const CH &vehCh;

        using SelectionPhase = RPHASTSelectionPhase<dij::NoCriterion>; 
        RPHASTEnv &rphastEnv;

        SelectionPhase targetsSelection;
        SelectionPhase sourcesSelection;

        using Query = PHASTQuery<CH::SearchGraph, CH::Weight, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, dij::NoCriterion>;
        Query forwardQuery;
        Query reverseQuery;
        
        // VehCHQuery vehChQuery;

        

        int64_t numSearchesRun;
        int64_t searchTime;
        // int64_t numEdgesRelaxed;
        // int64_t numVerticesScanned;
    
    };








}