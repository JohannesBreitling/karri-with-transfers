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
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromLastStopsToAllStops(const std::vector<int> &vehIdsLastStop, const std::vector<int> &vehIdsAllStops) {
            std::map<int, std::map<int, std::vector<int>>> result;

            if (vehIdsLastStop.size() == 0 || vehIdsAllStops.size() == 0)
                return result;

            // Collect all target stops and run selection for those stops
            std::vector<int> targets;
            for (const auto vehIdAllStops : vehIdsAllStops) {
                const auto stopLocations = routeState.stopLocationsFor(vehIdAllStops);

                for (int i = 1; i < stopLocations.size(); i++) {
                    targets.push_back(vehCh.rank(inputGraph.edgeTail(stopLocations[i])));
                }
            }

            Timer selectionTimer;
            RPHASTSelection allStopsSelection = targetsSelection.run(targets);
            // const auto selectionTime = selectionTimer.elapsed<std::chrono::microseconds>();

            // Run one PHAST query for every last stop and populate map
            int queryNumber = 1;
            for (const auto vehIdLastStop : vehIdsLastStop) { // TODO : Do the search for k last stops at the same time
                const auto numStops = routeState.numStopsOf(vehIdLastStop);
                const auto lastStopLoc = routeState.stopLocationsFor(vehIdLastStop)[numStops - 1];
                const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
                const auto lastStopRank = vehCh.rank(lastStopVertex);

                Timer queryTimer;
                forwardQuery.run(allStopsSelection, lastStopRank);
                
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
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllPickupsToAllStops(const std::vector<PDLoc> &pickups, const std::vector<int> &dVehIds) {
            std::map<int, std::map<int, std::vector<int>>> result;

            if (pickups.size() == 0 || dVehIds.size() == 0)
                return result;

            // Collect all target stops and run selection for those stops
            std::vector<int> targets;
            for (const auto dVehId : dVehIds) {
                const auto stopLocations = routeState.stopLocationsFor(dVehId);

                for (int i = 1; i < stopLocations.size(); i++) {
                    targets.push_back(vehCh.rank(inputGraph.edgeTail(stopLocations[i])));
                }
            }

            Timer selectionTimer;
            RPHASTSelection allStopsSelection = targetsSelection.run(targets);

            // Run one PHAST query for every pickup and populate map
            int queryNumber = 1;
            for (const auto &pickup : pickups) { // TODO : Do the search for k last stops at the same time
                const auto pickupLoc = pickup.loc;
                const auto pickupVertex = inputGraph.edgeHead(pickupLoc);
                const auto pickupRank = vehCh.rank(pickupVertex);

                // Timer queryTimer;
                forwardQuery.run(allStopsSelection, pickupRank);
                for (const auto dVehId : dVehIds) {
                    const auto stopLocations = routeState.stopLocationsFor(dVehId);
                    std::vector<int> distances;
                    
                    for (int i = 1; i < stopLocations.size(); i++) {
                        const auto vertex = inputGraph.edgeTail(stopLocations[i]);
                        const auto rank = vehCh.rank(vertex);
                        const auto rankInSelection = allStopsSelection.fullToSubMapping[rank];
                        
                        const auto distance = forwardQuery.getDistance(rankInSelection);
                        const int offset = inputGraph.travelTime(stopLocations[i]);
                        distances.push_back(distance + offset);
                    }

                    result[dVehId][pickup.id] = distances;
                }
                
                queryNumber++;
            }

            return result;
        }
        
        // Calculate distances from all stops of pickup vehicles to all dropoffs
        // Result is of form: veh - stop - dropoff
        std::map<int, std::map<int, std::vector<int>>> calculateDistancesFromAllStopsToAllDropoffs(const std::vector<int> &pVehIds, const std::vector<PDLoc> &dropoffs) {
            std::map<int, std::map<int, std::vector<int>>> result;

            if (pVehIds.size() == 0 || dropoffs.size() == 0)
                return result;

            // Collect all sources (all stops) and run selection for those stops
            std::vector<int> sources;
            for (const auto pVehId : pVehIds) {
                const auto stopLocations = routeState.stopLocationsFor(pVehId);
                
                for (int i = 1; i < stopLocations.size(); i++) {
                    sources.push_back(vehCh.rank(inputGraph.edgeHead(stopLocations[i])));
                }
            }

            Timer selectionTimer;
            RPHASTSelection allStopsSelection = sourcesSelection.run(sources); 
            
            // Run one PHAST query for every dropoff and populate map
            for (const auto &dropoff : dropoffs) {
                const auto dropoffLoc = dropoff.loc;
                const auto dropoffVertex = inputGraph.edgeTail(dropoffLoc);
                const auto dropoffOffset = inputGraph.travelTime(dropoffLoc);
                const auto dropoffRank = vehCh.rank(dropoffVertex);

                reverseQuery.run(allStopsSelection, dropoffRank);

                for (const auto pVehId : pVehIds) {
                    const auto stopLocations = routeState.stopLocationsFor(pVehId);
                    std::vector<int> distances;
                    
                    for (int i = 1; i < stopLocations.size(); i++) {
                        const auto vertex = inputGraph.edgeHead(stopLocations[i]);
                        const auto rank = vehCh.rank(vertex);
                        const auto rankInSelection = allStopsSelection.fullToSubMapping[rank];

                        const auto distance = dropoffOffset + reverseQuery.getDistance(rankInSelection);
                        distances.push_back(distance);
                    }

                    result[pVehId][dropoff.id] = distances;
                }
            }

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