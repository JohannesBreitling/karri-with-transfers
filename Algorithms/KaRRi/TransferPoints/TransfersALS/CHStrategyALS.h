

#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT>    
    class CHStrategyALS {

    public:

        CHStrategyALS(
            const RouteState &routeState,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv
            ) : routeState(routeState),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<>()) {}

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

            numSearchesRun = ranks.size();
            
            
            Timer searchTimer;
            auto result = vehChQuery.runOneToMany(sourceRank, ranks, offsets);
            searchTime = searchTimer.elapsed<std::chrono::nanoseconds>();

            return result;
        }
        
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;

        int64_t numSearchesRun;
        int64_t searchTime;
        // int64_t numEdgesRelaxed;
        // int64_t numVerticesScanned;
    
    };








}