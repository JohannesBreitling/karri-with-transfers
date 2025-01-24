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
            const auto numStopsPVeh = routeState.numStopsOf(pVeh.vehicleId);
            const auto lastStopLocPVeh = routeState.stopLocationsFor(pVeh.vehicleId)[numStopsPVeh - 1];
            const auto sourceRank = vehCh.rank(inputGraph.edgeHead(lastStopLocPVeh));

            return runFromSourceToAllStops(sourceRank, dVeh);
        }

        std::vector<int> calculateDistancesFromPickupToAllStops(const int pickupLoc, const Vehicle &dVeh) {
            const auto sourceRank = vehCh.rank(inputGraph.edgeHead(pickupLoc));
            return runFromSourceToAllStops(sourceRank, dVeh);
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

            return vehChQuery.runOneToMany(sourceRank, ranks, offsets);
        }
        
        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;
    
    };








}