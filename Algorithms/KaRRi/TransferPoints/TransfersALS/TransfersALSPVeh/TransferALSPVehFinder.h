#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"

#pragma once

namespace karri {

    template<typename TransferALSStrategyT, typename InputGraphT, typename VehCHEnvT>
    class TransferALSPVehFinder {
        
    // The pVeh drives the detour to the transfer point
    // This implies, that the pVeh drives from its last stop to a stop of the dVeh and drops off the customer
    // The dVeh then will then perform the dropoff ORD or ALS
    // The pickup could be BNS, ORD or ALS
    public:

        TransferALSPVehFinder(
            TransferALSStrategyT &strategy,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
            const RelevantPDLocs &relORDDropoffs,
            const Fleet &fleet,
            const RouteState &routeState,
            RequestState &requestState,
            /*const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,*/
            CostCalculator &calc
        ) : strategy(strategy),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            relORDDropoffs(relORDDropoffs),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            /*inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),*/
            calc(calc) {}

        void findAssignments() {
            // Reset the last stop distances
            lastStopDistances = std::map<int, std::map<int, std::vector<int>>>{};

            if ((relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 && relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0) || relORDDropoffs.getVehiclesWithRelevantPDLocs().size() == 0) {
                // No assignment can be found when no vehicle pairs can be built
                return;
            }

            //* Calculate the distances from the last stop of the pickup vehicles to all possible stops of the dropoff vehicles
            for (const auto &pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const auto &pVeh = &fleet[pVehId];
                
                for (const auto &dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                }
            }

            for (const auto &pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                const auto &pVeh = &fleet[pVehId];
                
                for (const auto &dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;   
                }
            }
            
            findAssignmentsWithPickupBNS();
            findAssignmentsWithPickupORD();
            findAssignmentsWithPickupALS();
        }

    private:

        void findAssignmentsWithPickupORD() {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0) {
                return;
            }

            // Loop over all possible vehicle pairs
            for (const auto &pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];
                const auto numStopsPVeh = routeState.numStopsOf(pVehId);

                // Try ORD dropoff
                for (const auto &dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    auto *dVeh = &fleet[dVehId];
                    const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                    const auto stopLocations = routeState.stopLocationsFor(dVehId);

                    const auto distances = lastStopDistances[pVehId][dVehId];

                    // Loop over the possible pickup dropoff combinations
                    for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                        for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                            assert(dropoff.stopIndex < numStopsDVeh);

                            // Try all possible transfer points
                            for (int i = 0; i < dropoff.stopIndex; i++) {
                                // Build the resulting assignment
                                TransferPoint tp;
                                tp.pVeh = pVeh;
                                tp.dVeh = dVeh;
                                tp.loc = stopLocations[i];
                                tp.distancePVehToTransfer = distances[i];
                                tp.distancePVehFromTransfer = 0; 
                                tp.distanceDVehToTransfer = 0;
                                tp.distanceDVehFromTransfer = 0;

                                AssignmentWithTransfer asgn = AssignmentWithTransfer(*pVeh, *dVeh, tp);
                                asgn.pickup = &requestState.pickups[pickup.pdId];
                                asgn.dropoff = &requestState.dropoffs[dropoff.pdId];
                                
                                
                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                
                                asgn.distToTransferPVeh = distances[i];
                                asgn.distFromTransferPVeh = 0;
                                asgn.distToTransferDVeh = 0; // We perform a transfer at stop
                                asgn.distFromTransferDVeh = 0;
                                
                                asgn.distToDropoff = dropoff.distToPDLoc;
                                asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;
                                
                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.transferIdxPVeh = numStopsPVeh - 1;
                                asgn.transferIdxDVeh = i;
                                asgn.dropoffIdx = dropoff.stopIndex;

                                (void) asgn;

                                // Assignemnt built, in this case we dont use any lower bounds, so we can directly try the assignment
                                std::cout << "Try Assignment with Pickup ORD, Transfer ALS\n";
                                
                                // TODO  requestState.tryAssignment(asgn);
                            }
                        }
                    }
                }

                // Try dropoff ALS
                // TODO Implement IndividiualBCHStrategy DALS

            }
        }

        void findAssignmentsWithPickupBNS() {
            //* In this case we consider all vehicles that are able to perform the pickup BNS
            if (relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0) {
                return;
            }
            
            // Loop over all possible vehicle pairs
            for (const auto &pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];
                const auto numStopsPVeh = routeState.numStopsOf(pVehId);

                // Try ORD dropoff
                for (const auto &dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    auto *dVeh = &fleet[dVehId];
                    const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                    const auto stopLocations = routeState.stopLocationsFor(dVehId);

                    const auto distances = lastStopDistances[pVehId][dVehId];

                    // Loop over the possible pickup dropoff combinations
                    for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVehId)) {
                        assert(pickup.stopIndex == 0);
                        for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                            assert(dropoff.stopIndex < numStopsDVeh);

                            // Try all possible transfer points
                            for (int i = 0; i < dropoff.stopIndex; i++) {
                                // Build the resulting assignment
                                TransferPoint tp;
                                tp.pVeh = pVeh;
                                tp.dVeh = dVeh;
                                tp.loc = stopLocations[i];
                                tp.distancePVehToTransfer = distances[i];
                                tp.distancePVehFromTransfer = 0; 
                                tp.distanceDVehToTransfer = 0;
                                tp.distanceDVehFromTransfer = 0;

                                AssignmentWithTransfer asgn = AssignmentWithTransfer(*pVeh, *dVeh, tp);
                                asgn.pickup = &requestState.pickups[pickup.pdId];
                                asgn.dropoff = &requestState.dropoffs[dropoff.pdId];
                                
                                
                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                
                                asgn.distToTransferPVeh = distances[i];
                                asgn.distFromTransferPVeh = 0;
                                asgn.distToTransferDVeh = 0; // We perform a transfer at stop
                                asgn.distFromTransferDVeh = 0;
                                
                                asgn.distToDropoff = dropoff.distToPDLoc;
                                asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;
                                
                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.transferIdxPVeh = numStopsPVeh - 1;
                                asgn.transferIdxDVeh = i;
                                asgn.dropoffIdx = dropoff.stopIndex;

                                (void) asgn;

                                // Assignemnt built, in this case we dont use any lower bounds, so we can directly try the assignment
                                std::cout << "Try Assignment with Pickup BNS, Transfer ALS\n";
                                
                                // TODO  requestState.tryAssignment(asgn);
                            }
                        }
                    }
                }

                // Try dropoff ALS
                // TODO Implement IndividiualBCHStrategy DALS

            }
        }
        


        void findAssignmentsWithPickupALS() {
            // In this case we consider the pickup to be ALS
            // TODO Implement IndividiualBCHStrategy PALS

            //* Calculate the distances from the pickup to the stops of the dropoff vehicle
            // TODO Use the ALS strategy to find the distances

        
        }

        // using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        TransferALSStrategyT &strategy;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;
        
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        /* const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery; */

        CostCalculator &calc;
        

        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, std::vector<int>>> lastStopDistances;

    };



}
