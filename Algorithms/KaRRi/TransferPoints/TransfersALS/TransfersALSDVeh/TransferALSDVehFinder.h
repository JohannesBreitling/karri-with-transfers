
#pragma once

namespace karri {

template<typename TransferALSStrategyT, typename TransfersDropoffALSStrategyT>
class TransferALSDVehFinder {
    
    // The dVeh drives the detour to the transfer point
    // This implies, that the dVeh drives from its last stop to a stop of the pVeh and picks up the customer
    // The dVeh then will drive to the dropoff and go into idle mode (dropoff ALS)
    // The pickup has to be BNS or ORD
    public:
        TransferALSDVehFinder(
            TransferALSStrategyT &strategy,
            TransfersDropoffALSStrategyT &dropoffALSStrategy,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
            const Fleet &fleet,
            const RouteState &routeState,
            RequestState &requestState,
            /*const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv,*/
            CostCalculator &calc
        ) : strategy(strategy),
            dropoffALSStrategy(dropoffALSStrategy),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            /*inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            vehChQuery(vehChEnv.template getFullCHQuery<>()),*/
            calc(calc) {}

    void findAssignments() {
        std::cout << "Find Assignments with Transfer ALS DVeh\n";

        findAssignmentsWithDropoffALS();
    }

    private:
        void findAssignmentsWithDropoffALS() {
            // The pickup has to be BNS or ORD
            // The set of pickup vehicles are the vehicles with BNS or ORD pickups
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 && relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0) {
                return;
            }

            // The vehicle set for the dropoff is the set of vehicles for the ALS dropoff
            // The distance from the last stop to the dropoff is a lower bound for the distance from the last stop to the dropoff via the transfer point
            
            // TODO Hier muss man zuerst noch andere Sachen berechnen....
            
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();

            for (const auto &dVehId : dVehIds) {
                for (const auto dropoff : requestState.dropoffs) {
                    const auto &dVeh = &fleet[dVehId];
                    // Get the distance from the last stop of the dVeh to the dropoff
                    int distanceLastStopToDropoff = dropoffALSStrategy.getDistanceToDropoff(dVehId, dropoff.id);

                    (void) dVeh;
                    (void) distanceLastStopToDropoff;


                    /*
                    for (const pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                        const auto &pVeh = &fleet[pVehId];
                        
                        for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVehId)) {
                            // Build the resulting assignment
                        }
                    }

                    for (const pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                        const auto &pVeh = &fleet[pVehId];
                        
                        for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                            // Build the resulting assignment

                        }
                    }*/
                
                    
                }
            }


            // TODO Implement the ALS strategy to find the distances


            // Loop over all possible vehicle pairs
            for (const auto &pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];
                const auto numStopsPVeh = routeState.numStopsOf(pVehId);

                (void) pVeh;
                (void) numStopsPVeh;
            }
            
            // Loop over all possible vehicle pairs
            for (const auto &pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];
                const auto numStopsPVeh = routeState.numStopsOf(pVehId);

                (void) pVeh;
                (void) numStopsPVeh;
            }
                /*
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
                */


            //* Calculate the distances from a vehicles last stop to all stops of possible pickup vehicles
            

            //* Calculate the distances from the stops of the pVeh to the possible dropoffs

        }

        TransferALSStrategyT &strategy;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        CostCalculator &calc;

};



}
