
#pragma once

namespace karri {

    template <typename InputGraphT, typename VehCHEnvT>
    class InsertionAsserter {

    public:
        InsertionAsserter(const RouteState &routeState,
            const InputGraphT &inputGraph,
            const VehCHEnvT &vehChEnv
            ) : routeState(routeState),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                vehChQuery(vehChEnv.template getFullCHQuery<>()) {}
        

        bool assertAssignment(const AssignmentWithTransfer &asgn) {

            // Assert the distances
            if (!assertPVeh(asgn) || !assertDVeh(asgn))
                return false;

            // Assert the times
            KASSERT(asgn.requestTime <= asgn.depAtPickup && asgn.arrAtTransferPoint > asgn.depAtPickup);

            return true;
        }
        
        int assertLastStopDistance(const int vehId, const int loc) {
            const int numStops = routeState.numStopsOf(vehId);
            const auto stopLocations = routeState.stopLocationsFor(vehId);
            
            const int lastStop = stopLocations[numStops - 1];
            return getDistanceBetweenLocations(lastStop, loc);
        }


    private:

        bool assertPVeh(const AssignmentWithTransfer &asgn) {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);

            const int stopBeforePickup = stopLocationsPVeh[asgn.pickupIdx];
            const int pickup = asgn.pickup->loc;
            const int transfer = asgn.transfer.loc;

            // Assert the distance to the pickup
            if (asgn.pickupIdx > 0) {
                const int distToPickup = stopBeforePickup != pickup ? getDistanceBetweenLocations(stopBeforePickup, pickup) : 0;
                
                if (distToPickup != asgn.distToPickup) {
                    KASSERT(false);
                    return false;
                }
            }
            
            if (asgn.pickupIdx != asgn.transferIdxPVeh && asgn.pickupIdx < numStopsPVeh - 1) {
                // Pickup not als
                const int stopAfterPickup = stopLocationsPVeh[asgn.pickupIdx + 1];
                const int distFromPickup = getDistanceBetweenLocations(pickup, stopAfterPickup);
                
                if (stopAfterPickup == pickup || distFromPickup != asgn.distFromPickup) {
                    KASSERT(false);
                    return false;
                }
            }

            if (asgn.pickupIdx == asgn.transferIdxPVeh) {
                // Assert paired distance
                const int pairedDistance = getDistanceBetweenLocations(pickup, transfer);
                if (pickup == transfer || asgn.distFromPickup > 0 || asgn.distToTransferPVeh != pairedDistance) {
                    KASSERT(false);
                    return false;
                }
            }

            /* // TODO BNS
            if (asgn.pickupIdx == 0) {
                // Assert the paired distance

                // Get the current location of the vehicle
            }*/

            if (asgn.pickupIdx != asgn.transferIdxPVeh) {
                // Assert the distance to the transfer
                const int stopBeforeTransfer = stopLocationsPVeh[asgn.transferIdxPVeh];
                const bool sameLoc = stopBeforeTransfer == transfer;
                const int distanceToTransfer = !sameLoc ? getDistanceBetweenLocations(stopBeforeTransfer, transfer) : 0;

                if (distanceToTransfer != asgn.distToTransferPVeh) {
                    KASSERT(false);
                    return false;
                }
            }

            if (asgn.transferIdxPVeh < numStopsPVeh - 1) {
                // Assert the distance from the transfer (transfer is not als)
                const int stopAfterTransfer = stopLocationsPVeh[asgn.transferIdxPVeh + 1];
                const int distanceFromTransfer = getDistanceBetweenLocations(transfer, stopAfterTransfer);
                
                if (stopAfterTransfer == transfer || distanceFromTransfer != asgn.distFromTransferPVeh) {
                    KASSERT(false);
                    return false;
                }
            }

            return true;
        }

        bool assertDVeh(const AssignmentWithTransfer &asgn) {
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            const int stopBeforeTransfer = stopLocationsDVeh[asgn.transferIdxDVeh];
            const int transfer = asgn.transfer.loc;
            const int dropoff = asgn.dropoff->loc;

            if (asgn.transferIdxDVeh > 0) {
                // Assert the distance to the transfer
                const int distToTransfer = stopBeforeTransfer != transfer ? getDistanceBetweenLocations(stopBeforeTransfer, transfer) : 0;
                if (distToTransfer != asgn.distToTransferDVeh) {
                    KASSERT(false);
                    return false;
                }
            }
            
            if (asgn.transferIdxDVeh != asgn.dropoffIdx && asgn.transferIdxDVeh < numStopsDVeh - 1) {
                // Transfer not als
                const int stopAfterTransfer = stopLocationsDVeh[asgn.transferIdxDVeh + 1];
                const int distFromTransfer = getDistanceBetweenLocations(transfer, stopAfterTransfer);
                
                if (stopAfterTransfer == transfer || distFromTransfer != asgn.distFromTransferDVeh) {
                    KASSERT(false);
                    return false;
                }
            }

            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                // Assert paired distance
                const int dropoff = asgn.dropoff->loc;
                const int pairedDistance = getDistanceBetweenLocations(transfer, dropoff);
                assert(asgn.distFromTransferDVeh == 0 && asgn.distToDropoff == pairedDistance);

                if (asgn.distFromTransferDVeh > 0 || asgn.distToDropoff != pairedDistance) {
                    KASSERT(false);
                    return false;
                }
            }

            /* // TODO BNS
            if (asgn.transferIdxDVeh == 0) {
                // TODO Assert the paired distance

                // TODO Get the current location of the vehicle
            }*/
            
            if (asgn.transferIdxDVeh != asgn.dropoffIdx) {
                // Assert the distance to the dropoff
                const int stopBeforeDropoff = stopLocationsDVeh[asgn.dropoffIdx];
                const bool sameLoc = stopBeforeDropoff == dropoff;
                const int distanceToDropoff = !sameLoc ? getDistanceBetweenLocations(stopBeforeDropoff, dropoff) : 0;
                KASSERT(distanceToDropoff == asgn.distToDropoff);

                if (distanceToDropoff != asgn.distToDropoff) {
                    KASSERT(false);
                    return false;
                }
            }

            if (asgn.dropoffIdx < numStopsDVeh - 1) {
                // Assert the distance from the dropoff (dropoff is not als)
                const int stopAfterDropoff = stopLocationsDVeh[asgn.dropoffIdx + 1];
                const int distanceFromDropoff = getDistanceBetweenLocations(dropoff, stopAfterDropoff);
                KASSERT(stopAfterDropoff != dropoff);
                KASSERT(distanceFromDropoff == asgn.distFromDropoff);
            
                if (stopAfterDropoff == dropoff || distanceFromDropoff != asgn.distFromDropoff) {
                    KASSERT(false);
                    return false;
                }
            }

            return true;
        }

        // Calculates the distance from the edgeHead of from to the edgeHead of to, using the to edge
        int getDistanceBetweenLocations(const int from, const int to) {
            const int sourceRank = vehCh.rank(inputGraph.edgeHead(from));
            const int targetRank = vehCh.rank(inputGraph.edgeTail(to));
            const int travelTime = inputGraph.travelTime(to);
        
            vehChQuery.run(sourceRank, targetRank);
            return vehChQuery.getDistance() + travelTime;
        }

        using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const InputGraphT &inputGraph;
        const CH &vehCh;
        VehCHQuery vehChQuery;
    };


}