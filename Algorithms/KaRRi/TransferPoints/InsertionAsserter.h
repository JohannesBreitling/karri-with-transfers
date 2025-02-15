
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
            if (asgn.cost.total >= INFTY)
                return true;
        
            // Assert the distances 
            assertPVeh(asgn);
            assertDVeh(asgn);

            return true;
        }
        
    


    private:

        void assertPVeh(const AssignmentWithTransfer &asgn) {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesPVeh = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            (void) schedDepTimesPVeh;

            const int stopBeforePickup = stopLocationsPVeh[asgn.pickupIdx];
            const int pickup = asgn.pickup->loc;
            const int transfer = asgn.transfer.loc;

            // Assert the distance to the pickup
            if (asgn.pickupIdx > 0) {
                const int distToPickup = stopBeforePickup != pickup ? getDistanceBetweenLocations(stopBeforePickup, pickup) : 0;
                assert(distToPickup == asgn.distToPickup);
            }            
            
            if (asgn.pickupIdx != asgn.transferIdxPVeh && asgn.pickupIdx < numStopsPVeh - 1) {
                // Pickup not als
                const int stopAfterPickup = stopLocationsPVeh[asgn.pickupIdx + 1];
                assert(stopAfterPickup != pickup);
                const int distFromPickup = getDistanceBetweenLocations(pickup, stopAfterPickup);
                assert(distFromPickup == asgn.distFromPickup);
            }

            if (asgn.pickupIdx == asgn.transferIdxPVeh) {
                // Assert paired distance
                assert(pickup != transfer);
                const int pairedDistance = getDistanceBetweenLocations(pickup, transfer);
                assert(asgn.distFromPickup == 0);
                assert(asgn.distToTransferPVeh == pairedDistance);
            }

            if (asgn.pickupIdx == 0) {
                // TODO Assert the paired distance

                // TODO Get the current location of the vehicle
            }

            if (asgn.pickupIdx != asgn.transferIdxPVeh) {
                // Assert the distance to the transfer
                const int stopBeforeTransfer = stopLocationsPVeh[asgn.transferIdxPVeh];
                const bool sameLoc = stopBeforeTransfer == transfer;
                const int distanceToTransfer = !sameLoc ? getDistanceBetweenLocations(stopBeforeTransfer, transfer) : 0;
                assert(distanceToTransfer == asgn.distToTransferPVeh);
            }

            if (asgn.transferIdxPVeh < numStopsPVeh - 1) {
                // Assert the distance from the transfer (transfer is not als)
                const int stopAfterTransfer = stopLocationsPVeh[asgn.transferIdxPVeh + 1];
                assert(stopAfterTransfer != transfer);
                const int distanceFromTransfer = getDistanceBetweenLocations(transfer, stopAfterTransfer);
                assert(distanceFromTransfer == asgn.distFromTransferPVeh);
            }
        }

        void assertDVeh(const AssignmentWithTransfer &asgn) {
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            const auto schedDepTimesDVeh = routeState.schedDepTimesFor(asgn.dVeh->vehicleId);
            (void) schedDepTimesDVeh;

            const int stopBeforeTransfer = stopLocationsDVeh[asgn.transferIdxDVeh];
            const int transfer = asgn.transfer.loc;
            const int dropoff = asgn.dropoff->loc;

            if (asgn.transferIdxDVeh > 0) {
                // Assert the distance to the transfer
                const int distToTransfer = stopBeforeTransfer != transfer ? getDistanceBetweenLocations(stopBeforeTransfer, transfer) : 0;
                assert(distToTransfer == asgn.distToTransferDVeh);
            }
            
            if (asgn.transferIdxDVeh != asgn.dropoffIdx && asgn.transferIdxDVeh < numStopsDVeh - 1) {
                // Transfer not als
                const int stopAfterTransfer = stopLocationsDVeh[asgn.transferIdxDVeh + 1];
                assert(stopAfterTransfer != transfer);
                const int distFromTransfer = getDistanceBetweenLocations(transfer, stopAfterTransfer);
                assert(distFromTransfer == asgn.distFromTransferDVeh);
            }

            if (asgn.transferIdxDVeh == asgn.dropoffIdx) {
                // Assert paired distance
                const int dropoff = asgn.dropoff->loc;
                const int pairedDistance = getDistanceBetweenLocations(transfer, dropoff);
                assert(asgn.distFromTransferDVeh == 0);
                assert(asgn.distToDropoff == pairedDistance);
            }

            if (asgn.transferIdxDVeh == 0) {
                // TODO Assert the paired distance

                // TODO Get the current location of the vehicle
            }
            
            if (asgn.transferIdxDVeh != asgn.dropoffIdx) {
                // Assert the distance to the dropoff
                const int stopBeforeDropoff = stopLocationsDVeh[asgn.dropoffIdx];
                const bool sameLoc = stopBeforeDropoff == dropoff;
                const int distanceToDropoff = !sameLoc ? getDistanceBetweenLocations(stopBeforeDropoff, dropoff) : 0;
                assert(distanceToDropoff == asgn.distToDropoff);
            }

            if (asgn.dropoffIdx < numStopsDVeh - 1) {
                // Assert the distance from the dropoff (dropoff is not als)
                const int stopAfterDropoff = stopLocationsDVeh[asgn.dropoffIdx + 1];
                assert(stopAfterDropoff != dropoff);
                const int distanceFromDropoff = getDistanceBetweenLocations(dropoff, stopAfterDropoff);
                assert(distanceFromDropoff == asgn.distFromDropoff);
            }
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