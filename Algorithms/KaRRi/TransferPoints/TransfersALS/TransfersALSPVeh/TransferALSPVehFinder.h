#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"

#pragma once

namespace karri {

    template<typename TransferALSStrategyT, typename TransfersPickupALSStrategyT, typename TransfersDropoffALSStrategyT, typename CurVehLocToPickupSearchesT>
    class TransferALSPVehFinder {
        
    // The pVeh drives the detour to the transfer point
    // This implies, that the pVeh drives from its last stop to a stop of the dVeh and drops off the customer
    // The dVeh then will then perform the dropoff ORD or ALS
    // The pickup could be BNS, ORD or ALS
    public:

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        TransferALSPVehFinder(
            TransferALSStrategyT &strategy,
            TransfersPickupALSStrategyT &pickupALSStrategy,
            TransfersDropoffALSStrategyT &dropoffALSStrategy,
            CurVehLocToPickupSearchesT &searches,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
            const RelevantPDLocs &relORDDropoffs,
            std::vector<AssignmentWithTransfer> &postponedAssignments,
            const Fleet &fleet,
            const RouteState &routeState,
            RequestState &requestState,
            CostCalculator &calc
        ) : strategy(strategy),
            pickupALSStrategy(pickupALSStrategy),
            dropoffALSStrategy(dropoffALSStrategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            relORDDropoffs(relORDDropoffs),
            postponedAssignments(postponedAssignments),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc) {}

        void findAssignments() {
            // Reset the last stop distances
            lastStopDistances = std::map<int, std::map<int, std::vector<int>>>{};

            //* Calculate the distances from the last stop of the pickup vehicles to all possible stops of the dropoff vehicles (for the case that the pickup is ORD or BNS)
            const auto dVehALSIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            for (const auto pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const auto &pVeh = &fleet[pVehId];
                
                for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                }

                for (const auto dVehId : dVehALSIds) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                }
            }

            for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                const auto &pVeh = &fleet[pVehId];
                
                for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;   
                }

                for (const auto dVehId : dVehALSIds) {
                    const auto &dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                }
            }

            assert(postponedAssignments.size() == 0);
            
            findAssignmentsWithPickupBNS();
            findAssignmentsWithPickupORD();
            findAssignmentsWithPickupALS();

            if (postponedAssignments.size() > 0)
                std::cout << "Pp: " << postponedAssignments.size() << std::endl;
        }

    private:

        void findAssignmentsWithPickupORD() {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0) {
                return;
            }

            // Loop over all possible vehicles and pickups
            for (const auto pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];

                for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup);
                    tryDropoffALS(pVeh, &pickup);
                }
            }
        }

        void findAssignmentsWithPickupBNS() {
            //* In this case we consider all vehicles that are able to perform the pickup BNS
            if (relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0) {
                return;
            }

            // Loop over all possible vehicles and pickups
            for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];

                for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup);
                    tryDropoffALS(pVeh, &pickup);
                }

                if (postponedAssignments.size() == 0)
                    continue;

                finishAssignments(pVeh);
            }
        }


        void findAssignmentsWithPickupALS() {
            //* In this case we consider all vehicles that are able to perform the pickup ALS
            const auto pVehIds = pickupALSStrategy.findPickupsAfterLastStop();

            if (pVehIds.size() == 0)
                return;

            for (const auto pVehId : pVehIds) {
                const auto *pVeh = &fleet[pVehId]; 

                for (const auto &pickup : requestState.pickups) {
                    // Get the distance from the last stop of the pVeh to the pickup
                    const auto distanceToPickup = pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                    tryDropoffORDForPickupALS(pVeh, &pickup, distanceToPickup);
                    tryDropoffALSForPickupALS(pVeh, &pickup, distanceToPickup);
                }
            }
        }


        void tryDropoffORDForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            
            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                (void) numStopsDVeh;
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;
                
                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                const auto distancesToTransfer = strategy.calculateDistancesFromPickupToAllStops(pickup->loc, *dVeh);
                
                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        assert(numStopsDVeh - 1 == distancesToTransfer.size());
                        
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];
                        
                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i, distancePVehToTransfer, 0, 0, 0);

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                        const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                        asgn.pickup = pickup;
                        asgn.dropoff = dropoffPDLoc;
                        asgn.distToPickup = distanceToPickup;
                        asgn.distFromPickup = 0;
                        asgn.distToDropoff = dropoff.distToPDLoc;
                        asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;

                        asgn.distToTransferPVeh = distancePVehToTransfer;
                        asgn.distFromTransferPVeh = 0;
                        asgn.distToTransferDVeh = 0;
                        asgn.distFromTransferDVeh = 0;
                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        asgn.pickupIdx = numStopsPVeh - 1;
                        asgn.dropoffIdx = dropoff.stopIndex;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // If the dropoff coincides with a stop, we skip the assignment if the dropoff will be inserted in a different leg
                        if (dropoffIsAtStop(dVeh, dropoffPDLoc->loc) >= 0 && dropoff.stopIndex != dropoffIsAtStop(dVeh, dropoffPDLoc->loc))
                            continue; 

                        // Try the finished assignment with ORD dropoff
                        requestState.tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALSForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup) {
            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (dVehIds.size() == 0)
                return;

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;
                
                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                const auto distancesToTransfer = strategy.calculateDistancesFromPickupToAllStops(pickup->loc, *dVeh);

                for (const auto &dropoff : requestState.dropoffs) {
                    assert(numStopsDVeh - 1 == distancesToTransfer.size());

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh; i++) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i, distancePVehToTransfer, 0, 0, 0);

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                        
                        asgn.pickup = pickup;
                        asgn.dropoff = &dropoff;

                        asgn.distToPickup = distanceToPickup;
                        asgn.distFromPickup = 0;

                        asgn.distToTransferPVeh = distancePVehToTransfer;
                        asgn.distFromTransferPVeh = 0;
                        asgn.distToTransferDVeh = 0;
                        asgn.distFromTransferDVeh = 0;

                        asgn.distToDropoff = dropoffALSStrategy.getDistanceToDropoff(dVehId, dropoff.id);
                        asgn.distFromDropoff = 0;
                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        asgn.pickupIdx = numStopsPVeh - 1;
                        asgn.dropoffIdx = numStopsDVeh - 1;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // Try the finished assignment with ORD dropoff
                        requestState.tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffORD(const Vehicle *pVeh, const RelevantPDLoc *pickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];

            bool bnsLowerBoundUsed = false; 
            int distanceToPickup = pickup->distToPDLoc;
            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId, pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;

                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                (void) numStopsDVeh;
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                
                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];

                    // Try all possible transfer points
                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        assert(numStopsDVeh - 1 == lastStopDistances[pVeh->vehicleId][dVeh->vehicleId].size());
                        
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = lastStopDistances[pVeh->vehicleId][dVeh->vehicleId][i - 1];
                        
                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i, distancePVehToTransfer, 0, 0, 0);

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                        asgn.pickup = pickupPDLoc;
                        asgn.dropoff = dropoffPDLoc;
                        asgn.distFromPickup = pickup->distFromPDLocToNextStop;
                        asgn.distToDropoff = dropoff.distToPDLoc;
                        asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;

                        asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;
                        asgn.distToPickup = distanceToPickup;

                        asgn.distToTransferPVeh = distancePVehToTransfer;
                        asgn.distFromTransferPVeh = 0;
                        asgn.distToTransferDVeh = 0;
                        asgn.distFromTransferDVeh = 0;
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        asgn.pickupIdx = pickup->stopIndex;
                        asgn.dropoffIdx = dropoff.stopIndex;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // If the dropoff coincides with a stop, we skip the assignment if the dropoff will be inserted in a different leg
                        if (dropoffIsAtStop(dVeh, dropoffPDLoc->loc) >= 0 && dropoff.stopIndex != dropoffIsAtStop(dVeh, dropoffPDLoc->loc))
                            continue;

                        // Try the finished assignment with ORD dropoff
                        requestState.tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup) {
            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            
            if (dVehIds.size() == 0)
                return;

            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            bool bnsLowerBoundUsed = false; 
            int distanceToPickup = pickup->distToPDLoc;
            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId, pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId : dVehIds) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;
                
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                
                for (const auto &dropoff : requestState.dropoffs) {
                    assert(numStopsDVeh - 1 == lastStopDistances[pVeh->vehicleId][dVeh->vehicleId].size());

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh; i++) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = lastStopDistances[pVeh->vehicleId][dVeh->vehicleId][i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i, distancePVehToTransfer, 0, 0, 0);
                        if (tp.loc == dropoff.loc)
                            continue;

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                        const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
                        asgn.pickup = pickupPDLoc;
                        asgn.dropoff = &dropoff;
                        asgn.distFromPickup = pickup->distFromPDLocToNextStop;

                        asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;
                        asgn.distToPickup = distanceToPickup;

                        asgn.distToTransferPVeh = distancePVehToTransfer;
                        asgn.distFromTransferPVeh = 0;
                        asgn.distToTransferDVeh = 0;
                        asgn.distFromTransferDVeh = 0;

                        asgn.distToDropoff = dropoffALSStrategy.getDistanceToDropoff(dVehId, dropoff.id);
                        asgn.distFromDropoff = 0;
                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        asgn.pickupIdx = pickup->stopIndex;
                        asgn.dropoffIdx = numStopsDVeh - 1;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // Try the finished assignment with ORD dropoff
                        requestState.tryAssignment(asgn);
                    }
                }
            }
        }

        void finishAssignments(const Vehicle *pVeh) {
            for (auto &asgn : postponedAssignments) {
                assert(asgn.pickupBNSLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            for (auto asgn : postponedAssignments) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                assert(asgn.isFinished());
 
                requestState.tryAssignment(asgn);
            }
        }

        // If the dropoff coincides with a stop, we return the index of the stop
        // Otherwise -1 is returned
        int dropoffIsAtStop(const Vehicle* dVeh, const int dropoffLoc) {
            for (int i = 0; i < routeState.numStopsOf(dVeh->vehicleId); i++) {
                if (routeState.stopLocationsFor(dVeh->vehicleId)[i] == dropoffLoc)
                    return i;
            }

            return -1;
        }


        TransferALSStrategyT &strategy;
        TransfersPickupALSStrategyT &pickupALSStrategy;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;

        std::vector<AssignmentWithTransfer> &postponedAssignments;
        
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator &calc;
        

        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, std::vector<int>>> lastStopDistances;

    };



}
