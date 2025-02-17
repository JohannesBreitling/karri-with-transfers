#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/CHStrategyALS.h"

#pragma once

namespace karri {

    template<typename TransferALSStrategyT, typename TransfersPickupALSStrategyT, typename TransfersDropoffALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
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
            CostCalculator &calc,
            InsertionAsserterT &asserter
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
            calc(calc),
            asserter(asserter) {}

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
                const auto *pVeh = &fleet[pVehId];
                
                for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto *dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;   
                }

                for (const auto dVehId : dVehALSIds) {
                    const auto *dVeh = &fleet[dVehId];
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                }
            }

            assert(postponedAssignments.size() == 0);
            
            findAssignmentsWithPickupBNS();
            findAssignmentsWithPickupORD();
            findAssignmentsWithPickupALS();

            assert(postponedAssignments.size() == 0);
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
                    const int distanceToPickup = pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                    assert(asserter.assertLastStopDistance(pVehId, pickup.loc) == distanceToPickup);
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
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;
                
                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                const auto distancesToTransfer = strategy.calculateDistancesFromPickupToAllStops(pickup->loc, *dVeh);
                
                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        assert(numStopsDVeh - 1 == distancesToTransfer.size());
                        
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];
                        
                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i, distancePVehToTransfer, 0, 0, 0);

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                        asgn.pickupIdx = numStopsPVeh - 1;
                        asgn.dropoffIdx = dropoff.stopIndex;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        asgn.pickup = pickup;
                        const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];
                        asgn.dropoff = dropoffPDLoc;

                        asgn.distToDropoff = dropoff.distToPDLoc;
                        asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;
                        finishDistances(asgn, distancePVehToTransfer, distanceToPickup, dropoff.distToPDLoc, 0);

                        assert(asgn.distFromPickup == 0);
                        assert(asgn.distFromTransferPVeh == 0);

                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        assert(asgn.distFromDropoff > 0);

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
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

                        asgn.pickupIdx = numStopsPVeh - 1;
                        asgn.dropoffIdx = numStopsDVeh - 1;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        const int distanceToDropoff = dropoffALSStrategy.getDistanceToDropoff(dVehId, dropoff.id);
                        finishDistances(asgn, distancePVehToTransfer, distanceToPickup, 0, distanceToDropoff);

                        assert(asgn.distFromPickup == 0);
                        assert(asgn.distFromTransferPVeh == 0);
                        assert(asgn.distFromDropoff == 0);
            
                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffORD(const Vehicle *pVeh, const RelevantPDLoc *pickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
            int distanceToPickup = pickup->distToPDLoc;
            bool bnsLowerBoundUsed = false;
            
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
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                
                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];

                    // Try all possible transfer points
                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = lastStopDistances[pVeh->vehicleId][dVeh->vehicleId][i - 1];
                        
                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i, distancePVehToTransfer, 0, 0, 0);

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                        asgn.pickupIdx = pickup->stopIndex;
                        asgn.dropoffIdx = dropoff.stopIndex;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;

                        asgn.pickup = pickupPDLoc;
                        asgn.dropoff = dropoffPDLoc;

                        assert(asgn.pickupIdx < asgn.transferIdxPVeh);
                        
                        asgn.distToPickup = distanceToPickup;
                        asgn.distFromPickup = pickup->distFromPDLocToNextStop;
                        asgn.distToDropoff = dropoff.distToPDLoc;
                        asgn.distFromDropoff = dropoff.distFromPDLocToNextStop;
                        
                        finishDistances(asgn, 0, distancePVehToTransfer, dropoff.distToPDLoc, 0);
                    
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            
            if (dVehIds.size() == 0)
                return;

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

                        asgn.pickupIdx = pickup->stopIndex;
                        asgn.dropoffIdx = numStopsDVeh - 1;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        assert(asgn.pickupIdx < numStopsPVeh - 1);

                        asgn.pickupBNSLowerBoundUsed = bnsLowerBoundUsed;

                        const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
                        asgn.pickup = pickupPDLoc;
                        asgn.dropoff = &dropoff;

                        asgn.distToPickup = distanceToPickup;
                        asgn.distFromPickup = pickup->distFromPDLocToNextStop;

                        const int distanceToDropoff = dropoffALSStrategy.getDistanceToDropoff(dVehId, dropoff.id);
                        finishDistances(asgn, 0, distancePVehToTransfer, 0, distanceToDropoff);
                        
                        assert(asgn.distFromTransferPVeh == 0);
                        assert(asgn.distFromDropoff == 0);

                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void finishDistances(AssignmentWithTransfer &asgn, const int pairedDistancePVeh, const int alsDistancePVeh, const int pairedDistanceDVeh, const int alsDistanceDVeh) {
            const int pickupIdx = asgn.pickupIdx;
            const int transferIdxPVeh = asgn.transferIdxPVeh;
            const int transferIdxDVeh = asgn.transferIdxDVeh;
            const int dropoffIdx = asgn.dropoffIdx;

            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId); 
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);

            const auto schedDepTimesPVeh = routeState.schedDepTimesFor(asgn.pVeh->vehicleId);
            const auto schedArrTimesPVeh = routeState.schedArrTimesFor(asgn.pVeh->vehicleId);
            const auto schedDepTimesDVeh = routeState.schedDepTimesFor(asgn.dVeh->vehicleId); 
            const auto schedArrTimesDVeh = routeState.schedArrTimesFor(asgn.dVeh->vehicleId);

            const int legPickup = pickupIdx < numStopsPVeh - 1 ? schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] : 0;
            const int legTransferPVeh =  transferIdxPVeh < numStopsPVeh - 1 ? schedArrTimesPVeh[transferIdxPVeh + 1] - schedDepTimesPVeh[transferIdxPVeh] : 0;
            const int legTransferDVeh = transferIdxDVeh < numStopsDVeh - 1 ? schedArrTimesDVeh[transferIdxDVeh + 1] - schedDepTimesDVeh[transferIdxDVeh] : 0;
            const int legDropoff = dropoffIdx < numStopsDVeh - 1 ? schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx] : 0;

            const bool pickupAtStop = asgn.pickup->loc == stopLocationsPVeh[pickupIdx];
            const bool transferAtStopPVeh = asgn.transfer.loc == stopLocationsPVeh[transferIdxPVeh];
            const bool transferAtStopDVeh = asgn.transfer.loc == stopLocationsDVeh[transferIdxDVeh];
            const bool dropoffAtStop = asgn.dropoff->loc == stopLocationsDVeh[dropoffIdx];

            // const bool pickupBNS = pickupIdx == 0;
            // const bool transferBNSDVeh = transferIdxDVeh == 0;
            
            const bool pairedPVeh = pickupIdx == transferIdxPVeh;
            const bool pairedDVeh = transferIdxDVeh == dropoffIdx;

            const bool pickupAfterLastStop = pickupIdx == numStopsPVeh - 1;
            const bool transferAfterLastStopPVeh = transferIdxPVeh == numStopsPVeh - 1;
            const bool transferAfterLastStopDVeh = transferIdxDVeh == numStopsDVeh - 1;
            const bool dropoffAfterLastStop = dropoffIdx == numStopsDVeh - 1;
            
            //* Pickup distances
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (pairedPVeh || pickupAfterLastStop)
                asgn.distFromPickup = 0;

            if (pickupAfterLastStop && !pickupAtStop)
                asgn.distToPickup = alsDistancePVeh;

            if (pairedPVeh)
                asgn.distFromPickup = 0;

            if (pairedPVeh)
                asgn.distToTransferPVeh = pairedDistancePVeh;

            if (pickupAtStop && !pairedPVeh && !pickupAfterLastStop)
                asgn.distFromPickup = legPickup;

            // if (pickupAtStop && pairedPVeh && !pickupAfterLastStop)
            //    asgn.distToTransferPVeh = legPickup;


            //* Transfer distances pVeh
            if (pairedPVeh)
                asgn.distToTransferPVeh = pairedDistancePVeh;

            if (!pairedPVeh && transferAfterLastStopPVeh)
                asgn.distToTransferPVeh = alsDistancePVeh;

            if (transferAfterLastStopPVeh)
                asgn.distFromTransferPVeh = 0;

            if (transferAtStopPVeh && !pairedPVeh)
                asgn.distToTransferPVeh = 0;

            if (transferAtStopPVeh)
                asgn.distFromTransferPVeh = legTransferPVeh;


            //* Transfer distances dVeh
            if (transferAtStopDVeh)
                asgn.distToTransferDVeh = 0;

            if (pairedDVeh || transferAfterLastStopDVeh)
                asgn.distFromTransferDVeh = 0;

            if (transferAfterLastStopDVeh && !transferAtStopDVeh)
                asgn.distToTransferDVeh = alsDistanceDVeh;

            if (pairedDVeh)
                asgn.distFromTransferDVeh = 0;

            if (pairedDVeh)
                asgn.distToDropoff = pairedDistanceDVeh;

            if (transferAtStopDVeh && !pairedDVeh && !transferAfterLastStopDVeh)
                asgn.distFromTransferDVeh = legTransferDVeh;

            // if (transferAtStopDVeh && pairedDVeh && !transferAfterLastStopDVeh)
            //     asgn.distToDropoff = legDropoff;


            //* Dropoff distances
            if (pairedDVeh)
                asgn.distToDropoff = pairedDistanceDVeh;

            if (!pairedDVeh && dropoffAfterLastStop)
                asgn.distToDropoff = alsDistanceDVeh;

            if (dropoffAfterLastStop)
                asgn.distFromDropoff = 0;

            if (dropoffAtStop && !pairedDVeh)
                asgn.distToDropoff = 0;;

            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {
            // Skip unecessary assignments
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            if ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
             || (asgn.transferIdxDVeh < numStopsDVeh - 1 && asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
             || (asgn.dropoffIdx < numStopsDVeh - 1 && asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]))
                return;

            requestState.tryAssignment(asgn);
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
 
                tryAssignment(asgn);
            }
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
        InsertionAsserterT &asserter;
        
        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, std::vector<int>>> lastStopDistances;
    };
}
