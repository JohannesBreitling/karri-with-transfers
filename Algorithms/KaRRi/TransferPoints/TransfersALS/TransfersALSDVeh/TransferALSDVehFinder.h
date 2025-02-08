
#pragma once

namespace karri {

template<typename TransferALSStrategyT, typename TransfersDropoffALSStrategyT, typename CurVehLocToPickupSearchesT>
class TransferALSDVehFinder {
    
    // The dVeh drives the detour to the transfer point
    // This implies, that the dVeh drives from its last stop to a stop of the pVeh and picks up the customer
    // The dVeh then will drive to the dropoff and go into idle mode (dropoff ALS)
    // The pickup has to be BNS or ORD
    public:
        TransferALSDVehFinder(
            TransferALSStrategyT &strategy,
            TransfersDropoffALSStrategyT &dropoffALSStrategy,
            CurVehLocToPickupSearchesT &searches,
            const RelevantPDLocs &relORDPickups,
            const RelevantPDLocs &relBNSPickups,
            std::vector<AssignmentWithTransfer> &postponedAssignments,
            const Fleet &fleet,
            const RouteState &routeState,
            RequestState &requestState,
            CostCalculator &calc
        ) : strategy(strategy),
            dropoffALSStrategy(dropoffALSStrategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            postponedAssignments(postponedAssignments),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc) {}

    void findAssignments() {
        auto &stats = requestState.stats().transferALSDVehStats;
        (void) stats;
        init();

        assert(postponedAssignments.size() == 0);
        findAssignmentsWithDropoffALS();

        if (postponedAssignments.size() > 0)
            std::cout << "Pp: " << postponedAssignments.size() << std::endl;
    }

    void init() {
        // TODO Initialize stats    
    }

    private:
        void findAssignmentsWithDropoffALS() {
            // The pickup has to be BNS or ORD
            // The set of pickup vehicles are the vehicles with BNS or ORD pickups
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 && relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0)
                return;

            // The vehicle set for the dropoff is the set of vehicles for the ALS dropoff
            // The distance from the last stop to the dropoff is a lower bound for the distance from the last stop to the dropoff via the transfer point
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            
            if (dVehIds.size() == 0)
                return;

            for (const auto dVehId : dVehIds) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                
                // Pickup BNS
                for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;
                    
                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    // Calculate the distances from the last stop of the dVeh to all stops of the pVeh
                    const auto distancesToTransfer = strategy.calculateDistancesFromLastStopToAllStops(*dVeh, *pVeh);
                
                    for (const auto &dropoff : requestState.dropoffs) {
                        const auto distancesToDropoff = strategy.calculateDistancesFromAllStopsToLocation(*pVeh, dropoff.loc);

                        for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVehId)) {
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = 1; i < numStopsPVeh; i++) {
                                assert(pickup.stopIndex == 0);
                                const int distancesToTransferSize = distancesToTransfer.size();
                                assert(numStopsPVeh - 1 == distancesToTransferSize);
                                assert(numStopsPVeh - 1 == distancesToDropoff.size());
                                
                                // Construct the transfer point
                                TransferPoint tp = TransferPoint(stopLocationsPVeh[i], pVeh, dVeh);
                                tp.distancePVehToTransfer = 0;
                                tp.distancePVehFromTransfer = 0;
                                tp.distanceDVehToTransfer = distancesToTransfer[i - 1];
                                tp.distanceDVehFromTransfer = 0;

                                tp.pickupFromTransferStopIdx = i;
                                tp.dropoffAtTransferStopIdx = numStopsDVeh - 1;
                            
                                // Build the resulting assignment
                                AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                                asgn.pickup = pickupPDLoc;
                                asgn.dropoff = &dropoff;

                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;

                                asgn.distToDropoff = distancesToDropoff[i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                asgn.distFromTransferPVeh = 0;
                                asgn.distToTransferDVeh = distancesToTransfer[i - 1];
                                asgn.distFromTransferDVeh = 0;
    
                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.dropoffIdx = numStopsDVeh - 1;
                                asgn.transferIdxPVeh = i;
                                asgn.transferIdxDVeh = numStopsDVeh - 1;

                                asgn.pickupType = BEFORE_NEXT_STOP;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // Skip the assignment if the pickup is in the same leg as the transfer, because then we would drive back to the stop before the pickup to perform the transfer
                                if (asgn.pickupIdx == asgn.transferIdxPVeh)
                                    continue;

                                // Try the finished assignment with ORD dropoff
                                tryAssignment(asgn);
                            }
                        }
                    }

                    if (postponedAssignments.size() == 0)
                        continue;

                    finishAssignments(pVeh);
                }

                // Pickup ORD
                for (const auto pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    if (dVehId == pVehId)
                        continue;

                    //* Calculate the distances from the vehicles last stop to all stops of possible pickup vehicles
                    const auto distancesToTransfer = strategy.calculateDistancesFromLastStopToAllStops(*dVeh, *pVeh);
                
                    for (const auto &dropoff : requestState.dropoffs) {
                        //* Calculate the distances from the stops of the pVeh to the possible dropoffs
                        const auto distancesToDropoff = strategy.calculateDistancesFromAllStopsToLocation(*pVeh, dropoff.loc);

                        for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = pickup.stopIndex; i < numStopsPVeh; i++) {
                                assert(numStopsPVeh - 1 == distancesToTransfer.size());
                                assert(numStopsPVeh - 1 == distancesToDropoff.size());
                                
                                // Construct the transfer point
                                TransferPoint tp = TransferPoint(stopLocationsPVeh[i], pVeh, dVeh);                                
                                tp.distancePVehToTransfer = 0;
                                tp.distancePVehFromTransfer = 0;
                                tp.distanceDVehToTransfer = distancesToTransfer[i - 1];
                                tp.distanceDVehFromTransfer = 0;

                                tp.pickupFromTransferStopIdx = i;
                                tp.dropoffAtTransferStopIdx = numStopsDVeh - 1;
                            
                                // Build the resulting assignment
                                AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);
                                asgn.pickup = pickupPDLoc;
                                asgn.dropoff = dropoff;

                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                asgn.distToDropoff = distancesToDropoff[i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                asgn.distFromTransferPVeh = 0;
                                asgn.distToTransferDVeh = distancesToTransfer[i - 1];
                                asgn.distFromTransferDVeh = 0;
    
                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.dropoffIdx = numStopsDVeh - 1;
                                asgn.transferIdxPVeh = i;
                                asgn.transferIdxDVeh = numStopsDVeh - 1;

                                asgn.pickupType = ORDINARY;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // Skip the assignment if the pickup is in the same leg as the transfer, because then we would drive back to the stop before the pickup to perform the transfer
                                if (asgn.pickupIdx == asgn.transferIdxPVeh)
                                    continue;

                                // Try the finished assignment with ORD dropoff
                                tryAssignment(asgn);
                            }
                        }
                    }
                }
            }
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {

            if (asgn.pickupIdx == 0) {
                // Pickup BNS
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
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

            postponedAssignments.clear();
        }

        TransferALSStrategyT &strategy;
        TransfersDropoffALSStrategyT &dropoffALSStrategy;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;

        std::vector<AssignmentWithTransfer> &postponedAssignments;
        
        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;

        CostCalculator &calc;

        int64_t numPossibleDropoffVehs;
        int64_t numPossiblePickupVehs;
        int64_t numORDPVehs;
        int64_t numBNSPVehs;
        int64_t numORDPickups;
        int64_t numBNSPickups;

};



}
