/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
/// ******************************************************************************

#pragma once

namespace karri {

template<typename TransferALSStrategyT, typename TransfersDropoffALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
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
            CostCalculator &calc,
            InsertionAsserterT &asserter
        ) : strategy(strategy),
            dropoffALSStrategy(dropoffALSStrategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            postponedAssignments(postponedAssignments),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc),
            asserter(asserter) {}

    void findAssignments() {
        Timer total;
        assert(postponedAssignments.size() == 0);
        
        findAssignmentsWithDropoffALS();

        // Write the stats
        auto &stats = requestState.stats().transferALSDVehStats;
        stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
        stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
        stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
        stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;
        stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
        stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
        stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
        stats.tryAssignmentsTime += tryAssignmentsTime;
        stats.numTransferPoints += numTransferPoints;
        stats.numSearchesRunLastStopToPVeh += numSearchesRunLastStopToPVeh;
        stats.numEdgesRelaxedLastStopToPVeh += numEdgesRelaxedLastStopToPVeh;
        stats.numVerticesScannedLastStopToPVeh += numVerticesScannedLastStopToPVeh;
        stats.searchTimeLastStopToPVeh += searchTimeLastStopToPVeh;
    }

    void init() {
        totalTime = 0;
        numCandidateVehiclesPickupBNS = 0;
        numCandidateVehiclesPickupORD = 0;
        numCandidateVehiclesDropoffALS = 0;
        numAssignmentsTriedPickupBNS = 0;
        numAssignmentsTriedPickupORD = 0;
        numAssignmentsTriedDropoffALS = 0;
        tryAssignmentsTime = 0;
        numTransferPoints = 0;
        numSearchesRunLastStopToPVeh = 0;
        // numEdgesRelaxedLastStopToPVeh = 0;
        // numVerticesScannedLastStopToPVeh = 0;
        searchTimeLastStopToPVeh = 0;
    }

    private:
        void findAssignmentsWithDropoffALS() {
            // The pickup has to be BNS or ORD
            // The set of pickup vehicles are the vehicles with BNS or ORD pickups
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0 && relBNSPickups.getVehiclesWithRelevantPDLocs().size() == 0)
                return;

            numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            
            // The vehicle set for the dropoff is the set of vehicles for the ALS dropoff
            // The distance from the last stop to the dropoff is a lower bound for the distance from the last stop to the dropoff via the transfer point
            const auto dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            numCandidateVehiclesDropoffALS += dVehIds.size();
            
            if (dVehIds.size() == 0)
                return;

            for (const auto dVehId : dVehIds) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                
                // Pickup BNS
                for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;
                    
                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    //* Calculate the distances from the last stop of the dVeh to all stops of the pVeh
                    const auto distancesToTransfer = strategy.calculateDistancesFromLastStopToAllStops(*dVeh, *pVeh);
                    numTransferPoints += distancesToTransfer.size();
                    numSearchesRunLastStopToPVeh += strategy.getNumSearchesRun();
                    searchTimeLastStopToPVeh += strategy.getSearchTime();

                    for (const auto &dropoff : requestState.dropoffs) {
                        const auto distancesToDropoff = strategy.calculateDistancesFromAllStopsToLocation(*pVeh, dropoff.loc);
                        
                        assert(numStopsPVeh - 1 == distancesToTransfer.size());
                        assert(numStopsPVeh - 1 == distancesToDropoff.size());

                        for (const auto &pickup : relBNSPickups.relevantSpotsFor(pVehId)) {
                            assert(pickup.stopIndex == 0);
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = 1; i < numStopsPVeh; i++) { // We start at i = 1 because at the first stop the transfer is only possible if the vehcle is currently waiting for another passenger, so we neglect this case
                                const bool transferAtLastStop = stopLocationsPVeh[i] == stopLocationsDVeh[numStopsDVeh - 1];    
                                
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

                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.dropoffIdx = numStopsDVeh - 1;
                                asgn.transferIdxPVeh = i;
                                asgn.transferIdxDVeh = numStopsDVeh - 1;

                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                asgn.distToDropoff = distancesToDropoff[i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                assert(asgn.pickupIdx != asgn.transferIdxPVeh); // In the pickup bns case we do not have a paired assignment
                                const int lengthOfLeg = i < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[i + 1] - routeState.schedDepTimesFor(pVehId)[i] : 0;
                                asgn.distFromTransferPVeh = lengthOfLeg;
                                
                                asgn.distToTransferDVeh = transferAtLastStop ? 0 : distancesToTransfer[i - 1];
                                asgn.distFromTransferDVeh = 0;
    
                                asgn.pickupType = BEFORE_NEXT_STOP;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
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
                    numTransferPoints += distancesToTransfer.size();
                
                    for (const auto &dropoff : requestState.dropoffs) {
                        //* Calculate the distances from the stops of the pVeh to the possible dropoffs
                        const auto distancesToDropoff = strategy.calculateDistancesFromAllStopsToLocation(*pVeh, dropoff.loc);

                        for (const auto &pickup : relORDPickups.relevantSpotsFor(pVehId)) {
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            assert(numStopsPVeh - 1 == distancesToTransfer.size());
                            assert(numStopsPVeh - 1 == distancesToDropoff.size());

                            for (int i = pickup.stopIndex + 1; i < numStopsPVeh; i++) {
                                assert(pickup.stopIndex > 0);
                                const bool transferAtLastStop = stopLocationsPVeh[i] == stopLocationsDVeh[numStopsDVeh - 1];
                                assert(pickup.stopIndex != i);
                                
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

                                asgn.pickupIdx = pickup.stopIndex;
                                asgn.dropoffIdx = numStopsDVeh - 1;
                                asgn.transferIdxPVeh = i;
                                asgn.transferIdxDVeh = numStopsDVeh - 1;

                                asgn.distToPickup = pickup.distToPDLoc;
                                asgn.distFromPickup = pickup.distFromPDLocToNextStop;
                                asgn.distToDropoff = distancesToDropoff[i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                const int lengthOfLeg = i < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[i + 1] - routeState.schedDepTimesFor(pVehId)[i] : 0;
                                asgn.distFromTransferPVeh = lengthOfLeg;
                                asgn.distToTransferDVeh = transferAtLastStop ? 0 : distancesToTransfer[i - 1];
                                asgn.distFromTransferDVeh = 0;

                                asgn.pickupType = ORDINARY;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
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
            // Skip unecessary assignments
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            
            // If the pickup, transfer coincide with the next stop, we also skip the assignment
            if ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
             || (asgn.transferIdxPVeh < numStopsPVeh - 1 && asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1]))
                return;

            if (asgn.pickupIdx == 0) {
                // Pickup BNS
                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                } else {
                    asgn.pickupBNSLowerBoundUsed = true;
                }
            }

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    numAssignmentsTriedPickupORD++;
                    break;

                default:
                    assert(false);
            }
            numAssignmentsTriedDropoffALS++;
            
            Timer time;
            requestState.tryAssignment(asgn);
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
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

        InsertionAsserterT &asserter;

        //* Statistics for the transfer als dveh assignment finder
        int64_t totalTime;
        
        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;

        int64_t numCandidateVehiclesDropoffALS;
        
        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;
        
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;
        
        // Stats for the transfer search itself
        int64_t numTransferPoints;
        
        // Search from last stop to all stops
        int64_t numSearchesRunLastStopToPVeh;
        int64_t numEdgesRelaxedLastStopToPVeh;
        int64_t numVerticesScannedLastStopToPVeh;
        int64_t searchTimeLastStopToPVeh;
    };
}
