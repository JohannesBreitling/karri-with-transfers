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
            asserter(asserter),
            dVehIds(fleet.size()) {}

        void init() {
            totalTime = 0;
            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesPickupALS = 0;
            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;
            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedPickupALS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;
            numTransferPoints = 0;
            numSearchesRunLastStopToDVeh = 0; // TODO
            // numEdgesRelaxedLastStopToDVeh = 0;
            // numVerticesScannedLastStopToDVeh = 0;
            searchTimeLastStopToDVeh = 0; // TODO
            numSearchesRunPickupToDVeh = 0; // TODO
            // numEdgesRelaxedPickupToDVeh = 0;
            // numVerticesScannedPickupToDVeh = 0;
            searchTimePickupToDVeh = 0; // TODO
        }
        
        void findAssignments() {
            Timer total;
            
            // Reset the last stop distances
            lastStopDistances = std::map<int, std::map<int, std::vector<int>>>{};

            //* Calculate the distances from the last stop of the pickup vehicles to all possible stops of the dropoff vehicles (for the case that the pickup is ORD or BNS)
            dVehIds = dropoffALSStrategy.findDropoffsAfterLastStop();
            numCandidateVehiclesDropoffALS += dVehIds.size();


            //* More versatile calculation of the last stop distances
            std::vector<int> relevantPVehIds;
            std::vector<int> relevantDVehIds;

            for (const auto pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                relevantPVehIds.push_back(pVehId);
            }

            for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                relevantPVehIds.push_back(pVehId);
            }

            for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                relevantDVehIds.push_back(dVehId);
            }

            for (const auto dVehId : dVehIds) {
                relevantDVehIds.push_back(dVehId);
            }

            auto newLastStopDistances = strategy.calculateDistancesFromLastStopsToAllStops(relevantPVehIds, relevantDVehIds);

            
            for (const auto pVehId : relORDPickups.getVehiclesWithRelevantPDLocs()) {
                const auto &pVeh = &fleet[pVehId];
                
                for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto &dVeh = &fleet[dVehId];
                    Timer searchTime;
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);
                    searchTimeLastStopToDVeh += searchTime.elapsed<std::chrono::nanoseconds>();
                    numSearchesRunLastStopToDVeh += strategy.getNumSearchesRun();

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;

                    // Assert calculations
                    for (int i = 0; i < distances.size(); i++) {
                        const auto newDistances = newLastStopDistances[pVehId][dVehId];
                        assert(newDistances[i] == distances[i]);
                    }
                }

                for (const auto dVehId : dVehIds) {
                    const auto &dVeh = &fleet[dVehId];
                    Timer searchTime;
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);
                    searchTimeLastStopToDVeh += searchTime.elapsed<std::chrono::nanoseconds>();
                    numSearchesRunLastStopToDVeh += strategy.getNumSearchesRun();

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;

                    // Assert calculations
                    for (int i = 0; i < distances.size(); i++) {
                        assert(newLastStopDistances[pVehId][dVehId][i] == distances[i]);
                    }
                }
            }

            for (const auto pVehId : relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                const auto *pVeh = &fleet[pVehId];
                
                for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    const auto *dVeh = &fleet[dVehId];
                    Timer searchTime;
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);
                    searchTimeLastStopToDVeh += searchTime.elapsed<std::chrono::nanoseconds>();
                    numSearchesRunLastStopToDVeh += strategy.getNumSearchesRun();

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                    numTransferPoints += distances.size();

                    // Assert calculations
                    for (int i = 0; i < distances.size(); i++) {
                        assert(newLastStopDistances[pVehId][dVehId][i] == distances[i]);
                    }
                }

                for (const auto dVehId : dVehIds) {
                    const auto *dVeh = &fleet[dVehId];
                    Timer searchTime;
                    const auto distances = strategy.calculateDistancesFromLastStopToAllStops(*pVeh, *dVeh);
                    searchTimeLastStopToDVeh += searchTime.elapsed<std::chrono::nanoseconds>();
                    numSearchesRunLastStopToDVeh += strategy.getNumSearchesRun();

                    // Save the distances for building the assignments later
                    lastStopDistances[pVehId][dVehId] = distances;
                    numTransferPoints += distances.size();

                    // Assert calculations
                    for (int i = 0; i < distances.size(); i++) {
                        assert(newLastStopDistances[pVehId][dVehId][i] == distances[i]);
                    }
                }
            }

            numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();
            
            findAssignmentsWithPickupBNS();
            assert(postponedAssignments.size() == 0);
            findAssignmentsWithPickupORD();
            findAssignmentsWithPickupALS();

            assert(postponedAssignments.size() == 0);
        
            // Write the stats
            auto &stats = requestState.stats().transferALSPVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            stats.numCandidateVehiclesPickupALS += numCandidateVehiclesPickupALS;
            stats.numCandidateVehiclesDropoffORD += numCandidateVehiclesDropoffORD;
            stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;
            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedPickupALS += numAssignmentsTriedPickupALS;
            stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numTransferPoints += numTransferPoints;
            stats.numSearchesRunLastStopToDVeh += numSearchesRunLastStopToDVeh;
            stats.numEdgesRelaxedLastStopToDVeh += numEdgesRelaxedLastStopToDVeh;
            stats.numVerticesScannedLastStopToDVeh += numVerticesScannedLastStopToDVeh;
            stats.searchTimeLastStopToDVeh += searchTimeLastStopToDVeh;
            stats.numSearchesRunPickupToDVeh += numSearchesRunPickupToDVeh;
            stats.numEdgesRelaxedPickupToDVeh += numEdgesRelaxedPickupToDVeh;
            stats.numVerticesScannedPickupToDVeh += numVerticesScannedPickupToDVeh;
            stats.searchTimePickupToDVeh += searchTimePickupToDVeh;
        }

    private:

        void findAssignmentsWithPickupORD() {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().size() == 0)
                return;

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
                postponedAssignments.clear();
            }
        }


        void findAssignmentsWithPickupALS() {
            //* In this case we consider all vehicles that are able to perform the pickup ALS
            const auto pVehIds = pickupALSStrategy.findPickupsAfterLastStop();
            numCandidateVehiclesPickupALS += pVehIds.size();

            if (pVehIds.size() == 0)
                return;

            //* More versatile distance calculation
            std::vector<int> relevantDVehIds;
            for (const auto dVehId : relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                relevantDVehIds.push_back(dVehId);
            }


            for (const auto dVehId : dVehIds) {
                relevantDVehIds.push_back(dVehId);
            }

            // Calculate the distances from all pickups to all stops of relevant dropoff vehicles
            fastTransferDistances = strategy.calculateDistancesFromAllPickupsToAllStops(requestState.pickups, relevantDVehIds);

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
                Timer searchTimer;
                const auto distancesToTransfer = strategy.calculateDistancesFromPickupToAllStops(pickup->loc, *dVeh);
                numTransferPoints += distancesToTransfer.size();
                searchTimePickupToDVeh += searchTimer.elapsed<std::chrono::nanoseconds>();
                numSearchesRunPickupToDVeh += strategy.getNumSearchesRun();

                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        assert(numStopsDVeh - 1 == distancesToTransfer.size());
                        
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];

                        assert(distancePVehToTransfer == fastTransferDistances[dVehId][pickup->id][i - 1]);
                        
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

                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        finishDistances(asgn, distancePVehToTransfer, distanceToPickup, dropoff.distToPDLoc, 0);
                        assert(asgn.distFromPickup == 0);
                        assert(asgn.distFromTransferPVeh == 0);
                        assert(asgn.distFromDropoff > 0);

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALSForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup) {
            // In this case we consider all the vehicles that are able to perform the dropoff ALS
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
                Timer searchTimer;
                const auto distancesToTransfer = strategy.calculateDistancesFromPickupToAllStops(pickup->loc, *dVeh);
                searchTimePickupToDVeh += searchTimer.elapsed<std::chrono::nanoseconds>();
                numTransferPoints += distancesToTransfer.size();
                numSearchesRunPickupToDVeh += strategy.getNumSearchesRun();

                for (const auto &dropoff : requestState.dropoffs) {
                    assert(numStopsDVeh - 1 == distancesToTransfer.size());

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh; i++) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];

                        assert(distancePVehToTransfer == fastTransferDistances[dVehId][pickup->id][i - 1]);

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
                        assert(distanceToDropoff > 0);
            
                        asgn.pickupType = AFTER_LAST_STOP;
                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        finishDistances(asgn, distancePVehToTransfer, distanceToPickup, 0, distanceToDropoff);
                        assert(asgn.distFromPickup == 0);
                        assert(asgn.distFromTransferPVeh == 0);
                        assert(asgn.distFromDropoff == 0);
                        
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
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);
                
                for (const auto &dropoff : relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];

                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

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
                    
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.dropoffType = ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        finishDistances(asgn, 0, distancePVehToTransfer, dropoff.distToPDLoc, 0);

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            // In this case we consider all the vehicles that are able to perform the dropoff ALS
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
                        assert(distanceToDropoff > 0);
                        
                        asgn.dropoffType = AFTER_LAST_STOP;
                        asgn.pickupType = pickup->stopIndex == 0 ? BEFORE_NEXT_STOP : ORDINARY;
                        asgn.transferTypePVeh = AFTER_LAST_STOP;
                        asgn.transferTypeDVeh = ORDINARY;

                        // If the pickup or dropoff conincides with the transfer, we skip the assignment
                        if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                            continue;

                        finishDistances(asgn, 0, distancePVehToTransfer, 0, distanceToDropoff);
                        assert(asgn.distFromTransferPVeh == 0);
                        assert(asgn.distFromDropoff == 0);
                        assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
                        assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

                        // Try the finished assignment with ORD dropoff
                        tryAssignment(asgn);
                    }
                }
            }
        }

        void finishDistances(AssignmentWithTransfer &asgn, const int pairedDistancePVeh, const int alsDistancePVeh, const int pairedDistanceDVeh, const int alsDistanceDVeh) {
            // assert(pairedDistancePVeh > 0 && pairedDistanceDVeh > 0); Not correct
            // assert(alsDistancePVeh > 0 && alsDistanceDVeh > 0);
            
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

            const bool pairedPVeh = pickupIdx == transferIdxPVeh;
            const bool pairedDVeh = transferIdxDVeh == dropoffIdx;

            const bool pickupAfterLastStop = pickupIdx == numStopsPVeh - 1;
            const bool transferAfterLastStopPVeh = transferIdxPVeh == numStopsPVeh - 1;
            const bool transferAfterLastStopDVeh = transferIdxDVeh == numStopsDVeh - 1;
            const bool dropoffAfterLastStop = dropoffIdx == numStopsDVeh - 1;
            
            //* Pickup distances
            // Distance to pickup
            if (pickupAtStop)
                asgn.distToPickup = 0;

            if (!pickupAtStop && pickupAfterLastStop)
                asgn.distToPickup = alsDistancePVeh;

            // Distance from pickup
            if (pairedPVeh || pickupAfterLastStop)
                asgn.distFromPickup = 0;
            
            if (!pairedPVeh && pickupAtStop)
                asgn.distFromPickup = legPickup;

            // Distance to transfer pVeh
            if (transferAtStopPVeh)
                asgn.distToTransferPVeh = 0;
            
            if (pairedPVeh)
                asgn.distToTransferPVeh = pairedDistancePVeh;

            if (!pickupAfterLastStop && transferAfterLastStopPVeh)
                asgn.distToTransferPVeh = alsDistancePVeh;

            // Distance from transfer pVeh
            if (transferAtStopPVeh)
                asgn.distFromTransferPVeh = legTransferPVeh;

            if (transferAfterLastStopPVeh)
                asgn.distFromTransferPVeh = 0;

            //* Dropoff Distances
            // Distance to transfer dVeh
            if (transferAtStopDVeh)
                asgn.distToTransferDVeh = 0;

            if (!transferAtStopDVeh && transferAfterLastStopDVeh)
                asgn.distToTransferDVeh = alsDistanceDVeh;

            // Distance from transfer dVeh
            if (pairedDVeh || transferAfterLastStopDVeh)
                asgn.distFromTransferDVeh = 0;
            
            if (!pairedDVeh && transferAtStopDVeh)
                asgn.distFromTransferDVeh = legTransferDVeh;
            
            // Distance to dropoff
            if (dropoffAtStop)
                asgn.distToDropoff = 0;
        
            if (pairedDVeh && !dropoffAfterLastStop)
                asgn.distToDropoff = pairedDistanceDVeh;

            if ((!transferAfterLastStopDVeh && dropoffAfterLastStop) || (transferAfterLastStopDVeh && transferAtStopDVeh))
                asgn.distToDropoff = alsDistanceDVeh;

            // Distance from dropoff
            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;

            if (dropoffAfterLastStop)
                asgn.distFromDropoff = 0;
        
            assert(asgn.distFromDropoff > 0 || dropoffAfterLastStop);
            assert(asgn.distFromTransferPVeh > 0 || transferAfterLastStopPVeh);
            assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
            assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

            assert(asgn.distToPickup > 0 || asgn.distFromPickup > 0 || asgn.distToTransferPVeh > 0 || asgn.distFromTransferPVeh > 0);
            assert(asgn.distToTransferDVeh > 0 || asgn.distFromTransferDVeh > 0 || asgn.distToDropoff > 0 || asgn.distFromDropoff > 0);
        }

        void tryAssignment(AssignmentWithTransfer &asgn) {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);

            // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
            if ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
             || (asgn.transferIdxDVeh < numStopsDVeh - 1 && asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
             || (asgn.dropoffIdx < numStopsDVeh - 1 && asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]))
                return;

            switch (asgn.pickupType) {
                case BEFORE_NEXT_STOP:
                    numAssignmentsTriedPickupBNS++;
                    break;

                case ORDINARY:
                    numAssignmentsTriedPickupORD++;
                    break;
                
                case AFTER_LAST_STOP:
                    numAssignmentsTriedPickupALS++;
                    break;
                default:
                    assert(false);
            }

            switch (asgn.dropoffType) {
                case ORDINARY:
                    numAssignmentsTriedDropoffORD++;
                    break;
                case AFTER_LAST_STOP:
                    numAssignmentsTriedDropoffALS++;
                    break;
                default:
                    assert(false);
            }

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

        std::map<int, std::map<int, std::vector<int>>> fastLastStopDistances;
        std::map<int, std::map<int, std::vector<int>>> fastTransferDistances;

        Subset dVehIds;


        //* Statistics for the transfer als pveh assignment finder
        int64_t totalTime;
        
        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;
        int64_t numCandidateVehiclesPickupALS;

        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;
        
        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;
        int64_t numAssignmentsTriedPickupALS;

        int64_t numAssignmentsTriedDropoffORD;
        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;
        
        // Stats for the transfer search itself
        int64_t numTransferPoints;
        
        // Search from last stop to all stops
        int64_t numSearchesRunLastStopToDVeh;
        int64_t numEdgesRelaxedLastStopToDVeh;
        int64_t numVerticesScannedLastStopToDVeh;
        int64_t searchTimeLastStopToDVeh;
        
        // Search from pickup to all stops
        int64_t numSearchesRunPickupToDVeh;
        int64_t numEdgesRelaxedPickupToDVeh;
        int64_t numVerticesScannedPickupToDVeh;
        int64_t searchTimePickupToDVeh;
    };
}
