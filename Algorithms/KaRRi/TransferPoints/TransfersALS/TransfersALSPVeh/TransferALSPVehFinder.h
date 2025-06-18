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

    template<typename TransferALSStrategyT, typename TransfersPickupALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
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
                CurVehLocToPickupSearchesT &searches,
                const RelevantPDLocs &relORDPickups,
                const RelevantPDLocs &relBNSPickups,
                const RelevantPDLocs &relORDDropoffs,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                CostCalculator &calc,
                InsertionAsserterT &asserter
        ) : strategy(strategy),
            pickupALSStrategy(pickupALSStrategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            relORDDropoffs(relORDDropoffs),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(calc),
            asserter(asserter) {}

        void init() {
            totalTime = 0;

            numCandidateVehiclesPickupBNS = 0;
            numCandidateVehiclesPickupORD = 0;
            numCandidateVehiclesPickupALS = 0;

            numCandidateVehiclesDropoffORD = 0;
            numCandidateVehiclesDropoffALS = 0;

            numPickups = 0;
            numDropoffs = 0;

            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedPickupALS = 0;
            numAssignmentsTriedDropoffORD = 0;
            numAssignmentsTriedDropoffALS = 0;

            tryAssignmentsTime = 0;

            numTransferPoints = 0;

            searchTimePickupALS = 0;
            searchTimeLastStopToTransfer = 0;
            searchTimePickupToTransfer = 0;
        }

        template<typename EllipsesT>
        void findAssignments(const RelevantDropoffsAfterLastStop& relALSDropoffs,  const EllipsesT&) {
            Timer total;

            // Reset the last stop distances
            lastStopDistances = std::map<int, std::map<int, std::vector<int>>>{};

            //* Calculate the distances from the last stop of the pickup vehicles to all possible stops of the dropoff vehicles (for the case that the pickup is ORD or BNS)


            //* More versatile calculation of the last stop distances
            std::vector<int> relevantPVehIds;
            std::vector<int> relevantDVehIds;

            std::vector<bool> pVehIdFlags(fleet.size(), false);
            std::vector<bool> dVehIdFlags(fleet.size(), false);

            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                relevantPVehIds.push_back(pVehId);
                pVehIdFlags[pVehId] = true;
            }

            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                if (!pVehIdFlags[pVehId]) {
                    relevantPVehIds.push_back(pVehId);
                    pVehIdFlags[pVehId] = true;
                }
            }

            if (!relevantPVehIds.empty()) {
                for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                    relevantDVehIds.push_back(dVehId);
                    dVehIdFlags[dVehId] = true;
                }

                for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                    if (!dVehIdFlags[dVehId]) {
                        relevantDVehIds.push_back(dVehId);
                        dVehIdFlags[dVehId] = true;
                    }
                }
            }

            if (!relevantPVehIds.empty() && !relevantDVehIds.empty()) {
                Timer lastStopSearchesTimer;
                lastStopDistances = strategy.calculateDistancesFromLastStopsToAllStops(relevantPVehIds,
                                                                                       relevantDVehIds);
                searchTimeLastStopToTransfer += lastStopSearchesTimer.elapsed<std::chrono::nanoseconds>();
            }

            numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffORD += relORDDropoffs.getVehiclesWithRelevantPDLocs().size();
            numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();

            findAssignmentsWithPickupBNS(relALSDropoffs);

            findAssignmentsWithPickupORD(relALSDropoffs);

            findAssignmentsWithPickupALS(relALSDropoffs);

            // Write the stats
            auto &stats = requestState.stats().transferALSPVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();

            stats.numCandidateVehiclesPickupBNS += numCandidateVehiclesPickupBNS;
            stats.numCandidateVehiclesPickupORD += numCandidateVehiclesPickupORD;
            stats.numCandidateVehiclesPickupALS += numCandidateVehiclesPickupALS;
            stats.numCandidateVehiclesDropoffORD += numCandidateVehiclesDropoffORD;
            stats.numCandidateVehiclesDropoffALS += numCandidateVehiclesDropoffALS;

            stats.numPickups += requestState.numPickups();
            stats.numDropoffs += requestState.numDropoffs();

            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedPickupALS += numAssignmentsTriedPickupALS;
            stats.numAssignmentsTriedDropoffORD += numAssignmentsTriedDropoffORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;

            stats.tryAssignmentsTime += tryAssignmentsTime;

            stats.numTransferPoints += numTransferPoints;

            stats.searchTimePickupALS += searchTimePickupALS;
            stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            stats.searchTimePickupToTransfer += searchTimePickupToTransfer;
        }

    private:
        void findAssignmentsWithPickupORD(const RelevantDropoffsAfterLastStop& relALSDropoffs) {
            //* In this case we consider all vehicles that are able to perform the pickup ORD
            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // Loop over all possible vehicles and pickups

            std::vector<AssignmentWithTransfer> postponedAssignments; // placeholder
            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, postponedAssignments);

                    if (!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                        tryDropoffALS(pVeh, &pickup, relALSDropoffs, postponedAssignments);
                }
                KASSERT(postponedAssignments.empty());
            }
        }

        void findAssignmentsWithPickupBNS(const RelevantDropoffsAfterLastStop& relALSDropoffs) {
            //* In this case we consider all vehicles that are able to perform the pickup BNS
            if (relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            std::vector<AssignmentWithTransfer> postponedAssignments;
            // Loop over all possible vehicles and pickups
            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                    tryDropoffORD(pVeh, &pickup, postponedAssignments);
                    if (!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                        tryDropoffALS(pVeh, &pickup, relALSDropoffs, postponedAssignments);
                }

                if (postponedAssignments.empty())
                    continue;

                finishAssignments(pVeh, postponedAssignments);
            }
        }


        void findAssignmentsWithPickupALS(const RelevantDropoffsAfterLastStop& relALSDropoffs) {
            //* In this case we consider all vehicles that are able to perform the pickup ALS

            Timer pickupALSTimer;
            const auto pVehIds = pickupALSStrategy.findPickupsAfterLastStop();
            searchTimePickupALS += pickupALSTimer.elapsed<std::chrono::nanoseconds>();

            numCandidateVehiclesPickupALS += pVehIds.size();

            if (pVehIds.empty())
                return;

            //* More versatile distance calculation
            std::vector<int> relevantDVehIds;
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                relevantDVehIds.push_back(dVehId);
            }


            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                relevantDVehIds.push_back(dVehId);
            }

            if (relevantDVehIds.empty())
                return;

            // Calculate the distances from all pickups to all stops of relevant dropoff vehicles
            Timer pickupToTransferSearchTimer;
            transferDistances = strategy.calculateDistancesFromAllPickupsToAllStops(requestState.pickups,
                                                                                    relevantDVehIds);
            searchTimePickupToTransfer = pickupToTransferSearchTimer.elapsed<std::chrono::nanoseconds>();

            for (const auto pVehId: pVehIds) {
                const auto *pVeh = &fleet[pVehId];

                for (const auto &pickup: requestState.pickups) {
                    // Get the distance from the last stop of the pVeh to the pickup
                    const int distanceToPickup = pickupALSStrategy.getDistanceToPickup(pVehId, pickup.id);
                    if (distanceToPickup >= INFTY)
                        continue;
                    
                    assert(asserter.assertLastStopDistance(pVehId, pickup.loc) == distanceToPickup);
                    KASSERT(!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty() || !relORDDropoffs.getVehiclesWithRelevantPDLocs().empty());
                    tryDropoffORDForPickupALS(pVeh, &pickup, distanceToPickup);
                    if (!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty())
                        tryDropoffALSForPickupALS(pVeh, &pickup, distanceToPickup, relALSDropoffs);
                }
            }
        }


        void tryDropoffORDForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                Timer searchTimer;
                const auto &distancesToTransfer = transferDistances[dVehId][pickup->id];

                for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    // Try all possible transfer points
                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    for (int i = dropoff.stopIndex; i > 0; i--) {
                        assert(numStopsDVeh - 1 == distancesToTransfer.size());

                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh - 1, i,
                                                         distancePVehToTransfer, 0, 0, 0);
                        numTransferPoints++;

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
                        tryFinishedAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffALSForPickupALS(const Vehicle *pVeh, const PDLoc *pickup, const int distanceToPickup,
                                       const RelevantDropoffsAfterLastStop &relALSDropoffs) {
            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                if (dVehId == pVeh->vehicleId)
                    continue;

                // Calculate the distances from the pickup to the stops of the dropoff vehicle
                Timer searchTimer;
                const auto &distancesToTransfer = transferDistances[dVehId][pickup->id];
                assert(numStopsDVeh - 1 == distancesToTransfer.size());

                for (const auto& dropoffEntry : relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto& dropoff = requestState.dropoffs[dropoffEntry.dropoffId];

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh; i++) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = distancesToTransfer[i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                         distancePVehToTransfer, 0, 0, 0);
                        numTransferPoints++;

                        // Build the assignment
                        AssignmentWithTransfer asgn = AssignmentWithTransfer(pVeh, dVeh, tp);

                        asgn.pickup = pickup;
                        asgn.dropoff = &dropoff;

                        asgn.pickupIdx = numStopsPVeh - 1;
                        asgn.dropoffIdx = numStopsDVeh - 1;
                        asgn.transferIdxPVeh = numStopsPVeh - 1;
                        asgn.transferIdxDVeh = i;

                        const int distanceToDropoff = dropoffEntry.distToDropoff;
                        KASSERT(distanceToDropoff > 0 && distanceToDropoff < INFTY);

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
                        tryFinishedAssignment(asgn);
                    }
                }
            }
        }

        void tryDropoffORD(const Vehicle *pVeh, const RelevantPDLoc *pickup,
                           std::vector<AssignmentWithTransfer> &postponedAssignments) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const auto *pickupPDLoc = &requestState.pickups[pickup->pdId];
            int distanceToPickup = pickup->distToPDLoc;
            bool bnsLowerBoundUsed = false;

            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId,
                                                                                                  pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relORDDropoffs.getVehiclesWithRelevantPDLocs()) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;

                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                for (const auto &dropoff: relORDDropoffs.relevantSpotsFor(dVehId)) {
                    const auto *dropoffPDLoc = &requestState.dropoffs[dropoff.pdId];

                    if (dropoff.stopIndex == numStopsDVeh - 1)
                        continue;

                    // Try all possible transfer points
                    for (int i = dropoff.stopIndex; i > 0; i--) {

                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = lastStopDistances[pVeh->vehicleId][dVeh->vehicleId][i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                         distancePVehToTransfer, 0, 0, 0);

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

                        // Try the assignment with ORD dropoff
                        tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                    }
                }
            }
        }

        void tryDropoffALS(const Vehicle *pVeh, const RelevantPDLoc *pickup,
                           const RelevantDropoffsAfterLastStop& relALSDropoffs,
                           std::vector<AssignmentWithTransfer>& postponedAssignments) {
            const auto numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);

            // In this case we consider all the vehicles that are able to perform the dropoff ALS
            KASSERT(!relALSDropoffs.getVehiclesWithRelevantPDLocs().empty());

            bool bnsLowerBoundUsed = false;
            int distanceToPickup = pickup->distToPDLoc;

            if (pickup->stopIndex == 0) {
                bnsLowerBoundUsed = searches.knowsDistance(pVeh->vehicleId, pickup->pdId);
                distanceToPickup = bnsLowerBoundUsed ? pickup->distToPDLoc : searches.getDistance(pVeh->vehicleId,
                                                                                                  pickup->pdId);
            }

            if (distanceToPickup >= INFTY)
                return;

            // Loop over all the possible dropoff vehicles and dropoffs
            for (const auto dVehId: relALSDropoffs.getVehiclesWithRelevantPDLocs()) {
                // pVeh an dVeh can not be the same vehicles
                if (dVehId == pVeh->vehicleId)
                    continue;

                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                for (const auto &dropoffEntry: relALSDropoffs.relevantSpotsFor(dVehId)) {
                    const auto& dropoff = requestState.dropoffs[dropoffEntry.dropoffId];
                    assert(numStopsDVeh - 1 == lastStopDistances[pVeh->vehicleId][dVeh->vehicleId].size());

                    // Try all possible transfer points
                    for (int i = 1; i < numStopsDVeh; i++) {
                        // Build the transfer point
                        const int transferLoc = stopLocationsDVeh[i];
                        const int distancePVehToTransfer = lastStopDistances[pVeh->vehicleId][dVeh->vehicleId][i - 1];

                        TransferPoint tp = TransferPoint(transferLoc, pVeh, dVeh, numStopsPVeh, i,
                                                         distancePVehToTransfer, 0, 0, 0);
                        if (tp.loc == dropoff.loc)
                            continue;

                        numTransferPoints++;

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

                        const int distanceToDropoff = dropoffEntry.distToDropoff;
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

                        // Try the assignment with ALS dropoff
                        tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                    }
                }
            }
        }

        void finishDistances(AssignmentWithTransfer &asgn, const int pairedDistancePVeh, const int alsDistancePVeh,
                             const int pairedDistanceDVeh, const int alsDistanceDVeh) {
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

            const int legPickup =
                    pickupIdx < numStopsPVeh - 1 ? schedArrTimesPVeh[pickupIdx + 1] - schedDepTimesPVeh[pickupIdx] : 0;
            const int legTransferPVeh = transferIdxPVeh < numStopsPVeh - 1 ? schedArrTimesPVeh[transferIdxPVeh + 1] -
                                                                             schedDepTimesPVeh[transferIdxPVeh] : 0;
            const int legTransferDVeh = transferIdxDVeh < numStopsDVeh - 1 ? schedArrTimesDVeh[transferIdxDVeh + 1] -
                                                                             schedDepTimesDVeh[transferIdxDVeh] : 0;
            const int legDropoff =
                    dropoffIdx < numStopsDVeh - 1 ? schedArrTimesDVeh[dropoffIdx + 1] - schedDepTimesDVeh[dropoffIdx]
                                                  : 0;

            const bool pickupAtStop = asgn.pickup->loc == stopLocationsPVeh[pickupIdx];
            const bool transferAtStopPVeh = asgn.transfer.loc == stopLocationsPVeh[transferIdxPVeh] && asgn.transferIdxPVeh > asgn.pickupIdx;
            const bool transferAtStopDVeh = asgn.transfer.loc == stopLocationsDVeh[transferIdxDVeh];
            const bool dropoffAtStop = asgn.dropoff->loc == stopLocationsDVeh[dropoffIdx] && asgn.dropoffIdx > asgn.transferIdxDVeh;

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

            if (!pickupAfterLastStop && transferAfterLastStopPVeh && !transferAtStopPVeh)
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
            if (pairedDVeh && !dropoffAfterLastStop)
                asgn.distToDropoff = pairedDistanceDVeh;

            if ((!transferAfterLastStopDVeh && dropoffAfterLastStop) ||
                (transferAfterLastStopDVeh && transferAtStopDVeh))
                asgn.distToDropoff = alsDistanceDVeh;

            if (dropoffAtStop)
                asgn.distToDropoff = 0;

            // Distance from dropoff
            if (dropoffAtStop)
                asgn.distFromDropoff = legDropoff;

            if (dropoffAfterLastStop)
                asgn.distFromDropoff = 0;

            assert(asgn.distFromDropoff > 0 || dropoffAfterLastStop);
            assert(asgn.distFromTransferPVeh > 0 || transferAfterLastStopPVeh);
            assert(asgn.distFromPickup > 0 || asgn.pickupIdx == asgn.transferIdxPVeh);
            assert(asgn.distFromTransferDVeh > 0 || asgn.transferIdxDVeh == asgn.dropoffIdx);

            assert(asgn.distToPickup > 0 || asgn.distFromPickup > 0 || asgn.distToTransferPVeh > 0 ||
                   asgn.distFromTransferPVeh > 0);
            assert(asgn.distToTransferDVeh > 0 || asgn.distFromTransferDVeh > 0 || asgn.distToDropoff > 0 ||
                   asgn.distFromDropoff > 0);
        }

        // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(asgn.dVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(asgn.dVeh->vehicleId);
            return ((asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                    || (asgn.transferIdxDVeh < numStopsDVeh - 1 &&
                        asgn.transfer.loc == stopLocationsDVeh[asgn.transferIdxDVeh + 1])
                    || (asgn.dropoffIdx < numStopsDVeh - 1 &&
                        asgn.dropoff->loc == stopLocationsDVeh[asgn.dropoffIdx + 1]));
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn) {
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
        }

        void tryFinishedAssignment(AssignmentWithTransfer &asgn) {
            KASSERT(asgn.isFinished());
            if (canSkipAssignment(asgn))
                return;
            trackAssignmentTypeStatistic(asgn);
            Timer time;
            const auto cost = calc.calc(asgn, requestState);
            requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer& asgn,
                                                std::vector<AssignmentWithTransfer>& postponedAssignments) {
            if (canSkipAssignment(asgn))
                return;
            trackAssignmentTypeStatistic(asgn);
            Timer time;
            if (!asgn.isFinished()) {
                const auto lowerBound = calc.calcLowerBound(asgn, requestState);
                if (lowerBound.total >= requestState.getBestCost())
                    return;
                postponedAssignments.push_back(asgn);
            } else {
                const auto cost = calc.calc(asgn, requestState);
                requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            }
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void finishAssignments(const Vehicle *pVeh, std::vector<AssignmentWithTransfer>& postponedAssignments) {
            for (const auto &asgn: postponedAssignments) {
                assert(asgn.pickupBNSLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            for (auto &asgn: postponedAssignments) {
                assert(searches.knowsCurrentLocationOf(pVeh->vehicleId));
                assert(searches.knowsDistance(pVeh->vehicleId, asgn.pickup->id));

                const int distance = searches.getDistance(pVeh->vehicleId, asgn.pickup->id);
                asgn.distToPickup = distance;
                asgn.pickupBNSLowerBoundUsed = false;

                assert(asgn.isFinished());
                tryFinishedAssignment(asgn);
            }
            postponedAssignments.clear();
        }

        TransferALSStrategyT &strategy;
        TransfersPickupALSStrategyT &pickupALSStrategy;

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;
        const RelevantPDLocs &relORDDropoffs;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator &calc;
        InsertionAsserterT &asserter;

        // Stores for each pickup vehicle, the distances to all possible stops of dropoff vehicles
        std::map<int, std::map<int, std::vector<int>>> lastStopDistances;
        std::map<int, std::map<int, std::vector<int>>> transferDistances;


        //* Statistics for the transfer als pveh assignment finder
        int64_t totalTime;

        // Stats for the PD Locs
        int64_t numCandidateVehiclesPickupBNS;
        int64_t numCandidateVehiclesPickupORD;
        int64_t numCandidateVehiclesPickupALS;

        int64_t numCandidateVehiclesDropoffORD;
        int64_t numCandidateVehiclesDropoffALS;

        int64_t numPickups;
        int64_t numDropoffs;

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
        int64_t searchTimePickupALS;
        int64_t searchTimePickupToTransfer;
        int64_t searchTimeLastStopToTransfer;
    };
}
