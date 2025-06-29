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

    template<typename TransferALSStrategyT, typename CurVehLocToPickupSearchesT, typename InsertionAsserterT>
    class TransferALSDVehFinder {

        // The dVeh drives the detour to the transfer point
        // This implies, that the dVeh drives from its last stop to a stop of the pVeh and picks up the customer
        // The dVeh then will drive to the dropoff and go into idle mode (dropoff ALS)
        // The pickup has to be BNS or ORD
    public:
        TransferALSDVehFinder(
                TransferALSStrategyT &strategy,
                CurVehLocToPickupSearchesT &searches,
                const RelevantPDLocs &relORDPickups,
                const RelevantPDLocs &relBNSPickups,
                const Fleet &fleet,
                const RouteState &routeState,
                RequestState &requestState,
                InsertionAsserterT &asserter
        ) : strategy(strategy),
            searches(searches),
            relORDPickups(relORDPickups),
            relBNSPickups(relBNSPickups),
            fleet(fleet),
            routeState(routeState),
            requestState(requestState),
            calc(routeState, fleet),
            distancesToTransfer(),
            distancesToDropoff(),
            asserter(asserter) {}

        template<typename EllipsesT>
        void findAssignments(const RelevantDropoffsAfterLastStop& relALSDropoffs, const EllipsesT&) {
            Timer total;

            findAssignmentsWithDropoffALS(relALSDropoffs);

            // Write the stats
            auto &stats = requestState.stats().transferALSDVehStats;
            stats.totalTime = total.elapsed<std::chrono::nanoseconds>();
            stats.numCandidateVehiclesPickupBNS += relBNSPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesPickupORD += relORDPickups.getVehiclesWithRelevantPDLocs().size();
            stats.numCandidateVehiclesDropoffALS += relALSDropoffs.getVehiclesWithRelevantPDLocs().size();
            stats.numPickups += requestState.numPickups();
            stats.numDropoffs += requestState.numDropoffs();
            stats.numAssignmentsTriedPickupBNS += numAssignmentsTriedPickupBNS;
            stats.numAssignmentsTriedPickupORD += numAssignmentsTriedPickupORD;
            stats.numAssignmentsTriedDropoffALS += numAssignmentsTriedDropoffALS;
            stats.tryAssignmentsTime += tryAssignmentsTime;
            stats.numTransferPoints += numTransferPoints;

            stats.searchTimeLastStopToTransfer += searchTimeLastStopToTransfer;
            stats.searchTimeTransferToDropoff += searchTimeTransferToDropoff;
        }

        void init() {
            totalTime = 0;

            numPickups = 0;
            numDropoffs = 0;

            numAssignmentsTriedPickupBNS = 0;
            numAssignmentsTriedPickupORD = 0;
            numAssignmentsTriedDropoffALS = 0;
            tryAssignmentsTime = 0;

            numTransferPoints = 0;

            searchTimeLastStopToTransfer = 0;
            searchTimeTransferToDropoff = 0;
        }

    private:
        void findAssignmentsWithDropoffALS(const RelevantDropoffsAfterLastStop& relALSDropoffs) {
            // The pickup has to be BNS or ORD
            // The set of pickup vehicles are the vehicles with BNS or ORD pickups
            if (relORDPickups.getVehiclesWithRelevantPDLocs().empty() &&
                relBNSPickups.getVehiclesWithRelevantPDLocs().empty())
                return;

            // The vehicle set for the dropoff is the set of vehicles for the ALS dropoff
            // The distance from the last stop to the dropoff is a lower bound for the distance from the last stop to the dropoff via the transfer point
            const auto& dVehIds = relALSDropoffs.getVehiclesWithRelevantPDLocs();

            if (dVehIds.empty())
                return;

            //* Calculate the relevant distances from all stops of the 
            std::vector<int> pVehIds;
            std::vector<bool> pVehIdFlags(fleet.size(), false);

            for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                pVehIds.push_back(pVehId);
                pVehIdFlags[pVehId] = true;
            }

            for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                if (pVehIdFlags[pVehId]) {
                    continue;
                }

                pVehIds.push_back(pVehId);
                pVehIdFlags[pVehId] = true;
            }

            Timer searchToDropoffsTimer;
            distancesToDropoff = strategy.calculateDistancesFromAllStopsToAllDropoffs(pVehIds, requestState.dropoffs);
            searchTimeTransferToDropoff = searchToDropoffsTimer.elapsed<std::chrono::nanoseconds>();

            std::vector<int> dVehIdsVec;
            for (const auto dVehId: dVehIds)
                dVehIdsVec.push_back(dVehId);

            Timer searchToTransfersTimer;
            distancesToTransfer = strategy.calculateDistancesFromLastStopsToAllStops(dVehIdsVec, pVehIds);
            searchTimeLastStopToTransfer = searchToTransfersTimer.elapsed<std::chrono::nanoseconds>();

            std::vector<AssignmentWithTransfer> postponedAssignments;
            for (const auto dVehId: dVehIds) {
                const auto *dVeh = &fleet[dVehId];
                const auto numStopsDVeh = routeState.numStopsOf(dVehId);
                const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

                // Pickup BNS
                for (const auto pVehId: relBNSPickups.getVehiclesWithRelevantPDLocs()) {
                    KASSERT(postponedAssignments.empty());
                    // pVeh an dVeh can not be the same vehicles
                    if (dVehId == pVehId)
                        continue;

                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    for (const auto &dropoff: requestState.dropoffs) {
                        assert(numStopsPVeh - 1 == distancesToTransfer[dVehId][pVehId].size());
                        assert(numStopsPVeh - 1 == distancesToDropoff[pVehId][dropoff.id].size());

                        for (const auto &pickup: relBNSPickups.relevantSpotsFor(pVehId)) {
                            assert(pickup.stopIndex == 0);
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            for (int i = 1; i <
                                            numStopsPVeh; i++) { // We start at i = 1 because at the first stop the transfer is only possible if the vehcle is currently waiting for another passenger, so we neglect this case
                                const bool transferAtLastStop =
                                        stopLocationsPVeh[i] == stopLocationsDVeh[numStopsDVeh - 1];

                                // Construct the transfer point
                                TransferPoint tp = TransferPoint(stopLocationsPVeh[i], pVeh, dVeh);
                                numTransferPoints++;
                                tp.distancePVehToTransfer = 0;
                                tp.distancePVehFromTransfer = 0;
                                tp.distanceDVehToTransfer = distancesToTransfer[dVehId][pVehId][i - 1];
                                tp.distanceDVehFromTransfer = 0;

                                tp.stopIdxPVeh = i;
                                tp.stopIdxDVeh = numStopsDVeh - 1;

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
                                asgn.distToDropoff = distancesToDropoff[pVehId][dropoff.id][i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                assert(asgn.pickupIdx !=
                                       asgn.transferIdxPVeh); // In the pickup bns case we do not have a paired assignment
                                const int lengthOfLeg =
                                        i < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[i + 1] -
                                                               routeState.schedDepTimesFor(pVehId)[i] : 0;
                                asgn.distFromTransferPVeh = lengthOfLeg;

                                asgn.distToTransferDVeh = transferAtLastStop ? 0 : distancesToTransfer[dVehId][pVehId][
                                        i - 1];
                                asgn.distFromTransferDVeh = 0;

                                asgn.pickupType = BEFORE_NEXT_STOP;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                    continue;

                                if (searches.knowsDistance(asgn.pVeh->vehicleId, asgn.pickup->id)) {
                                    asgn.distToPickup = searches.getDistance(asgn.pVeh->vehicleId, asgn.pickup->id);
                                } else {
                                    asgn.pickupBNSLowerBoundUsed = true;
                                }

                                // Try the potentially unfinished assignment with BNS pickup
                                tryPotentiallyUnfinishedAssignment(asgn, postponedAssignments);
                            }
                        }
                    }

                    if (postponedAssignments.empty())
                        continue;

                    finishAssignments(pVeh, postponedAssignments);
                }

                // Pickup ORD
                for (const auto pVehId: relORDPickups.getVehiclesWithRelevantPDLocs()) {
                    const auto *pVeh = &fleet[pVehId];
                    const auto numStopsPVeh = routeState.numStopsOf(pVehId);
                    const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);

                    if (dVehId == pVehId)
                        continue;

                    for (const auto &dropoff: requestState.dropoffs) {
                        //* Calculate the distances from the stops of the pVeh to the possible dropoffs
                        for (const auto &pickup: relORDPickups.relevantSpotsFor(pVehId)) {
                            const auto *pickupPDLoc = &requestState.pickups[pickup.pdId];

                            assert(numStopsPVeh - 1 == distancesToTransfer[dVehId][pVehId].size());
                            assert(numStopsPVeh - 1 == distancesToDropoff[pVehId][dropoff.id].size());

                            for (int i = pickup.stopIndex + 1; i < numStopsPVeh; i++) {
                                assert(pickup.stopIndex > 0);
                                const bool transferAtLastStop =
                                        stopLocationsPVeh[i] == stopLocationsDVeh[numStopsDVeh - 1];
                                assert(pickup.stopIndex != i);

                                // Construct the transfer point
                                TransferPoint tp = TransferPoint(stopLocationsPVeh[i], pVeh, dVeh);
                                numTransferPoints++;
                                tp.distancePVehToTransfer = 0;
                                tp.distancePVehFromTransfer = 0;
                                tp.distanceDVehToTransfer = distancesToTransfer[dVehId][pVehId][i - 1];
                                tp.distanceDVehFromTransfer = 0;

                                tp.stopIdxPVeh = i;
                                tp.stopIdxDVeh = numStopsDVeh - 1;

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
                                asgn.distToDropoff = distancesToDropoff[pVehId][dropoff.id][i - 1];
                                asgn.distFromDropoff = 0;

                                asgn.distToTransferPVeh = 0;
                                const int lengthOfLeg =
                                        i < numStopsPVeh - 1 ? routeState.schedArrTimesFor(pVehId)[i + 1] -
                                                               routeState.schedDepTimesFor(pVehId)[i] : 0;
                                asgn.distFromTransferPVeh = lengthOfLeg;
                                asgn.distToTransferDVeh = transferAtLastStop ? 0 : distancesToTransfer[dVehId][pVehId][
                                        i - 1];
                                asgn.distFromTransferDVeh = 0;

                                asgn.pickupType = ORDINARY;
                                asgn.transferTypePVeh = ORDINARY;
                                asgn.transferTypeDVeh = AFTER_LAST_STOP;
                                asgn.dropoffType = AFTER_LAST_STOP;

                                // If the pickup or dropoff conincides with the transfer, we skip the assignment
                                if (asgn.pickup->loc == asgn.transfer.loc || asgn.transfer.loc == asgn.dropoff->loc)
                                    continue;

                                // Try the finished assignment with ORD pickup
                                tryFinishedAssignment(asgn);
                            }
                        }
                    }
                }
            }
        }

        // Skip unecessary assignments (e.g. if the pickup or dropoff is already at the next stop)
        bool canSkipAssignment(const AssignmentWithTransfer &asgn) const {
            const int numStopsPVeh = routeState.numStopsOf(asgn.pVeh->vehicleId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(asgn.pVeh->vehicleId);
            return (asgn.pickupIdx < numStopsPVeh - 1 && asgn.pickup->loc == stopLocationsPVeh[asgn.pickupIdx + 1])
                   || (asgn.transferIdxPVeh < numStopsPVeh - 1 &&
                       asgn.transfer.loc == stopLocationsPVeh[asgn.transferIdxPVeh + 1]);
        }

        void trackAssignmentTypeStatistic(const AssignmentWithTransfer &asgn) {
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
        }

        void tryFinishedAssignment(AssignmentWithTransfer &asgn) {
            if (canSkipAssignment(asgn))
                return;

            trackAssignmentTypeStatistic(asgn);

            Timer time;
            const auto cost = calc.calc(asgn, requestState);
            requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void tryPotentiallyUnfinishedAssignment(AssignmentWithTransfer &asgn,
                                                std::vector<AssignmentWithTransfer>& postponedAssignments) {
            if (canSkipAssignment(asgn))
                return;

            trackAssignmentTypeStatistic(asgn);

            Timer time;
            if (!asgn.isFinished()) {
//                const auto cost = calc.calcLowerBound(asgn, requestState);
                const auto cost = calc.calc(asgn, requestState);
                if (cost.total <= requestState.getBestCost()) {
                    postponedAssignments.push_back(asgn);
                }
            } else {
                const auto cost = calc.calc(asgn, requestState);
                requestState.tryFinishedTransferAssignmentWithKnownCost(asgn, cost);
            }
            tryAssignmentsTime += time.elapsed<std::chrono::nanoseconds>();
        }

        void finishAssignments(const Vehicle *pVeh, std::vector<AssignmentWithTransfer> &postponedAssignments) {
            for (const auto &asgn: postponedAssignments) {
                assert(asgn.pickupBNSLowerBoundUsed);
                searches.addPickupForProcessing(asgn.pickup->id, asgn.distToPickup);
            }

            searches.computeExactDistancesVia(*pVeh);

            for (auto& asgn: postponedAssignments) {
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

        CurVehLocToPickupSearchesT &searches;

        const RelevantPDLocs &relORDPickups;
        const RelevantPDLocs &relBNSPickups;

        const Fleet &fleet;
        const RouteState &routeState;
        RequestState &requestState;
        CostCalculator calc;

        std::map<int, std::map<int, std::vector<int>>> distancesToTransfer;
        std::map<int, std::map<int, std::vector<int>>> distancesToDropoff;

        InsertionAsserterT &asserter;

        //* Statistics for the transfer als dveh assignment finder
        int64_t totalTime;

        // Stats for the PD Locs

        int64_t numPickups;
        int64_t numDropoffs;

        // Stats for the tried assignments 
        int64_t numAssignmentsTriedPickupBNS;
        int64_t numAssignmentsTriedPickupORD;

        int64_t numAssignmentsTriedDropoffALS;

        int64_t tryAssignmentsTime;

        // Stats for the transfer search itself
        int64_t numTransferPoints;

        int64_t searchTimeLastStopToTransfer;
        int64_t searchTimeTransferToDropoff;

    };
}
