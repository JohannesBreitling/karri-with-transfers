/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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


#include "Algorithms/KaRRi/BaseObjects/Request.h"
#include "Algorithms/KaRRi/Stats/PerformanceStats.h"
#include "Algorithms/KaRRi/Stats/OsmRoadCategoryStats.h"
#include "Algorithms/KaRRi/BaseObjects/Assignment.h"
#include "Algorithms/KaRRi/InputConfig.h"
#include "Tools/Simd/AlignedVector.h"
#include "DataStructures/Containers/Subset.h"
#include "Algorithms/KaRRi/CostCalculator.h"
#include "Algorithms/KaRRi/RequestState/FindPDLocsInRadiusQuery.h"

namespace karri {

    // Holds information relating to a specific request like its pickups and dropoffs and the best known assignment.
    struct RequestState {

        RequestState(const CostCalculator &calculator)
                : originalRequest(),
                  originalReqDirectDist(-1),
                  minDirectPDDist(-1),
                  pickups(),
                  dropoffs(),
                  calculator(calculator) {}


        ~RequestState() {
            auto &roadCatLogger = LogManager<std::ofstream>::getLogger(karri::stats::OsmRoadCategoryStats::LOGGER_NAME,
                                                                       "type," +
                                                                       karri::stats::OsmRoadCategoryStats::getLoggerCols());
            roadCatLogger << "all_pd_locs, " << allPDLocsRoadCatStats.getLoggerRow() << "\n";
            roadCatLogger << "chosen_pd_locs, " << chosenPDLocsRoadCatStats.getLoggerRow() << "\n";
        }


        // Information about current request itself
        Request originalRequest;
        int originalReqDirectDist;
        int minDirectPDDist;

        std::vector<PDLoc> pickups;
        std::vector<PDLoc> dropoffs;

        int numPickups() const {
            return pickups.size();
        }

        int numDropoffs() const {
            return dropoffs.size();
        }

        // Shorthand for requestTime
        int now() const {
            return originalRequest.requestTime;
        }

        int getOriginalReqMaxTripTime() const {
            assert(originalReqDirectDist >= 0);
            return static_cast<int>(InputConfig::getInstance().alpha * static_cast<double>(originalReqDirectDist)) + InputConfig::getInstance().beta;
        }

        int getPassengerArrAtPickup(const int pickupId) const {
            assert(pickupId < numPickups());
            return originalRequest.requestTime + pickups[pickupId].walkingDist;
        }

        int getMaxPDTripTime(const int pickupId, const int dropoffId) const {
            assert(pickupId < numPickups() && dropoffId < numDropoffs());
            assert(originalReqDirectDist >= 0);
            return getOriginalReqMaxTripTime() - (pickups[pickupId].walkingDist + dropoffs[dropoffId].walkingDist);
        }

        int getMaxArrTimeAtDropoff(const int pickupId, const int dropoffId) const {
            assert(pickupId < numPickups() && dropoffId < numDropoffs());
            return getPassengerArrAtPickup(pickupId) + getMaxPDTripTime(pickupId, dropoffId);
        }

        int getMaxArrTimeAtTransfer(const AssignmentWithTransfer &asgn) const {
            assert(asgn.pickup->id < numPickups() && asgn.dropoff->id < numDropoffs());
            return getMaxArrTimeAtDropoff(asgn.pickup->id, asgn.dropoff->id) - asgn.tripTimeDVeh;
        }

        int getMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        int getMaxDepTimeAtTransfer(const AssignmentWithTransfer &asgn) const {
            int delta = InputConfig::getInstance().maxWaitTime - asgn.waitTimeAtPickup;
            return asgn.arrAtTransferPoint + std::max(0, delta);
        }

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        const AssignmentWithTransfer &getBestAssignmentWithTransfer() const {
            return bestAssignmentWithTransfer;
        }

        void tryFinishedTransferAssignmentWithKnownCost(AssignmentWithTransfer &asgn, const RequestCost& cost) {
            KASSERT(asgn.isFinished());
            KASSERT(asgn.pVeh->vehicleId >= 0 && asgn.dVeh->vehicleId >= 0 && asgn.pVeh->vehicleId != asgn.dVeh->vehicleId && asgn.pickup && asgn.dropoff);
            KASSERT(asgn.distToPickup >= 0 && asgn.distFromPickup >= 0 && asgn.distToTransferPVeh >= 0 && asgn.distFromTransferPVeh >= 0 && asgn.distToTransferDVeh >= 0 && asgn.distFromTransferDVeh >= 0 && asgn.distToDropoff >= 0 && asgn.distFromDropoff >= 0);
            KASSERT(asgn.pickupIdx == asgn.transferIdxPVeh || asgn.distFromPickup > 0 || asgn.pickupType == AFTER_LAST_STOP);
            KASSERT(asgn.transferIdxDVeh == asgn.dropoffIdx || asgn.distFromTransferDVeh > 0 || asgn.transferTypeDVeh == AFTER_LAST_STOP);
            KASSERT(asgn.pickup->loc != asgn.transfer.loc && asgn.dropoff->loc != asgn.transfer.loc);

            KASSERT(cost == calculator.calc(asgn, *this));

            if (cost.total < bestCostWithTransfer) {
                asgn.maxDepAtPickup = getMaxDepTimeAtPickup();
                bestAssignmentWithTransfer = AssignmentWithTransfer(asgn);
                bestCostWithTransfer = cost.total;
                bestCostObjectWT = cost;
                notUsingVehicleIsBest = false;
                notUsingVehicleDist = INFTY;
            }
//
//            // Calculate the cost of the assignment and try to update the best known assignment if the assignment is finished
//            RequestCost cost;
//            if (!asgn.isFinished()) {
//                cost = calculator.calcLowerBound(asgn, *this);
//            } else {
//                calculator.recomputePVeh(asgn, *this);
//                cost = calculator.calc(asgn, *this);
//            }
//
//            if (cost.total >= INFTY)
//                return;
//
//            if (asgn.isFinished() && cost.total < bestCostWithTransfer) {
//                asgn.maxDepAtPickup = getMaxDepTimeAtPickup();
//                bestAssignmentWithTransfer = AssignmentWithTransfer(asgn);
//                bestCostWithTransfer = cost.total;
//                bestCostObjectWT = cost;
//                notUsingVehicleIsBest = false;
//                notUsingVehicleDist = INFTY;
//            } else if (cost.total < bestCostWithTransfer) {
//                postponedAssignments.push_back(asgn);
//            }
        }
        
        bool improvementThroughTransfer() const {
            return bestCostWithTransfer < bestCost;
        }

        const RequestCost getCostObjectWithoutTransfer() const {
            return bestCostObjectWOT;
        }

        const RequestCost getCostObjectWithTransfer() const {
            return bestCostObjectWT;
        }

        const int &getBestCostWithTransfer() const {
            return bestCostWithTransfer;
        }

        const int &getBestCost() const {
            return std::min(bestCost, bestCostWithTransfer);
        }

        bool isNotUsingVehicleBest() const {
            return notUsingVehicleIsBest;
        }

        const int &getNotUsingVehicleDist() const {
            return notUsingVehicleDist;
        }

        bool tryAssignment(const Assignment &asgn) {
            const auto cost = calculator.calc(asgn, *this);
            return tryAssignmentWithKnownCost(asgn, cost);
        }

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const RequestCost cost) {
            assert(calculator.calc(asgn, *this).total == cost.total);

            if (cost.total < INFTY && (cost.total < bestCost || (cost.total == bestCost &&
                                    breakCostTie(asgn, bestAssignment)))) {

                
                bestAssignment = asgn;
                bestCost = cost.total;
                bestCostObjectWOT = cost;
                notUsingVehicleIsBest = false;
                notUsingVehicleDist = INFTY;
                return true;
            }
            
            return false;
        }

        void tryNotUsingVehicleAssignment(const int notUsingVehDist, const int travelTimeOfDestEdge) {
            const int cost = CostCalculator::calcCostForNotUsingVehicle(notUsingVehDist, travelTimeOfDestEdge, *this);
            if (cost < bestCost) {
                bestAssignment = Assignment();
                bestCost = cost;
                notUsingVehicleIsBest = true;
                notUsingVehicleDist = notUsingVehDist;
            }
        }

        stats::DispatchingPerformanceStats &stats() {
            return perfStats;
        }

        const stats::DispatchingPerformanceStats &stats() const {
            return perfStats;
        }

        stats::OsmRoadCategoryStats &allPDLocsRoadCategoryStats() {
            return allPDLocsRoadCatStats;
        }

        stats::OsmRoadCategoryStats &chosenPDLocsRoadCategoryStats() {
            return chosenPDLocsRoadCatStats;
        }

        void reset() {
            perfStats.clear();

            originalRequest = {};
            originalReqDirectDist = INFTY;
            minDirectPDDist = INFTY;
            pickups.clear();
            dropoffs.clear();

            bestAssignment = Assignment();
            bestAssignmentWithTransfer = AssignmentWithTransfer();
            bestCost = INFTY;
            bestCostWithTransfer = INFTY;
            notUsingVehicleIsBest = false;
            notUsingVehicleDist = INFTY;

            bestCostObjectWOT = RequestCost::INFTY_COST();
            bestCostObjectWT = RequestCost::INFTY_COST();
        }

    private:

        stats::DispatchingPerformanceStats perfStats;
        stats::OsmRoadCategoryStats allPDLocsRoadCatStats;
        stats::OsmRoadCategoryStats chosenPDLocsRoadCatStats;

        const CostCalculator &calculator;

        // Information about best known assignment for current request
        Assignment bestAssignment;
        AssignmentWithTransfer bestAssignmentWithTransfer;

        int bestCost;
        int bestCostWithTransfer;

        RequestCost bestCostObjectWT;
        RequestCost bestCostObjectWOT;

        bool notUsingVehicleIsBest;
        int notUsingVehicleDist;
    };
}