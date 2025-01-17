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

        int getMaxDepTimeAtPickup() const {
            return originalRequest.requestTime + InputConfig::getInstance().maxWaitTime;
        }

        int getMaxDepTimeAtTransfer(const AssignmentWithTransfer &asgn) const {
            assert(asgn.waitTimeAtPickup <= InputConfig::getInstance().maxWaitTime);

            return asgn.arrAtTransferPoint + InputConfig::getInstance().maxWaitTime - asgn.waitTimeAtPickup;
        }

        // Information about best known assignment for current request

        // TODO Just for testing purpose (s. t. AssignmentWithTransfer is the best assignment and will be inserted)
        void clearAssignment() {
            bestCost = INFTY;
            bestAssignment = Assignment();           
        }

        const Assignment &getBestAssignment() const {
            return bestAssignment;
        }

        const AssignmentWithTransfer &getBestAssignmentWithTransfer() const {
            return bestAssignmentWithTransfer;
        }

        void tryAssignmentWithTransfer(const AssignmentWithTransfer &asgn) {
            
            if (asgn.cost.total >= bestCostWithTransfer) {
                return;
            }
            
            bestCostWithTransfer = asgn.cost.total;
            bestAssignmentWithTransfer = AssignmentWithTransfer(asgn);
        }

        bool improvementThroughTransfer() const {
            return bestCostWithTransfer < bestCost;
        }

        const int &getBestCost() const {
            return bestCost;
        }

        const int &getBestCostWithTransfer() const {
            return bestCostWithTransfer;
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

        bool tryAssignmentWithKnownCost(const Assignment &asgn, const int cost) {
            assert(calculator.calc(asgn, *this) == cost);

            if (cost < INFTY && (cost < bestCost || (cost == bestCost &&
                                    breakCostTie(asgn, bestAssignment)))) {

                bestAssignment = asgn;
                bestCost = cost;
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
            bestCost = INFTY;
            bestCostWithTransfer = INFTY;
            notUsingVehicleIsBest = false;
            notUsingVehicleDist = INFTY;
            bestAssignmentWithTransfer = AssignmentWithTransfer();
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
        bool notUsingVehicleIsBest;
        int notUsingVehicleDist;
        
    };
}