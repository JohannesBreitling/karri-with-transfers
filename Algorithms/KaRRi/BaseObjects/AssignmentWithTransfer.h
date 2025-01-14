/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2024 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include <cassert>

#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"

#include "Algorithms/KaRRi/BaseObjects/RequestCost.h"

namespace karri {

    struct AssignmentWithTransfer {
        
        AssignmentWithTransfer() {}
        // AssignmentWithTransfer(const AssignmentWithTransfer&) = default;

        /*
        AssignmentWithTransfer(const AssignmentWithTransfer &asgn) {
            pVeh = asgn.pVeh;
            pickup = asgn.pickup;
            transfer = asgn.transfer;
            pickupBNSLowerBoundUsed = asgn.pickupBNSLowerBoundUsed;
            pickupPairedLowerBoundUsed = asgn.pickupPairedLowerBoundUsed;
            pickupIdx = asgn.pickupIdx;
            transferIdxPVeh = asgn.transferIdxPVeh;
            transferIdxDVeh = asgn.transferIdxDVeh;

            distToPickup = asgn.distToPickup;
            distFromPickup = asgn.distFromPickup;
            distToTransferPVeh = asgn.distToTransferPVeh;
            distFromTransferPVeh = asgn.distFromTransferPVeh;
            
            distToTransferDVeh = asgn.distToTransferDVeh;
            distFromTransferDVeh = asgn.distFromTransferDVeh;

            costPVeh = asgn.costPVeh;
            waitTimeAtPickup = asgn.waitTimeAtPickup;
            arrAtTransferPoint = asgn.arrAtTransferPoint;

            dVeh = asgn.dVeh;
            dropoff = asgn.dropoff;
            cost = asgn.cost;
            dropoffIdx = asgn.dropoffIdx;

            dropoffBNSLowerBoundUsed = asgn.dropoffBNSLowerBoundUsed;
            dropoffPairedLowerBoundUsed = asgn.dropoffPairedLowerBoundUsed;

            distToDropoff = asgn.distToDropoff;
            distFromDropoff = asgn.distFromDropoff;
        }
        */

        const Vehicle *pVeh = nullptr;
        const Vehicle *dVeh = nullptr;

        const PDLoc *pickup = nullptr;
        TransferPoint transfer;
        const PDLoc *dropoff = nullptr;

        RequestCost cost;

        int pickupIdx = INVALID_INDEX;
        int transferIdxPVeh = INVALID_INDEX;
        int transferIdxDVeh = INVALID_INDEX;
        int dropoffIdx = INVALID_INDEX;

        bool pickupBNSLowerBoundUsed = false;
        bool pickupPairedLowerBoundUsed = false;
        bool dropoffBNSLowerBoundUsed = false;
        bool dropoffPairedLowerBoundUsed = false;
        
        int distToPickup; // distance from previous stop to pickup
        int distFromPickup; // distance from pickup to next stop (or 0 if pickupIdx == transferIdxPVeh)
        int distToTransferPVeh; // distance from previous stop to transfer point (or from transfer point if pickupIdx == transferIdxPVeh) 
        int distFromTransferPVeh; // distance from transfer point to next stop (or 0 if there is no next stop)
        
        int distToTransferDVeh; // distance from previous stop to transfer point
        int distFromTransferDVeh; // distance from transfer point to next stop (or 0 if transferIdxDVeh == dropoffIdx)
        int distToDropoff; // distance from previous stop to dropoff (or from transfer point if transferIdxDVeh == dropoffIdx)
        int distFromDropoff; // distance from dropoff to next stop (or 0 if there is no next stop)

        int costPVeh; // Cost of the trip until the passenger arrives at the transfer point
        int waitTimeAtPickup; // Wait time at pickup
        int arrAtTransferPoint; // Arrival time of pVeh at transfer point

    };

}