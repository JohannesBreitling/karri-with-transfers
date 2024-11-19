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

#include "Vehicle.h"
#include "Request.h"
#include "PD.h"
#include "Algorithms/KaRRi/TransferPoints/TransferPoint.h"

namespace karri {

    struct AssignmentWithTransfer {
        
        AssignmentWithTransfer() {}

        const Vehicle *pVeh = nullptr;
        const Vehicle *dVeh = nullptr;

        const PDLoc *pickup = nullptr;
        const TransferPoint transfer;
        const PDLoc *dropoff = nullptr;

        int pickupIdx = INVALID_INDEX;
        int transferIdxPVeh = INVALID_INDEX;
        int transferIdxDVeh = INVALID_INDEX;
        int dropoffIdx = INVALID_INDEX;
        
        int distToPickup; // distance from previous stop to pickup
        int distFromPickup; // distance from pickup to next stop (or 0 if pickupIdx == transferIdxPVeh)
        int distToTransferPVeh; // distance from previous stop to transfer point (or from transfer point if pickupIdx == transferIdxPVeh) 
        int distFromTransferPVeh; // distance from transfer point to next stop (or 0 if there is no next stop)
        
        int distToTransferDVeh; // distance from previous stop to transfer point
        int distFromTransferDVeh; // distance from transfer point to next stop (or 0 if transferIdxDVeh == dropoffIdx)
        int distToDropoff; // distance from previous stop to dropoff (or from transfer point if transferIdxDVeh == dropoffIdx)
        int distFromDropoff; // distance from dropoff to next stop (or 0 if there is no next stop)

    };

}