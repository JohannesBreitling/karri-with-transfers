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
#include "TransferPoints/TransferPoint.h"

namespace karri {

    struct AssignmentWithTransfer {

        const Vehicle* pVehicle = nullptr;
        const Vehicle* dVehicle = nullptr;
        const PDLoc *pickup = nullptr;
        const PDLoc *dropoff = nullptr;

        TransferPoint* transfer = nullptr;

        const int transferpoint = INVALID_INDEX;

        int pickupStopIdx = INVALID_INDEX;  // Pickup is inserted at or after stop with index pickupStopIdx in route of pickup vehicle
        int dropoffStopIdx = INVALID_INDEX;  // Dropoff is inserted at or after stop with index dropoffStopIdx in route of dropoff vehicle

        int distToPickup = 0; // Distance from previous stop to pickup
        int distFromPickup = 0; // Distance from pickup to next stop (or 0 if pickupStopIdx == dropoffStopIdx)
        int distToDropoff = 0; // Distance from previous stop to dropoff (or from pickup to dropoff if pickupStopIdx == dropoffStopIdx)
        int distFromDropoff = 0; // Distance from dropoff to next stop

    };


}