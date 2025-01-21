
#pragma once

#include "Tools/Constants.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {

    struct TransferPoint {
        TransferPoint() {}
        TransferPoint(const Vehicle *pVeh, const Vehicle *dVeh) : pVeh(pVeh), dVeh(dVeh) {}
        TransferPoint(const TransferPoint&) = default;

        const Vehicle *pVeh = nullptr;
        const Vehicle *dVeh = nullptr;

        int dropoffAtTransferStopIdx = INVALID_INDEX;
        int pickupFromTransferStopIdx = INVALID_INDEX;
       
        int loc = INVALID_EDGE; // Location in the road network
        
        int distancePVehToTransfer = -1;
        int distancePVehFromTransfer = -1;
        int distanceDVehToTransfer = -1;
        int distanceDVehFromTransfer = -1;

    };

}