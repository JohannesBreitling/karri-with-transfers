
#pragma once

#include "Tools/Constants.h"
#include "BaseObjects/Vehicle.h"

namespace karri {
    
    
    struct TransferPoint {

        const Vehicle* pVeh = nullptr;
        const Vehicle* pVeh = nullptr;

        int dropoffAtTransferStopIdx = INVALID_INDEX;
        int pickupFromTransferStopIdx = INVALID_INDEX;
       
        const int loc = INVALID_EDGE; // Location in the road network
        
        int distancePVehToTransfer = 0;
        int distancePVehFromTransfer = 0;
        int distanceDVehToTransfer = 0;
        int distanceDVehFromTransfer = 0;

    };


}