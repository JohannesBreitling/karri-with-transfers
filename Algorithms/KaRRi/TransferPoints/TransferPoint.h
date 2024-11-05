
#pragma once

#include "Tools/Constants.h"
#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {
    
    
    struct TransferPoint {

        TransferPoint() {} 

        Vehicle* pVeh = nullptr;
        Vehicle* dVeh = nullptr;

        int dropoffAtTransferStopIdx = INVALID_INDEX;
        int pickupFromTransferStopIdx = INVALID_INDEX;
       
        int loc = INVALID_EDGE; // Location in the road network
        
        int distancePVehToTransfer = 0;
        int distancePVehFromTransfer = 0;
        int distanceDVehToTransfer = 0;
        int distanceDVehFromTransfer = 0;

    };


}