
#pragma once

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {

    // Type of the pickup / dropoff (ordinary, before next stop, after last stop)
    enum PDType {
        ORD,
        BNS,
        ALS
    };


    struct PD {

        PD(const Vehicle* veh) : vehicle(veh) {}

        // Type of the pickup / dropoff
        PDType type;
        // Vehicle that does the pickup / dropoff
        const Vehicle *vehicle;
        // Index after which the pickup / dropoff will be performed
        int pdIdx;
        // Walking distance from origin to pickup / dropoff to destination 
        int walkingDistance;
        // Detour of vehicle to get to the pickup / dropoff
        int detourToPD;
        // Detour of vehicle to get from pickup / dropoff to next scheduled stop
        int detourFromPD;

    };

    using Pickup = PD;
    using Dropoff = PD;
    
}


