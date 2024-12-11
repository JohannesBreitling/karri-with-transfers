
#pragma once

namespace karri {
    
    struct RequestCost {
        int total;

        int walkingCost;
        int tripCost;
        int waitTimeViolationCost;
        int changeInTripCostsOfOthers;
        int vehCost;
    };
    
}