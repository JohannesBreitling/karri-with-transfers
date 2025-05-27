
#pragma once

namespace karri {
    
    class RequestCost {

        public:

            RequestCost() :
            total(INFTY),
            walkingCost(INFTY),
            tripCost(INFTY),
            waitTimeViolationCost(INFTY),
            changeInTripCostsOfOthers(INFTY),
            vehCost(INFTY),
            arrAtDropoff(INFTY) {}

            int total;

            int walkingCost;
            int tripCost;
            int waitTimeViolationCost;
            int changeInTripCostsOfOthers;
            int vehCost;

            int arrAtDropoff;

            static RequestCost INFTY_COST() {
                RequestCost cost;

                cost.total = INFTY;
                cost.tripCost = INFTY;
                cost.waitTimeViolationCost = INFTY;
                cost.changeInTripCostsOfOthers = INFTY;
                cost.vehCost = INFTY;
                cost.walkingCost = INFTY;
                cost.arrAtDropoff = INFTY;

                return cost;
            }
    };

}