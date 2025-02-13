
#pragma once

namespace karri {
    
    class RequestCost {

        public:
            int total;

            int walkingCost;
            int tripCost;
            int waitTimeViolationCost;
            int changeInTripCostsOfOthers;
            int vehCost;

            bool error = false;

            static RequestCost INFTY_COST() {
                RequestCost cost;

                cost.total = INFTY;
                cost.tripCost = INFTY;
                cost.waitTimeViolationCost = INFTY;
                cost.changeInTripCostsOfOthers = INFTY;
                cost.vehCost = INFTY;
                cost.walkingCost = INFTY;

                return cost;
            }
    };

}