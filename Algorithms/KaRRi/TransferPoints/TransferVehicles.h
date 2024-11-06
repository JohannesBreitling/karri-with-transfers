
#pragma once

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/RequestState/RelevantPDLocs.h"

namespace karri {

    class TransferVehicleCollection {

        using RelevantPDLoc = RelevantPDLocs::RelevantPDLoc;

        public:
            TransferVehicleCollection() {
                init();
            }

            void init() {
                containedKeys = std::map<int, int>{};
                vehicles = std::vector<const Vehicle*>{}; 
            }

            std::vector<const Vehicle*> *getVehicles() {
                return &vehicles;
            }

            void pushBack(const Vehicle *v , std::vector<RelevantPDLoc> relevantPDLocs) {
                if (!containedKeys.count(v->vehicleId)) {
                    vehicles.push_back(v);
                    relevantLocs[v->vehicleId] = relevantPDLocs;
                    containedKeys[v->vehicleId] = vehicles.size() - 1;
                }
            }

        private:
            std::vector<const Vehicle*> vehicles;
            std::map<int, std::vector<RelevantPDLoc>> relevantLocs;
            std::map<int, int> containedKeys;
    };

    using PickupVehicles = TransferVehicleCollection;
    using DropoffVehicles = TransferVehicleCollection;

}