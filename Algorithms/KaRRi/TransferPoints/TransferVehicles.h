
#pragma once

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"
#include "Algorithms/KaRRi/BaseObjects/PD.h"
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

            void pushBack(const Vehicle *v , std::vector<PD> relevantPDs, PDType type) {
                if (!containedKeys.count(v->vehicleId)) {
                    vehicles.push_back(v);
                    containedKeys[v->vehicleId] = vehicles.size() - 1;
                }

                switch (type) {
                    case ORD:
                        ordPDs[v->vehicleId] = relevantPDs;
                        break;
                    case BNS:
                        bnsPDs[v->vehicleId] = relevantPDs;
                        break;
                    case ALS:
                        alsPDs[v->vehicleId] = relevantPDs;
                        break;
                }
            }

            std::vector<PD> getOrdPDsForVehicle(int vehId) {
                return ordPDs[vehId];
            }

            std::vector<PD> getBnsPDsForVehicle(int vehId) {
                return bnsPDs[vehId];
            }

            std::vector<PD> getAlsPDsForVehicle(int vehId) {
                return alsPDs[vehId];
            }
 
        private:
            std::vector<const Vehicle*> vehicles;

            std::map<int, std::vector<PD>> ordPDs;
            std::map<int, std::vector<PD>> bnsPDs;
            std::map<int, std::vector<PD>> alsPDs;

            std::map<int, int> containedKeys;
    };

    using PickupVehicles = TransferVehicleCollection;
    using DropoffVehicles = TransferVehicleCollection;

}