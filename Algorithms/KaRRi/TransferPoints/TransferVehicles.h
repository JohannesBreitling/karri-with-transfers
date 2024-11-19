/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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

                // TODO WARUM?????

                ordPDs = std::map<int, std::vector<PD>>{};
                bnsPDs = std::map<int, std::vector<PD>>{};
                alsPDs = std::map<int, std::vector<PD>>{};
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