#pragma once

#include "Algorithms/KaRRi/BaseObjects/Vehicle.h"

namespace karri {

    class OrdinaryTransferFinder {
        
    // Both vehicles can drive a detour to the transfer point, but none drives the detour ALS
    // This implies, that the pickup is BNS or ORD, the dropoff can be BNS, ORD or ALS
    // If the dropoff is ALS, we need to consider a different set of vehicles to calculate the transfer points between
    public:
        
        void findAssignments() {
            findAssignmentsWihtDropoffORDorBNS();
            findAssignmentsWithDropoffALS();
        }

    private:
        
        void findAssignmentsWithDropoffALS() {
            //* Calculate the transfer points for the doropoff ALS 
        }

        void findAssignmentsWihtDropoffORDorBNS() {
            //* Calculate the transfer points for the dropoff ORD or BNS
        }

        

        // std::vector<Vehicle> pVehsORD;
        // std::vector<Vehicle> pVehsBNS;
        // We dont consider ALS for the pickuü because this would imply a transfer ALS, which is handled in the TransferALSPVehFinder

        // std::vector<Vehicle&> dVehsORD;
        // std::vector<Vehicle&> dVehsALS; ???

    };

}
