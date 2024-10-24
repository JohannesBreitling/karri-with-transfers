

#pragma once

namespace karri {

    
    class TransferPointFinder {

    public:
        TransferPointFinder(
            PickupVehicles &pVehs,
            DropoffVehicles &dVehs
        ) : pVehs(pVehs), dVehs(dVehs) {}

        void calculateTransferPoints() {

        }

    private:
        PickupVehicles &pVehs;
        DropoffVehicles &dVehs;
    
    
    }

















}