



#pragma once

namespace karri {


template <typename TransferPointPickerT, typename IntermediateAssignmentFinderT, typename RequestStateInitializerT>
class HeuristicTransferFinder {

    public:
        
        HeuristicTransferFinder(
            RequestState &reqState,
            RequestState &intermediateReqState,
            const RouteState &routeState,
            RequestStateInitializerT &requestStateInitializer,
            TransferPointPickerT &transferPointPicker,
            IntermediateAssignmentFinderT &intermediateAssignmentFinder
        ) : reqState(reqState),
            intermediateReqState(intermediateReqState),
            routeState(routeState),
            requestStateInitializer(requestStateInitializer),
            transferPointPicker(transferPointPicker),
            intermediateAssignmentFinder(intermediateAssignmentFinder),
            runningIntermediateId(1000000) {}

        void findAssignments() {
            
            // Get the transfer points for the request
            std::vector<int> transferPointLocations = transferPointPicker.pickTransferPointLocations(reqState.pickups, reqState.dropoffs, 100);
            intermediateReqState.reset();
            transferPDLocs.clear();

            const int cost = reqState.getBestCost();
            
            int bestTotalCost = INFTY;

            const auto origReq = reqState.originalRequest;

            // Find the best assignment with one heuristic transfer for every transfer point
            for (const auto &tp : transferPointLocations) {
                
                if (tp == origReq.origin || tp == origReq.destination)
                    continue;
                
                // Calculate the intermediate assignments
                // Pickup - Transfer
                Request intermediatePickupRequest;
                intermediatePickupRequest.requestId = runningIntermediateId++;
                intermediatePickupRequest.origin = origReq.origin;
                intermediatePickupRequest.destination = tp;
                intermediatePickupRequest.requestTime = origReq.requestTime;
                intermediatePickupRequest.numRiders = origReq.numRiders;
                requestStateInitializer.initializeIntermediatePickupState(intermediatePickupRequest); // TODO: Problem we want a pickup from the actual request finder 

                // std::cout << "P(" << intermediateReqState.pickups[0].loc << ")\n";
                // std::cout << "O(" << intermediatePickupRequest.origin << ")\n";
                // std::cout << "D(" << intermediatePickupRequest.destination << ")\n";
                // std::cout << "TP(" << tp << ")\n";
                
                // Find the first half of the assignment, run a normal karri query for that
                const auto &intermediatePickupResponse = intermediateAssignmentFinder.findBestAssignment();
                Assignment intermediatePickupAssignment = intermediatePickupResponse.getBestAssignment();

                assert(intermediatePickupAssignment.pickup->id == 0);
                assert(intermediatePickupAssignment.dropoff->id == 0);

                const int arrAtTransfer = intermediatePickupAssignment.cost.arrAtDropoff;

                // std::cout << "AP(" << intermediatePickupAssignment.pickup->loc << ")\n";
                // std::cout << "AD(" << intermediatePickupAssignment.dropoff->loc  << ")\n";
                
                if (intermediatePickupAssignment.cost.total >= bestTotalCost || intermediatePickupAssignment.pickup->loc == tp)
                    continue;

                // Save the transfer PD Locs
                int i = transferPDLocs.size();

                PDLoc pickup(*intermediatePickupAssignment.pickup);
                PDLoc transferPVeh(*intermediatePickupAssignment.dropoff);

                const int pickupIdx = transferPDLocs.size();
                transferPDLocs.push_back(pickup);
                intermediatePickupAssignment.pickup = &transferPDLocs[i++];
                assert(intermediatePickupAssignment.pickup->id == 0);
                transferPDLocs.push_back(transferPVeh);
                intermediatePickupAssignment.dropoff = &transferPDLocs[i++];
                assert(intermediatePickupAssignment.dropoff->id == 0);

                // Transfer - Dropoff
                Request intermediateTransferRequest;
                intermediateTransferRequest.requestId = runningIntermediateId++;
                intermediateTransferRequest.origin = tp;
                intermediateTransferRequest.destination = origReq.destination;
                intermediateTransferRequest.requestTime = arrAtTransfer;
                intermediateTransferRequest.numRiders = origReq.numRiders;
                
                requestStateInitializer.initializeIntermediateTransferState(intermediateTransferRequest); // TODO: Problem we want a dropoff from the actual request finder
                
                // Find the second half of the assignment, run a normal karri query for that
                const auto &intermediateTransferResponse = intermediateAssignmentFinder.findBestAssignment();
                Assignment intermediateTransferAssignment = intermediateTransferResponse.getBestAssignment();

                PDLoc transferDVeh(*intermediateTransferAssignment.pickup);
                PDLoc dropoff(*intermediateTransferAssignment.dropoff);

                transferPDLocs.push_back(transferDVeh);
                intermediateTransferAssignment.pickup = &transferPDLocs[i++];
                const int dropoffIdx = transferPDLocs.size();
                transferPDLocs.push_back(dropoff);
                intermediateTransferAssignment.dropoff = &transferPDLocs[i];
                
                if (intermediatePickupAssignment.vehicle->vehicleId == intermediateTransferAssignment.vehicle->vehicleId)
                    continue; // TODO: We acutally have to adjust the set of possible dropoff vehicles, the pickup vehicle should not be part of it
                
                if (tp == intermediateTransferAssignment.dropoff->loc)
                    continue;
                
                const int currentTotal = intermediatePickupAssignment.cost.total + intermediateTransferAssignment.cost.total;

                if (currentTotal < bestTotalCost) {
                    //assert(intermediatePickupAssignment.pickup->id == 0 && intermediateTransferAssignment.dropoff->id == 0);

                    auto asgnWithTransfer = buildAssignment(intermediatePickupAssignment, intermediateTransferAssignment, tp, &transferPDLocs[pickupIdx], &transferPDLocs[dropoffIdx]);

                    assert(asgnWithTransfer.pickup->id == 0 && asgnWithTransfer.dropoff->id == 0);

                    reqState.tryAssignment(asgnWithTransfer);
                    
                    if (asgnWithTransfer.cost.total < bestTotalCost)
                        bestTotalCost = currentTotal;
                }
            }

            if (bestTotalCost >= INFTY)
                return;

            // std::cout << "Old cost: " << cost << " ";
            // std::cout << "Best Cost (Calc): " << bestTotalCost;
            // std::cout << " Costs higher by " << ((bestTotalCost - cost) * 100) / cost << "%" << "\n";
            
            if (bestTotalCost < cost)
                std::cout << "Improvement through heuristic transfer!\n";
        }


    private:

        AssignmentWithTransfer buildAssignment(const Assignment &asgnPickup, const Assignment &asgnTransfer, const int transferLoc, const PDLoc* pickup, const PDLoc* dropoff) {
            assert(pickup->id == 0);
            assert(dropoff->id == 0);

            // Construct the assignment with transfer from the two intermediate assignments
            const Vehicle* pVeh = asgnPickup.vehicle;
            const Vehicle* dVeh = asgnTransfer.vehicle;

            const int distToPickup = asgnPickup.distToPickup; 
            const int distFromPickup = asgnPickup.distFromPickup;
            const int distToTransferPVeh = asgnPickup.distToDropoff;
            const int distFromTransferPVeh = asgnPickup.distFromDropoff;
            const int distToTransferDVeh = asgnTransfer.distToPickup;
            const int distFromTransferDVeh = asgnTransfer.distFromPickup;
            const int distToDropoff = asgnTransfer.distToDropoff;
            const int distFromDropoff = asgnTransfer.distFromDropoff;

            const int pickupIdx = asgnPickup.pickupStopIdx;
            const int transferIdxPVeh = asgnPickup.dropoffStopIdx;
            const int transferIdxDVeh = asgnTransfer.pickupStopIdx;
            const int dropoffIdx = asgnTransfer.dropoffStopIdx;

            TransferPoint tp(transferLoc, pVeh, dVeh, transferIdxPVeh, transferIdxDVeh, distToTransferPVeh, distFromTransferPVeh, distToTransferDVeh, distFromTransferDVeh);
            AssignmentWithTransfer bestAsgn(pVeh, dVeh, tp);

            bestAsgn.pickup = pickup;
            bestAsgn.dropoff = dropoff;
            bestAsgn.pickupIdx = pickupIdx;
            bestAsgn.transferIdxPVeh = transferIdxPVeh;
            bestAsgn.transferIdxDVeh = transferIdxDVeh;
            bestAsgn.dropoffIdx = dropoffIdx;

            bestAsgn.distToPickup = distToPickup;
            bestAsgn.distFromPickup = distFromPickup;
            bestAsgn.distToTransferPVeh = distToTransferPVeh;
            bestAsgn.distFromTransferPVeh = distFromTransferPVeh;

            bestAsgn.distToTransferDVeh = distToTransferDVeh;
            bestAsgn.distFromTransferDVeh = distFromTransferDVeh;
            bestAsgn.distToDropoff = distToDropoff;
            bestAsgn.distFromDropoff = distFromDropoff;

            const int numStopsPVeh = routeState.numStopsOf(pVeh->vehicleId);
            const int numStopsDVeh = routeState.numStopsOf(dVeh->vehicleId);
            
            bestAsgn.pickupType = ORDINARY;
            if (bestAsgn.pickupIdx == 0) bestAsgn.pickupType = BEFORE_NEXT_STOP;
            if (bestAsgn.pickupIdx == numStopsPVeh - 1) bestAsgn.pickupType = AFTER_LAST_STOP;

            bestAsgn.transferTypePVeh = ORDINARY;
            if (bestAsgn.transferIdxPVeh == 0) bestAsgn.transferTypePVeh = BEFORE_NEXT_STOP;
            if (bestAsgn.transferIdxPVeh == numStopsPVeh - 1) bestAsgn.transferTypePVeh = AFTER_LAST_STOP;

            bestAsgn.transferTypeDVeh = ORDINARY;
            if (bestAsgn.transferIdxDVeh == 0) bestAsgn.transferTypeDVeh = BEFORE_NEXT_STOP;
            if (bestAsgn.transferIdxDVeh == numStopsDVeh - 1) bestAsgn.transferTypeDVeh = AFTER_LAST_STOP;

            bestAsgn.dropoffType = ORDINARY;
            if (bestAsgn.dropoffIdx == 0) bestAsgn.dropoffType = BEFORE_NEXT_STOP;
            if (bestAsgn.dropoffIdx == numStopsDVeh - 1) bestAsgn.dropoffType = AFTER_LAST_STOP;

            assert(bestAsgn.pickup->loc != bestAsgn.transfer.loc);
            assert(bestAsgn.transfer.loc != bestAsgn.dropoff->loc);

            return bestAsgn;
        }

        std::vector<PDLoc> transferPDLocs;


        RequestState &reqState;
        RequestState &intermediateReqState;
        const RouteState &routeState;
        RequestStateInitializerT &requestStateInitializer;

        TransferPointPickerT &transferPointPicker;
        IntermediateAssignmentFinderT &intermediateAssignmentFinder;

        int runningIntermediateId;

};


}