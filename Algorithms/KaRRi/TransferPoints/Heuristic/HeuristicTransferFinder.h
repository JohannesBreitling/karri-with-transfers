



#pragma once

namespace karri {


template <typename TransferPointPickerT, typename IntermediateAssignmentFinderT, typename RequestStateInitializerT>
class HeuristicTransferFinder {

    public:
        
        HeuristicTransferFinder(
            RequestState &reqState,
            RequestState &intermediateReqState,
            RequestStateInitializerT &requestStateInitializer,
            TransferPointPickerT &transferPointPicker,
            IntermediateAssignmentFinderT &intermediateAssignmentFinder
        ) : reqState(reqState),
            intermediateReqState(intermediateReqState),
            requestStateInitializer(requestStateInitializer),
            transferPointPicker(transferPointPicker),
            intermediateAssignmentFinder(intermediateAssignmentFinder),
            runningIntermediateId(1000000) {}

        void findAssignments() {
            
            std::cout << "- - - - - - - - - - - - - -\n";
            std::cout << "Find heuristic assignments!\n";

            std::cout << "Request " << reqState.originalRequest.requestId << " at " << reqState.originalRequest.requestTime << "\n";

            // Get the transfer points for the request
            std::vector<int> transferPointLocations = transferPointPicker.pickTransferPointLocations(reqState.pickups, reqState.dropoffs, 1000);

            std::cout << "Number of transfer points: " << transferPointLocations.size() << "\n";

            intermediateReqState.reset();

            Assignment bestAssignmentPickup;
            Assignment bestAssignmentTransfer;
            int bestTotalCost = INFTY;

            const auto origReq = reqState.originalRequest;
            
            // Find the best assignment with one heuristic transfer for every transfer point
            for (const auto &tp : transferPointLocations) {
                // Calculate the intermediate assignments
                // Pickup - Transfer
                Request intermediatePickupRequest;
                intermediatePickupRequest.requestId = runningIntermediateId++;
                intermediatePickupRequest.origin = origReq.origin;
                intermediatePickupRequest.destination = tp;
                intermediatePickupRequest.requestTime = origReq.requestTime;
                intermediatePickupRequest.numRiders = origReq.numRiders;
                requestStateInitializer.initializeIntermediatePickupState(intermediatePickupRequest);
                
                // Find the first half of the assignment
                const auto &intermediatePickupResponse = intermediateAssignmentFinder.findBestAssignment();
                const auto intermediatePickupAssignment = intermediatePickupResponse.getBestAssignment();
                const int arrAtTransfer = intermediatePickupAssignment.cost.arrAtDropoff;

                if (intermediatePickupAssignment.cost.total >= bestTotalCost)
                    continue;
                
                // Transfer - Dropoff
                Request intermediateTransferRequest;
                intermediateTransferRequest.requestId = runningIntermediateId++;
                intermediateTransferRequest.origin = tp;
                intermediateTransferRequest.destination = origReq.destination;
                intermediateTransferRequest.requestTime = arrAtTransfer;
                intermediateTransferRequest.numRiders = origReq.numRiders;
                
                requestStateInitializer.initializeIntermediateTransferState(intermediateTransferRequest);
                
                // Find the second half of the assignment
                const auto &intermediateTransferResponse = intermediateAssignmentFinder.findBestAssignment();
                const auto intermediateTransferAssignment = intermediateTransferResponse.getBestAssignment();
                
                const int currentTotal = intermediatePickupAssignment.cost.total + intermediateTransferAssignment.cost.total;
                if (currentTotal < bestTotalCost) {
                    bestAssignmentPickup = intermediatePickupAssignment;
                    bestAssignmentTransfer = intermediateTransferAssignment;
                    bestTotalCost = currentTotal;

                    std::cout << "We actually found someting! With cost: " << bestTotalCost << "\n";
                }
            }

            std::cout << "- - - - - - - - - - - - - -\n";
        }


    private:

        RequestState &reqState;
        RequestState &intermediateReqState;
        RequestStateInitializerT &requestStateInitializer;

        TransferPointPickerT &transferPointPicker;
        IntermediateAssignmentFinderT &intermediateAssignmentFinder;

        int runningIntermediateId;

};


}