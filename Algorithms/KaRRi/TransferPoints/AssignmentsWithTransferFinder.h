/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
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

#include "Algorithms/KaRRi/TransferPoints/TransferPointFinder.h"
#include "Algorithms/KaRRi/TransferPoints/OrdinaryTransfers/OrdinaryTransferFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSPVeh/TransferALSPVehFinder.h"
#include "Algorithms/KaRRi/TransferPoints/TransfersALS/TransfersALSDVeh/TransferALSDVehFinder.h"

#include "Algorithms/KaRRi/TimeUtils.h"
#include "Algorithms/KaRRi/BaseObjects/AssignmentWithTransfer.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"

namespace karri {

    template <typename OrdinaryTransferFinderT, typename TransferALSPVehFinderT, typename TransferALSDVehFinderT, typename InsertionAsserterT>
    class AssignmentsWithTransferFinder {

    public:
        AssignmentsWithTransferFinder(
            OrdinaryTransferFinderT &ordinaryTransfers,
            TransferALSPVehFinderT &transfersALSPVeh,
            TransferALSDVehFinderT &transfersALSDVeh,
            RequestState &requestState,
            InsertionAsserterT &asserter)
                            : ordinaryTransfers(ordinaryTransfers),
                              transfersALSPVeh(transfersALSPVeh),
                              transfersALSDVeh(transfersALSDVeh),
                              requestState(requestState),
                              asserter(asserter) {}

        void init() {
            // no op
        }

        // TODO Stats wie die Verteilung der Verbesserung der Kosten liegen und so
        // TODO Im Aufschrieb, SPP gut ausarbeiten, um das Framework gut zu erklären, um eventuelle Follow Ups zu erleichtern 
        // Method to find the best assignment with exactly one transfer
        void findBestAssignment() {
            Timer totalTimer;

            // std::cout << "// - - - - - - - - - - - - - - - - - - - -" << std::endl;
            // std::cout << "REQUEST START" << std::endl;

            // auto &stats = requestState.stats().transferStats;
            // numPartialsTried = 0;
            // numAssignmentsTried = 0;
            // numAssignmentsWithUnkownPairedDistanceTried = 0;
            // int64_t numPickupDropoffPairs = 0;
            // int64_t numStopPairs = 0;

            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            transfersALSPVeh.findAssignments();
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            ordinaryTransfers.findAssignments();
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            // The pickup vehicle picks up the user either bns, ord or als
            // Then the pickup vehicle drives to one of the stops of the dropoff vehicle, where the transfer is done
            transfersALSDVeh.findAssignments();

            //* Test the best assignment found
            assert(asserter.assertAssignment(requestState.getBestAssignmentWithTransfer()));

            return;


            // std::cout << "REQUEST END" << std::endl;
            // std::cout << "- - - - - - - - - - - - - - - - - - - - //" << std::endl;

            // const auto totalTime = totalTimer.elapsed<std::chrono::nanoseconds>();

            // stats.totalTime += totalTime;
            // stats.numPickupDropoffPairs += numPickupDropoffPairs;
            // stats.numStopPairs += numStopPairs;
            // stats.numPartialAssignmentsTried += numPartialsTried;
            // stats.numAssignmentsTried += numAssignmentsTried;
        }


    private:

        OrdinaryTransferFinderT &ordinaryTransfers;
        TransferALSPVehFinderT &transfersALSPVeh;
        TransferALSDVehFinderT &transfersALSDVeh;

        RequestState &requestState;

        InsertionAsserterT &asserter;
        // int64_t numAssignmentsTried = 0;
        // int64_t numPartialsTried = 0;
        // int64_t numAssignmentsWithUnkownPairedDistanceTried = 0;
    };


}