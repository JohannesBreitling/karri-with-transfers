/// ******************************************************************************
/// MIT License
///
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
            ordinaryTransfers.init();
            transfersALSPVeh.init();
            transfersALSDVeh.init();
        }

        void findBestAssignment() {
            // Method to find the best assignment with exactly one transfer, i. e. the best possible
            // single transfer journey for the given request
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            transfersALSPVeh.findAssignments();

            // * ORDINARY TRANSFER
             ordinaryTransfers.findAssignments();
            
            // * TRANSFER AFTER LAST STOP (PVeh)
            transfersALSDVeh.findAssignments();

            //* Test the best assignment found
             KASSERT(asserter.assertAssignment(requestState.getBestAssignmentWithTransfer()));
        }


    private:

        OrdinaryTransferFinderT &ordinaryTransfers;
        TransferALSPVehFinderT &transfersALSPVeh;
        TransferALSDVehFinderT &transfersALSDVeh;

        RequestState &requestState;

        InsertionAsserterT &asserter;
    };
}