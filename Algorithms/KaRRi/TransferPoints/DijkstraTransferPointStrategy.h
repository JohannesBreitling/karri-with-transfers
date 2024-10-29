/// ******************************************************************************
/// MIT License
///
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

#include "Algorithms/Dijkstra/Dijkstra.h"

namespace karri::TransferPointStrategies {



    template<
        typename InputGraphT,
        typename DijLabelSet
    >
    class DijkstraTransferPointStrategy {

    public:
        DijkstraTransferPointStrategy(
                const RouteState &routeState,
                const InputGraphT &inputGraph,
                const InputGraphT &reverseGraph) : 
                routeState(routeState),
                inputGraph(inputGraph),
                reverseGraph(reverseGraph),
                dijSearchTransferPointsFw(inputGraph, {}),
                dijSearchTransferPointsBw(reverseGraph, {}) {}

        void findTransferPoints() {
            
        }

    private:
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;


        struct TransferPointSearch {
            TransferPointSearch() {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distFromV, const DistLabelContainerT & /*distLabels*/) {
                std::cout << "Dijkstra Check Stop Criterion" << std::endl;
                return false;
            }

        private:
            
        };

        const RouteState &routeState;
        const InputGraphT &inputGraph;
        const InputGraphT &reverseGraph;

        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsFw;
        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsBw;
    };

}

