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
#include "cassert"

namespace karri::TransferPointStrategies {

    class SearchSpaceIntersection {
    
    public:
        SearchSpaceIntersection(const int numSearches) : numSearches(numSearches), currSearch(0), verteciesFound(std::map<int, std::tuple<int, TransferPoint>>{}) {}
        
        void init() {
            currSearch = 0;
            verteciesFound = std::map<int, std::tuple<int, TransferPoint>>{};
        }

        void nextSearch() {
            assert(currSearch < numSearches + 1);
            currSearch++;
        }

        void vertexFound(int v, int distance) {
            if (currSearch == 0) {
                verteciesFound[v] = {1, TransferPoint()};
                std::get<1>(verteciesFound[v]).loc = v;
                std::get<1>(verteciesFound[v]).distancePVehToTransfer = distance;
                return;
            }

            if (!verteciesFound.count(v)) {
                verteciesFound.erase(v);
                return;
            } 

            switch (currSearch) {
                case 1:
                    std::get<1>(verteciesFound[v]).distancePVehFromTransfer = distance;
                    break;
                case 2:
                    std::get<1>(verteciesFound[v]).distanceDVehToTransfer = distance;
                    break;
                case 3:
                    std::get<1>(verteciesFound[v]).distanceDVehFromTransfer = distance;
                    break;
            }
        }

        std::vector<TransferPoint> getIntersection() {
            auto intersection = std::vector<TransferPoint>{};

            for (auto it = verteciesFound.begin(); it != verteciesFound.end(); it++) {

            }

/*
            for (auto it = verteciesFound.begin(); it != verteciesFound.end(); it++) 
                // int found = std::get<0>(it->second);
                //if (found < numSearches) continue;
                // intersection.push_back({it->first, it->second[1], it->second[2], it->second[3], it->second[4]});
            }*/

            return intersection;
        }

    private:
        const int numSearches;
        int currSearch;
        std::map<int, std::tuple<int, TransferPoint>> verteciesFound;
    };



    template<
        typename InputGraphT,
        typename DijLabelSet
    >
    class DijkstraTransferPointStrategy {

    public:
        DijkstraTransferPointStrategy(
                const RouteState &routeState,
                const InputGraphT &inputGraph,
                const InputGraphT &reverseGraph,
                std::vector<TransferPoint> &posTransferPoints) : 
                routeState(routeState),
                inputGraph(inputGraph),
                reverseGraph(reverseGraph),
                maxDetour(-1),
                maxDetourP(-1),
                maxDetourD(-1),
                settledVertecies(0),
                searchSpaceIntersection(SearchSpaceIntersection(4)),
                currSearch(0),
                dijSearchTransferPointsFw(inputGraph, {*this, maxDetour, settledVertecies}, {}),
                dijSearchTransferPointsBw(reverseGraph, {*this, maxDetour, settledVertecies}, {}),
                possibleTransferPoints(posTransferPoints) {}

        void setMaxDetours(int detourP, int detourD) {
            maxDetourP = detourP;
            maxDetourD = detourD;
        }

        void findTransferPoints(int pStop, int pNextStop, int dStop, int dNextStop) {
            searchSpaceIntersection.init();
            
            maxDetour = maxDetourP;

            dijSearchTransferPointsFw.run(pStop);
            searchSpaceIntersection.nextSearch();
        
            dijSearchTransferPointsBw.run(pNextStop);
            searchSpaceIntersection.nextSearch();

            maxDetour = maxDetourD;
            
            dijSearchTransferPointsFw.run(dStop);
            searchSpaceIntersection.nextSearch();
            
            dijSearchTransferPointsBw.run(dNextStop);

            possibleTransferPoints = searchSpaceIntersection.getIntersection();
        }

        void settleVertex(int v, int distance) {
            searchSpaceIntersection.vertexFound(v, distance);
        }

    private:
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;


        struct TransferPointSearch {
            TransferPointSearch(DijkstraTransferPointStrategy &strategy, int &maxDetour, int &settledVertecies)
                            : strategy(strategy),
                              maxDetour(maxDetour),
                              setteledVertecies(settledVertecies) {}

            template<typename DistLabelT, typename DistLabelContainerT>
            bool operator()(const int v, DistLabelT &distToV, const DistLabelContainerT & /*distLabels*/) {
                // Check stopping criterion
                LabelMask radiusExceeded = distToV > DistanceLabel(maxDetour); 

                if (allSet(radiusExceeded)) {
                    return true;
                }

                // Settle the found vertex (add to search space intersection)
                strategy.settleVertex(v, distToV[v]);

                std::cout << "<" << v << ", " << distToV[v] << ">" << std::endl; 

                setteledVertecies++;
                return false;
            }

        private:
            DijkstraTransferPointStrategy &strategy;
            int &maxDetour;
            int &setteledVertecies; 
        };

        const RouteState &routeState;
        const InputGraphT &inputGraph;
        const InputGraphT &reverseGraph;
        int maxDetour;
        int maxDetourP;
        int maxDetourD;
        int settledVertecies;
        SearchSpaceIntersection searchSpaceIntersection;
        int currSearch;

        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsFw;
        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsBw;

        std::vector<TransferPoint> &possibleTransferPoints;
    };

}

