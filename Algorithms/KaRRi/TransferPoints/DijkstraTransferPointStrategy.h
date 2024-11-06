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
        SearchSpaceIntersection(const int numSearches) : numSearches(numSearches), currSearch(0), vertecies(std::map<int, int>{}), distances(std::map<int, int[4]>{}) {}
        
        void init() {
            currSearch = 0;
            vertecies = std::map<int, int>{};
            distances = std::map<int, int[4]>{};
        }

        void nextSearch() {
            assert(currSearch < numSearches + 1);
            currSearch++;
        }

        void vertexFound(int v, int distance) {
            if (currSearch == 0) {
                vertecies[v] = 1;
                distances[v][0] = distance;
                return;
            }

            if (!vertecies.count(v)) {
                // Vertex would be seen the first time in a search later than the first search
                return;
            }

            if (vertecies[v] < currSearch) {
                // vertecies.erase(v);
                return;
            }

            vertecies[v] = (vertecies[v] + 1);
            distances[v][currSearch] = distance;
        }

        std::vector<TransferPoint> getIntersection() {
            auto intersection = std::vector<TransferPoint>{};

            for (auto it = vertecies.begin(); it != vertecies.end(); it++) {
                
                const auto v = it->first;
                const auto numFound = it->second;
                
                const auto foundDistances = distances[v];

                if (numFound < numSearches)
                    continue;
                
                TransferPoint tp = TransferPoint();
                tp.loc = v;
                tp.distancePVehToTransfer = foundDistances[0];
                tp.distancePVehFromTransfer = foundDistances[1];
                tp.distanceDVehToTransfer = foundDistances[2];
                tp.distanceDVehFromTransfer = foundDistances[3];

                intersection.push_back(tp);
            }

            return intersection;
        }

    private:
        const int numSearches;
        int currSearch;
        std::map<int, int> vertecies;
        std::map<int, int[4]> distances;
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
                strategy.settleVertex(v, distToV[0]);

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

