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

#include "DataStructures/Graph/Attributes/TravelTimeAttribute.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "cassert"

namespace karri::TransferPointStrategies {

    class SearchSpaceIntersection {
    
    public:
        SearchSpaceIntersection(const int numSearches) : numSearches(numSearches), currSearchIntersect(0), edges(std::map<int, int>{}), distances(std::map<int, int[4]>{}) {}
        
        void init(const int maxLeewayP, const int maxLeewayD) {
            currSearchIntersect = 0;
            edges = std::map<int, int>{};
            distances = std::map<int, int[4]>{};
            maxLeewayPVeh = maxLeewayP;
            maxLeewayDVeh = maxLeewayD;
            edgesFoundInFirstSearch = 0;
        }

        void nextSearch() {
            assert(currSearchIntersect < numSearches + 1);
            currSearchIntersect++;
        }

        void edgeFound(int e, int distance) {
            if (currSearchIntersect == 0) {
                edges[e] = 1;
                distances[e][0] = distance;
                edgesFoundInFirstSearch++;
                return;
            }

            if (!edges.count(e)) {
                // Edge would be seen the first time in a search later than the first search
                return;
            }

            if (edges[e] < currSearchIntersect) {
                return;
            }

            edges[e] = (currSearchIntersect + 1);
            distances[e][currSearchIntersect] = distance;
        }

        int getEdgesFoundInFirstSearch() {
            return edgesFoundInFirstSearch;
        }

        std::vector<TransferPoint> getIntersection() {
            auto intersection = std::vector<TransferPoint>{};

            for (auto it = edges.begin(); it != edges.end(); it++) {
                
                const auto e = it->first;
                const auto numFound = it->second;
                
                const auto foundDistances = distances[e];

                if (numFound < numSearches)
                    continue;
                
                TransferPoint tp = TransferPoint();
                tp.loc = e;
                tp.distancePVehToTransfer = foundDistances[0];
                tp.distancePVehFromTransfer = foundDistances[1];
                tp.distanceDVehToTransfer = foundDistances[2];
                tp.distanceDVehFromTransfer = foundDistances[3];

                // Prune the solutions where the max leeway is exceeded
                if (tp.distancePVehToTransfer + tp.distancePVehFromTransfer > maxLeewayPVeh || tp.distanceDVehToTransfer + tp.distanceDVehFromTransfer > maxLeewayDVeh)
                    continue;

                intersection.push_back(tp);
            }

            return intersection;
        }

    private:
        const int numSearches;
        int currSearchIntersect;
        int maxLeewayPVeh;
        int maxLeewayDVeh;
        std::map<int, int> edges;
        std::map<int, int[4]> distances;
        int edgesFoundInFirstSearch;
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
                const Fleet& fleet) :
                routeState(routeState),
                inputGraph(inputGraph),
                reverseGraph(reverseGraph),
                fleet(fleet),
                maxDetour(-1),
                settledVertecies(0),
                searchSpaceIntersection(SearchSpaceIntersection(4)),
                currSearch(0),
                dijSearchTransferPointsFw(inputGraph, {*this, maxDetour, settledVertecies}, {}),
                dijSearchTransferPointsBw(reverseGraph, {*this, maxDetour, settledVertecies}, {}),
                dijkstraSearchLogger(LogManager<std::ofstream>::getLogger("dijkstra.csv",
                    "size\n")) {}

        // Given a set of stop IDs for pickup vehicles and dropoff vehicles, this computes the transfer points between
        // any pair of stops in the two sets. The given stop IDs should indicate the first stop in a pair of stops of
        // the respective vehicle.
       void  computeTransferPoints(const std::vector<int> &pVehStopIds, const std::vector<int> &dVehStopIds) {
            numSearchesRun = 0;
            numEdgesRelaxed = 0;
            numVerticesScanned = 0;

           if (pVehStopIds.empty() || dVehStopIds.empty())
               return;

            std::fill(idxOfStopPVeh.begin(), idxOfStopPVeh.end(), INVALID_INDEX);
            idxOfStopPVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);
            std::fill(idxOfStopDVeh.begin(), idxOfStopDVeh.end(), INVALID_INDEX);
            idxOfStopDVeh.resize(routeState.getMaxStopId() + 1, INVALID_INDEX);

            numStopsPVeh = 0;
            for (const auto &pVehStopId: pVehStopIds) {
                if (idxOfStopPVeh[pVehStopId] == INVALID_INDEX) {
                    idxOfStopPVeh[pVehStopId] = numStopsPVeh++;
                }
            }

            numStopsDVeh = 0;
            for (const auto &dVehStopId: dVehStopIds) {
                if (idxOfStopDVeh[dVehStopId] == INVALID_INDEX) {
                    idxOfStopDVeh[dVehStopId] = numStopsDVeh++;
                }
            }

            if (transferPoints.size() < numStopsPVeh * numStopsDVeh)
                transferPoints.resize(numStopsPVeh * numStopsDVeh);

           // Flatten for parallelization of work
           std::vector<std::pair<int, int>> stopIdPairs;
           stopIdPairs.reserve(numStopsPVeh * numStopsDVeh);
           for (const auto &stopIdPStop: pVehStopIds) {
               const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);

               for (const auto &stopIdDStop: dVehStopIds) {
                   const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);

                   if (vehIdPStop == vehIdDStop)
                       continue;

                   stopIdPairs.emplace_back(stopIdPStop, stopIdDStop);
               }
           }

           // TODO parallelize this loop
           for (const auto &[stopIdPStop, stopIdDStop]: stopIdPairs) {
               const auto vehIdPStop = routeState.vehicleIdOf(stopIdPStop);
               const auto internalIdxPStop = idxOfStopPVeh[stopIdPStop];

               const auto vehIdDStop = routeState.vehicleIdOf(stopIdDStop);
               const auto internalIdxDStop = idxOfStopDVeh[stopIdDStop];

               KASSERT(vehIdPStop != vehIdDStop);

               auto &transferPointsForPair = transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
               transferPointsForPair.clear();
               findTransferPointsBetweenStops(stopIdPStop, stopIdDStop, transferPointsForPair);
           }
        }

        // Given the IDs of the respective first stops in a pair of stops in a pickup vehicle and a pair of stops in a
        // dropoff vehicle, this returns the feasible transfer points for these stop pairs.
        const std::vector<TransferPoint> &getTransferPoints(const int pVehStopId, const int dVehStopId) {
            const auto internalIdxPStop = idxOfStopPVeh[pVehStopId];
            const auto internalIdxDStop = idxOfStopDVeh[dVehStopId];
            KASSERT(internalIdxPStop != INVALID_INDEX && internalIdxDStop != INVALID_INDEX);
            return transferPoints[internalIdxPStop * numStopsDVeh + internalIdxDStop];
        }

    private:

        void findTransferPointsBetweenStops(const int pStopId, const int dStopId, std::vector<TransferPoint>& result) {

            const auto pVehId = routeState.vehicleIdOf(pStopId);
            const auto dVehId = routeState.vehicleIdOf(dStopId);
            const auto stopIdxPVeh = routeState.stopPositionOf(pStopId);
            const auto stopIdxDVeh = routeState.stopPositionOf(dStopId);
            const auto stopLocationsPVeh = routeState.stopLocationsFor(pVehId);
            const auto stopLocationsDVeh = routeState.stopLocationsFor(dVehId);

            const int stopLocPStop = stopLocationsPVeh[stopIdxPVeh];
            const int stopLocPNStop = stopLocationsPVeh[stopIdxPVeh + 1];
            const int stopLocDStop = stopLocationsDVeh[stopIdxDVeh];
            const int stopLocDNStop = stopLocationsDVeh[stopIdxDVeh + 1];

            // Convert the stop locations, that are edges to the respective vertecies
            int pStopVertex = inputGraph.edgeHead(stopLocPStop);
            // assert(inputGraph.edgeTail(pStopLoc) == reverseGraph.edgeHead(pStopLoc));
            int pNextStopVertex = inputGraph.edgeTail(stopLocPNStop);
            // assert(inputGraph.edgeTail(pNStopLoc) == reverseGraph.edgeHead(pNStopLoc));
            int dStopVertex = inputGraph.edgeHead(stopLocDStop);
            int dNextStopVertex = inputGraph.edgeTail(stopLocDNStop);

            dijkstraSearchLogger << searchSpaceIntersection.getEdgesFoundInFirstSearch() << '\n';

            // Get max leeways for the current leg
            const int maxLeewayPVeh = routeState.leewayOfLegStartingAt(pStopId);
            const int maxLeewayDVeh = routeState.leewayOfLegStartingAt(dStopId);


            currSearch = 0;
            searchSpaceIntersection.init(maxLeewayPVeh, maxLeewayDVeh);

            offsetPNStop = inputGraph.travelTime(stopLocPNStop);
            offsetDNStop = inputGraph.travelTime(stopLocDNStop);

            maxDetour = maxLeewayPVeh;
            offsetReverseSearch = offsetPNStop;

            dijSearchTransferPointsFw.run(pStopVertex);
            searchSpaceIntersection.edgeFound(stopLocPStop, 0);
            searchSpaceIntersection.nextSearch();
            currSearch++;
        
            dijSearchTransferPointsBw.run(pNextStopVertex);
            searchSpaceIntersection.edgeFound(stopLocPNStop, 0);
            searchSpaceIntersection.nextSearch();
            currSearch++;

            maxDetour = maxLeewayDVeh;
            offsetReverseSearch = offsetDNStop;

            dijSearchTransferPointsFw.run(dStopVertex);
            searchSpaceIntersection.edgeFound(stopLocDStop, 0);
            searchSpaceIntersection.nextSearch();
            currSearch++;
            
            dijSearchTransferPointsBw.run(dNextStopVertex);
            searchSpaceIntersection.edgeFound(stopLocDNStop, 0);

            result = searchSpaceIntersection.getIntersection();
            for (auto &tp : result) {
                tp.pVeh = &fleet[pVehId];
                tp.dVeh = &fleet[dVehId];
                tp.stopIdxPVeh = stopIdxPVeh;
                tp.stopIdxDVeh = stopIdxDVeh;
                KASSERT(tp.distancePVehToTransfer >= 0);
                KASSERT(tp.distancePVehFromTransfer >= 0);
                KASSERT(tp.distanceDVehToTransfer >= 0);
                KASSERT(tp.distanceDVehFromTransfer >= 0);
            }

            numSearchesRun += 4;
        }

        void settleVertex(int v, int distance) {
            numVerticesScanned++;
            
            int firstEdge;
            int degree;

            if (currSearch % 2 == 0) {
                // Forward search
                firstEdge = inputGraph.firstEdge(v);
                degree = inputGraph.degree(v);
                
                // Scan the outward edges of the vertex
                for (int i = 0; i < degree; i++) {
                    const auto e = firstEdge + i;
                    const auto travelTime = inputGraph.travelTime(e);
                    const auto distanceToEdgeHead = distance + travelTime;
                
                    if (distanceToEdgeHead < maxDetour) {
                        numEdgesRelaxed++;
                        searchSpaceIntersection.edgeFound(e, distanceToEdgeHead);
                    }
                }
            } else {
                // Backward search
                if (distance > maxDetour)
                    return;
                
                firstEdge = reverseGraph.firstEdge(v);
                degree = reverseGraph.degree(v);

                for (int i = 0; i < degree; i++) {
                    int currentEdge = firstEdge + i;
                    int distanceForEdge = distance + offsetReverseSearch;
                    const int forwardEdge = reverseGraph.edgeId(currentEdge);

                    if (distanceForEdge < maxDetour) {
                        numEdgesRelaxed++;
                        searchSpaceIntersection.edgeFound(forwardEdge, distanceForEdge);
                    }
                }
            }
        }

        int64_t getNumSearchesRun() {
            return numSearchesRun;
        }
        
        int64_t getNumEdgesRelaxed() {
            return numEdgesRelaxed;
        }

        int64_t getNumVerticesScanned() {
            return numVerticesScanned;
        }

    private:
        using DistanceLabel = typename DijLabelSet::DistanceLabel;
        using LabelMask = typename DijLabelSet::LabelMask;

        int offsetPNStop = 0;
        int offsetDNStop = 0;
        int offsetReverseSearch = 0;


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
        const Fleet& fleet;
        int maxDetour;
        int maxDetourP;
        int maxDetourD;
        int settledVertecies;
        SearchSpaceIntersection searchSpaceIntersection;
        int currSearch;

        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsFw;
        Dijkstra<InputGraphT, TravelTimeAttribute, DijLabelSet, TransferPointSearch> dijSearchTransferPointsBw;

        int numStopsPVeh;
        int numStopsDVeh;

        // Maps a stop ID to an internal index in the vector of stop IDs.
        std::vector<int> idxOfStopPVeh;
        std::vector<int> idxOfStopDVeh;

        // For two stop IDs i (pVeh) and j (dVeh), transferPoints[idxOfStopPVeh[i] * numStopsDVeh + idxOfStopDVeh[j]] contains the
        // transfer points between the two stops.
        std::vector<std::vector<TransferPoint>> transferPoints;

        int64_t numSearchesRun;
        int64_t numEdgesRelaxed;
        int64_t numVerticesScanned;

        std::ofstream &dijkstraSearchLogger;
    };

}

