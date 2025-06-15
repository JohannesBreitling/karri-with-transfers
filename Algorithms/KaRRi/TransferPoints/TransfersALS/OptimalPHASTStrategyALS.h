#pragma once

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename RPHASTEnv>    
    class OptimalPHASTStrategyALS {

    public:

        OptimalPHASTStrategyALS(
            const RouteState &routeState,
            const Fleet &fleet,
            const InputGraphT &inputGraph,
            VehCHEnvT &vehChEnv,
            RPHASTEnv &rphastEnv
            ) : routeState(routeState),
                fleet(fleet),
                inputGraph(inputGraph),
                vehCh(vehChEnv.getCH()),
                // vehChQuery(vehChEnv.template getFullCHQuery<>())
                rphastEnv(rphastEnv),
               targetsSelection(rphastEnv.getTargetsSelectionPhase()),
               sourcesSelection(rphastEnv.getSourcesSelectionPhase()),
                fullSelection(),
                forwardQuery(rphastEnv.getForwardRPHASTQuery()),
                reverseQuery(rphastEnv.getReverseRPHASTQuery()) {

            // Build the full selection for a PHAST query
            const auto numVertices = vehCh.downwardGraph().numVertices();

            fullSelection.fullToSubMapping = std::vector<int>(numVertices);
            fullSelection.subToFullMapping = std::vector<int>(numVertices);
            for (int i = 0; i < numVertices; i++) {
                fullSelection.fullToSubMapping[i] = numVertices - 1 - i;
                fullSelection.subToFullMapping[i] = numVertices - 1 - i;
            }

            fullSelection.subGraph = vehCh.downwardGraph();
            Permutation perm(fullSelection.fullToSubMapping.begin(), fullSelection.fullToSubMapping.end());
            fullSelection.subGraph.permuteVertices(perm);
        }

        // Maps: lastStopId -> tpLoc -> distance last stop to transfer
        std::map<int, std::map<int, int>> calculateDistancesFromLastStopToAllTransfers(std::vector<int>& lastStopLocs, std::vector<EdgeInEllipse>& transferPoints) {
            std::map<int, std::map<int, int>> result;

            if (lastStopLocs.size() == 0 || transferPoints.size() == 0)
                return result;

            
            // Build selection with all transfer points
            
            std::vector<int> targetRanks;
            for (const auto edgeInEllipse : transferPoints) {
                const int tpLoc = edgeInEllipse.edge;
                const int tpTail = inputGraph.edgeTail(tpLoc); 
                const int tpRank = vehCh.rank(tpTail);
                targetRanks.push_back(tpRank);
            }

            RPHASTSelection transferSelection = targetsSelection.run(targetRanks);
            
            for (const int lastStopLoc : lastStopLocs) {
                const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
                const auto lastStopRank = vehCh.rank(lastStopVertex);
                
                forwardQuery.run(transferSelection, lastStopRank);
                
                // std::vector<int> distances;
                for (int i = 0; i < transferPoints.size(); i++) {
                    // Get the rank of the transfer point
                    const int tpLoc = transferPoints[i].edge;
                    const int tpTail = inputGraph.edgeTail(tpLoc); 
                    const int tpRank = vehCh.rank(tpTail);
                    const int tpRankSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = forwardQuery.getDistance(tpRankSelection);
                    const int offset = inputGraph.travelTime(tpLoc);
                    
                    result[lastStopLoc][tpLoc] = distance + offset;
                    // distances.push_back();
                }
                
                // KASSERT(distances.size() == transferPoints.size());
                // result[lastStopId] = distances;
            }

            return result;
        }


        // Maps: pickupLoc -> tpLoc -> distance pickup to transfer point
        std::map<int, std::map<int, int>> caluclateDistancesFromPickupsToAllTransfers(std::vector<int>& pickupLocs, std::vector<EdgeInEllipse>& transferPoints) {
            std::map<int, std::map<int, int>> result;

            if (pickupLocs.size() == 0 || transferPoints.size() == 0)
                return result;

            std::vector<int> targetRanks;
            for (const auto edgeInEllipse : transferPoints) {
                const int tpLoc = edgeInEllipse.edge;
                const int tpTail = inputGraph.edgeTail(tpLoc); 
                const int tpRank = vehCh.rank(tpTail);
                targetRanks.push_back(tpRank);
            }

            RPHASTSelection transferSelection = targetsSelection.run(targetRanks);
            

            for (const int pickupLoc : pickupLocs) {
                const auto pickupVertex = inputGraph.edgeHead(pickupLoc);
                const auto pickupRank = vehCh.rank(pickupVertex);
                
                forwardQuery.run(transferSelection, pickupRank);
                
                std::vector<int> distances;
                for (int i = 0; i < transferPoints.size(); i++) {
                    const int tpLoc = transferPoints[i].edge;
                    const int tpTail = inputGraph.edgeTail(tpLoc);
                    const int tpRank = vehCh.rank(tpTail);
                    const int tpRankInSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = forwardQuery.getDistance(tpRankInSelection);
                    const int offset = inputGraph.travelTime(tpLoc);
                    
                    result[pickupLoc][tpLoc] = distance + offset;
                }

                // result[pickupLoc] = distances;                
            }

            return result;
        }

        // Maps: dropoffLoc -> tpLoc -> distance transfer point to dropoff
        std::map<int, std::map<int, int>> caluclateDistancesFromAllTransfersToDropoffs(std::vector<EdgeInEllipse>& transferPoints, std::vector<int>& dropoffLocs) {
            std::map<int, std::map<int, int>> result;

            if (dropoffLocs.size() == 0 || transferPoints.size() == 0)
                return result;

            std::vector<int> sourceRanks;
            for (const auto edgeInEllipse : transferPoints) {
                const int tpLoc = edgeInEllipse.edge;
                const int tpHead = inputGraph.edgeHead(tpLoc);
                const int tpRank = vehCh.rank(tpHead);
                sourceRanks.push_back(tpRank);
            }

            RPHASTSelection transferSelection = sourcesSelection.run(sourceRanks);

            for (const int dropoffLoc : dropoffLocs) {
                const auto dropoffVertex = inputGraph.edgeTail(dropoffLoc);
                const auto dropoffRank = vehCh.rank(dropoffVertex);
                const auto dropoffOffset = inputGraph.travelTime(dropoffVertex);
                
                reverseQuery.run(transferSelection, dropoffRank);
                
                for (int i = 0; i < transferPoints.size(); i++) {
                    const int tpLoc = transferPoints[i].edge;
                    const int tpHead = inputGraph.edgeHead(tpLoc);
                    const int tpRank = vehCh.rank(tpHead);
                    const int tpRankInSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = reverseQuery.getDistance(tpRankInSelection);
                    
                    result[dropoffLoc][tpLoc] = distance + dropoffOffset;
                }
            }

            return result;
        }

        int64_t getNumSearchesRun() {
            return numSearchesRun;
        }

        int64_t getSearchTime() {
            return searchTime;
        }
            

    private:
        // using VehCHQuery = typename VehCHEnvT::template FullCHQuery<>;

        const RouteState &routeState;
        const Fleet &fleet;
        const InputGraphT &inputGraph;

        const CH &vehCh;

        using SelectionPhase = RPHASTSelectionPhase<dij::NoCriterion>; 
        RPHASTEnv &rphastEnv;

        SelectionPhase targetsSelection;
        SelectionPhase sourcesSelection;
        RPHASTSelection fullSelection;

        using Query = PHASTQuery<CH::SearchGraph, CH::Weight, BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>, dij::NoCriterion>;
        Query forwardQuery;
        Query reverseQuery;
        
        // VehCHQuery vehChQuery;

        

        int64_t numSearchesRun;
        int64_t searchTime;
        // int64_t numEdgesRelaxed;
        // int64_t numVerticesScanned;
    
    };








}