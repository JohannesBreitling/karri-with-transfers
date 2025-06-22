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

        struct FlatRegular2DDistanceArray {

            ConstantVectorRange<int> getDistancesFor(const int row) const {
                KASSERT(row >= 0 && (width == 0 || row < distances.size() / width));
                return {distances.begin() + row * width, distances.begin() + (row + 1) * width};
            }

            int getMinDistanceFor(const int row) const {
                KASSERT(row >= 0 && row < minDistancePerRow.size());
                return minDistancePerRow[row];
            }

        private:

            friend OptimalPHASTStrategyALS;

            // Constructs a 2D distance array with numRows rows and rowWidth distances per row.
            // No initialization of values takes place.
            FlatRegular2DDistanceArray(const int numRows, const int rowWidth) : width(rowWidth), distances(numRows * rowWidth), minDistancePerRow(numRows, INFTY) {}


            int width; // Number of distances per row
            std::vector<int> distances;
            std::vector<int> minDistancePerRow;
        };

        FlatRegular2DDistanceArray calculateDistancesFromLastStopToAllTransfers(const std::vector<int>& lastStopLocs, const std::vector<EdgeInEllipse>& transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(lastStopLocs.size(), numTransferPoints);

            if (lastStopLocs.empty() || transferPoints.empty())
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
            
            for (int i = 0; i < lastStopLocs.size(); i++) {
                const int lastStopLoc = lastStopLocs[i];
                const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
                const auto lastStopRank = vehCh.rank(lastStopVertex);
                
                forwardQuery.run(transferSelection, lastStopRank);

                const int idxOffset = i * result.width;
                int minDistanceInRow = INFTY;
                for (int j = 0; j < transferPoints.size(); j++) {
                    // Get the rank of the transfer point
                    const int tpLoc = transferPoints[j].edge;
                    const int tpTail = inputGraph.edgeTail(tpLoc); 
                    const int tpRank = vehCh.rank(tpTail);
                    const int tpRankSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = forwardQuery.getDistance(tpRankSelection) + inputGraph.travelTime(tpLoc);

                    result.distances[idxOffset + j] = distance;
                    minDistanceInRow = std::min(minDistanceInRow, distance);
                }
                result.minDistancePerRow[i] = minDistanceInRow;
            }

            return result;
        }

        FlatRegular2DDistanceArray calculateDistancesFromPickupsToAllTransfers(const std::vector<int>& pickupLocs, const std::vector<EdgeInEllipse>& transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(pickupLocs.size(), numTransferPoints);

            if (pickupLocs.empty() || transferPoints.empty())
                return result;

            std::vector<int> targetRanks;
            for (const auto edgeInEllipse : transferPoints) {
                const int tpLoc = edgeInEllipse.edge;
                const int tpTail = inputGraph.edgeTail(tpLoc); 
                const int tpRank = vehCh.rank(tpTail);
                targetRanks.push_back(tpRank);
            }

            RPHASTSelection transferSelection = targetsSelection.run(targetRanks);
            

            for (auto i = 0; i < pickupLocs.size(); ++i) {
                const int pickupLoc = pickupLocs[i];
                const auto pickupVertex = inputGraph.edgeHead(pickupLoc);
                const auto pickupRank = vehCh.rank(pickupVertex);
                
                forwardQuery.run(transferSelection, pickupRank);

                const int idxOffset = i * result.width;
                int minDistanceInRow = INFTY;
                for (int j = 0; j < transferPoints.size(); j++) {
                    const int tpLoc = transferPoints[j].edge;
                    const int tpTail = inputGraph.edgeTail(tpLoc);
                    const int tpRank = vehCh.rank(tpTail);
                    const int tpRankInSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = forwardQuery.getDistance(tpRankInSelection) + inputGraph.travelTime(tpLoc);

                    result.distances[idxOffset + j] = distance;
                    minDistanceInRow = std::min(minDistanceInRow, distance);
                }
                result.minDistancePerRow[i] = minDistanceInRow;
            }

            return result;
        }

        // Maps: dropoffLoc -> tpLoc -> distance transfer point to dropoff
        FlatRegular2DDistanceArray calculateDistancesFromAllTransfersToDropoffs(const std::vector<EdgeInEllipse>& transferPoints, const std::vector<int>& dropoffLocs) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(dropoffLocs.size(), numTransferPoints);

            if (dropoffLocs.empty() || transferPoints.empty())
                return result;

            std::vector<int> sourceRanks;
            for (const auto edgeInEllipse : transferPoints) {
                const int tpLoc = edgeInEllipse.edge;
                const int tpHead = inputGraph.edgeHead(tpLoc);
                const int tpRank = vehCh.rank(tpHead);
                sourceRanks.push_back(tpRank);
            }

            RPHASTSelection transferSelection = sourcesSelection.run(sourceRanks);

            for (auto i = 0; i < dropoffLocs.size(); ++i) {
                const int dropoffLoc = dropoffLocs[i];
                const auto dropoffVertex = inputGraph.edgeTail(dropoffLoc);
                const auto dropoffRank = vehCh.rank(dropoffVertex);
                const auto dropoffOffset = inputGraph.travelTime(dropoffLoc);
                
                reverseQuery.run(transferSelection, dropoffRank);

                const int idxOffset = i * result.width;
                int minDistanceInRow = INFTY;
                for (int j = 0; j < transferPoints.size(); j++) {
                    const int tpLoc = transferPoints[j].edge;
                    const int tpHead = inputGraph.edgeHead(tpLoc);
                    const int tpRank = vehCh.rank(tpHead);
                    const int tpRankInSelection = transferSelection.fullToSubMapping[tpRank];
                    const int distance = reverseQuery.getDistance(tpRankInSelection) + dropoffOffset;

                    result.distances[idxOffset + j] = distance;
                    minDistanceInRow = std::min(minDistanceInRow, distance);
                }
                result.minDistancePerRow[i] = minDistanceInRow;
            }

            return result;
        }
            

    private:

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
    
    };








}