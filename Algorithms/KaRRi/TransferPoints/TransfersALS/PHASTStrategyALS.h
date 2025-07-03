/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2025 Johannes Breitling <johannes.breitling@student.kit.edu>
/// Copyright (c) 2025 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "FlatRegular2DDistanceArray.h"

namespace karri {

    template<typename InputGraphT, typename VehCHEnvT, typename RPHASTEnv, typename LabelSetT, typename LoggerT = NullLogger>
    class PHASTStrategyALS {

        static constexpr int K = LabelSetT::K;

    public:

        PHASTStrategyALS(
                const RouteState &routeState,
                const Fleet &fleet,
                const InputGraphT &inputGraph,
                VehCHEnvT &vehChEnv,
                RPHASTEnv &rphastEnv
        ) : routeState(routeState),
            fleet(fleet),
            inputGraph(inputGraph),
            vehCh(vehChEnv.getCH()),
            rphastEnv(rphastEnv),
            targetsSelectionPhase(rphastEnv.getTargetsSelectionPhase()),
            sourcesSelectionPhase(rphastEnv.getSourcesSelectionPhase()),
            fullSourcesSelection(),
            fullTargetsSelection(),
            transferSourcesSelection(),
            transferTargetsSelection(),
            forwardQuery(rphastEnv.template getForwardRPHASTQuery<LabelSetT>()),
            reverseQuery(rphastEnv.template getReverseRPHASTQuery<LabelSetT>()),
            selectionLogger(LogManager<LoggerT>::getLogger("tals_optimal_phast_selection_stats.csv",
                                                           "num_transfer_points,"
                                                           "num_vertices_source,"
                                                           "num_edges_source,"
                                                           "num_vertices_target,"
                                                           "num_edges_target,"
                                                           "time_source,"
                                                           "time_target\n")) {

            // Build the full selection for a PHAST query
            const auto numVertices = vehCh.downwardGraph().numVertices();
            std::vector<int> allRanks(numVertices);
            std::iota(allRanks.begin(), allRanks.end(), 0);
            std::reverse(allRanks.begin(), allRanks.end());
            fullSourcesSelection = sourcesSelectionPhase.runForKnownVertices(allRanks);
            fullTargetsSelection = targetsSelectionPhase.runForKnownVertices(allRanks);
        }


        // Initializes the strategy with the given transfer points.
        // If RunSelectionPhase is true, RPHAST is used, and a selection phase is run for the given transfer points.
        // If it is false, PHAST is used (with no selection specific to the transfer points).
        // Subsequent calls to the calculate methods will use the chosen selection.
        template<bool RunSelectionPhase = true>
        void init(const std::vector<int> &transferPoints) {
            if constexpr (RunSelectionPhase) {
                Timer timer;
                transferSourcesSelection = runSelectionPhaseForTransferPointsAsSources(transferPoints);
                const auto timeSource = timer.elapsed<std::chrono::nanoseconds>();

                timer.restart();
                transferTargetsSelection = runSelectionPhaseForTransferPointsAsTargets(transferPoints);
                const auto timeTarget = timer.elapsed<std::chrono::nanoseconds>();

                selectionLogger << transferPoints.size() << ","
                                << transferSourcesSelection.subGraph.numVertices() << ","
                                << transferSourcesSelection.subGraph.numEdges() << ","
                                << transferTargetsSelection.subGraph.numVertices() << ","
                                << transferTargetsSelection.subGraph.numEdges() << ","
                                << timeSource << ","
                                << timeTarget << "\n";

                chosenSourcesSelection = &transferSourcesSelection;
                chosenTargetsSelection = &transferTargetsSelection;
            } else {
                chosenSourcesSelection = &fullSourcesSelection;
                chosenTargetsSelection = &fullTargetsSelection;
            }
        }

        FlatRegular2DDistanceArray calculateDistancesFromLastStopToAllTransfers(const std::vector<int> &lastStopLocs,
                                                                                const std::vector<int> &transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(lastStopLocs.size(), numTransferPoints);

            if (lastStopLocs.empty() || transferPoints.empty())
                return result;

            KASSERT(chosenTargetsSelection);

            // Process queries for last stops in batches of size K.
            std::array<int, K> sources;
            const int numBatches = lastStopLocs.size() / K + (lastStopLocs.size() % K != 0);
            for (int batchIdx = 0; batchIdx < numBatches; ++batchIdx) {
                const int batchStart = batchIdx * K;
                const int batchEnd = std::min(batchStart + K, static_cast<int>(lastStopLocs.size()));
                for (int i = batchStart; i < batchEnd; ++i) {
                    const int lastStopLoc = lastStopLocs[i];
                    const auto lastStopVertex = inputGraph.edgeHead(lastStopLoc);
                    sources[i - batchStart] = vehCh.rank(lastStopVertex);
                }
                for (int i = batchEnd; i < batchStart + K; ++i) {
                    sources[i - batchStart] = sources[0]; // copy of first source to fill partial batch
                }

                forwardQuery.run(*chosenTargetsSelection, sources);

                for (int i = batchStart; i < batchEnd; ++i) {
                    const int idxOffset = i * result.width;
                    const int lastStopLoc = lastStopLocs[i];
                    int minDistanceInRow = INFTY;
                    for (int j = 0; j < transferPoints.size(); j++) {
                        const int tpLoc = transferPoints[j];
                        const int tpTail = inputGraph.edgeTail(tpLoc);
                        const int tpRank = vehCh.rank(tpTail);
                        const int tpRankSelection = chosenTargetsSelection->fullToSubMapping[tpRank];
                        const int distance =
                                tpLoc == lastStopLoc ? 0 : forwardQuery.getDistance(tpRankSelection, i - batchStart) +
                                                           inputGraph.travelTime(tpLoc);

                        result.distances[idxOffset + j] = distance;
                        minDistanceInRow = std::min(minDistanceInRow, distance);
                    }
                    result.minDistancePerRow[i] = minDistanceInRow;
                }
            }

            return result;
        }

        FlatRegular2DDistanceArray calculateDistancesFromPickupsToAllTransfers(const std::vector<int> &pickupLocs,
                                                                               const std::vector<int> &transferPoints) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(pickupLocs.size(), numTransferPoints);

            if (pickupLocs.empty() || transferPoints.empty())
                return result;

            KASSERT(chosenTargetsSelection);

            // Process queries for pickups in batches of size K.
            std::array<int, K> sources;
            const int numBatches = pickupLocs.size() / K + (pickupLocs.size() % K != 0);
            for (int batchIdx = 0; batchIdx < numBatches; ++batchIdx) {
                const int batchStart = batchIdx * K;
                const int batchEnd = std::min(batchStart + K, static_cast<int>(pickupLocs.size()));
                for (int i = batchStart; i < batchEnd; ++i) {
                    const int pickupLoc = pickupLocs[i];
                    const auto pickupVertex = inputGraph.edgeHead(pickupLoc);
                    sources[i - batchStart] = vehCh.rank(pickupVertex);
                }
                for (int i = batchEnd; i < batchStart + K; ++i) {
                    sources[i - batchStart] = sources[0]; // copy of first source to fill partial batch
                }

                forwardQuery.run(*chosenTargetsSelection, sources);

                for (int i = batchStart; i < batchEnd; ++i) {
                    const int idxOffset = i * result.width;
                    int minDistanceInRow = INFTY;
                    for (int j = 0; j < transferPoints.size(); j++) {
                        const int tpLoc = transferPoints[j];
                        const int tpTail = inputGraph.edgeTail(tpLoc);
                        const int tpRank = vehCh.rank(tpTail);
                        const int tpRankInSelection = chosenTargetsSelection->fullToSubMapping[tpRank];
                        const int distance = forwardQuery.getDistance(tpRankInSelection, i - batchStart) +
                                             inputGraph.travelTime(tpLoc);

                        result.distances[idxOffset + j] = distance;
                        minDistanceInRow = std::min(minDistanceInRow, distance);
                    }
                    result.minDistancePerRow[i] = minDistanceInRow;
                }
            }

            return result;
        }

        FlatRegular2DDistanceArray
        calculateDistancesFromAllTransfersToDropoffs(const std::vector<int> &transferPoints,
                                                     const std::vector<int> &dropoffLocs) {
            const int numTransferPoints = static_cast<int>(transferPoints.size());
            FlatRegular2DDistanceArray result(dropoffLocs.size(), numTransferPoints);

            if (dropoffLocs.empty() || transferPoints.empty())
                return result;

            KASSERT(chosenSourcesSelection);

            // Process queries for dropoffs in batches of size K.
            std::array<int, K> targets;
            const int numBatches = dropoffLocs.size() / K + (dropoffLocs.size() % K != 0);
            for (int batchIdx = 0; batchIdx < numBatches; ++batchIdx) {
                const int batchStart = batchIdx * K;
                const int batchEnd = std::min(batchStart + K, static_cast<int>(dropoffLocs.size()));
                for (int i = batchStart; i < batchEnd; ++i) {
                    const int dropoffLoc = dropoffLocs[i];
                    const auto dropoffVertex = inputGraph.edgeTail(dropoffLoc);
                    targets[i - batchStart] = vehCh.rank(dropoffVertex);
                }
                for (int i = batchEnd; i < batchStart + K; ++i) {
                    targets[i - batchStart] = targets[0]; // copy of first source to fill partial batch
                }

                reverseQuery.run(*chosenSourcesSelection, targets);

                for (int i = batchStart; i < batchEnd; ++i) {
                    const auto dropoffOffset = inputGraph.travelTime(dropoffLocs[i]);
                    const int idxOffset = i * result.width;
                    int minDistanceInRow = INFTY;
                    for (int j = 0; j < transferPoints.size(); j++) {
                        const int tpLoc = transferPoints[j];
                        const int tpHead = inputGraph.edgeHead(tpLoc);
                        const int tpRank = vehCh.rank(tpHead);
                        const int tpRankInSelection = chosenSourcesSelection->fullToSubMapping[tpRank];
                        const int distance =
                                reverseQuery.getDistance(tpRankInSelection, i - batchStart) + dropoffOffset;

                        result.distances[idxOffset + j] = distance;
                        minDistanceInRow = std::min(minDistanceInRow, distance);
                    }
                    result.minDistancePerRow[i] = minDistanceInRow;
                }
            }

            return result;
        }


    private:

        RPHASTSelection runSelectionPhaseForTransferPointsAsTargets(const std::vector<int> &transferPoints) {
            std::vector<int> targetRanks;
            for (const auto tpLoc: transferPoints) {
                const int tpTail = inputGraph.edgeTail(tpLoc);
                const int tpRank = vehCh.rank(tpTail);
                targetRanks.push_back(tpRank);
            }

            return targetsSelectionPhase.run(targetRanks);
        }

        RPHASTSelection runSelectionPhaseForTransferPointsAsSources(const std::vector<int> &transferPoints) {
            std::vector<int> sourceRanks;
            for (const auto tpLoc: transferPoints) {
                const int tpHead = inputGraph.edgeHead(tpLoc);
                const int tpRank = vehCh.rank(tpHead);
                sourceRanks.push_back(tpRank);
            }

            return sourcesSelectionPhase.run(sourceRanks);
        }

        const RouteState &routeState;
        const Fleet &fleet;
        const InputGraphT &inputGraph;

        const CH &vehCh;

        using SelectionPhase = RPHASTSelectionPhase<dij::NoCriterion>;
        RPHASTEnv &rphastEnv;

        SelectionPhase targetsSelectionPhase;
        SelectionPhase sourcesSelectionPhase;


        RPHASTSelection fullSourcesSelection; // Contains whole search graph, used for full PHAST queries.
        RPHASTSelection fullTargetsSelection; // Contains whole search graph, used for full PHAST queries.
        RPHASTSelection transferSourcesSelection; // Contains search graph for transfer points as sources, used for RPHAST queries.
        RPHASTSelection transferTargetsSelection; // Contains search graph for transfer points as targets, used for RPHAST queries.

        RPHASTSelection const *chosenSourcesSelection;
        RPHASTSelection const *chosenTargetsSelection;

        using Query = PHASTQuery<CH::SearchGraph, CH::Weight, LabelSetT, dij::NoCriterion>;
        Query forwardQuery;
        Query reverseQuery;

        LoggerT &selectionLogger;

    };


}