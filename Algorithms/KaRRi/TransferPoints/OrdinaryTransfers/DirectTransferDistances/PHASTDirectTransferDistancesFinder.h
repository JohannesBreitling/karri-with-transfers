/// ******************************************************************************
/// MIT License
///
/// Copyright (c) 2023 Moritz Laupichler <moritz.laupichler@kit.edu>
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

#include "Tools/Timer.h"
#include "Algorithms/CH/CH.h"
#include "Algorithms/Buckets/SharedSearchSpaceBucketContainer.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "DataStructures/Labels/SimdLabelSet.h"
#include "Algorithms/KaRRi/RequestState/RequestState.h"
#include "DirectTransferDistances.h"

namespace karri {

    template<typename CHEnvT, typename RPHASTEnvT, typename LabelSetT>
    
    class PHASTDirectTransferDistancesFinder {

    public:
        static constexpr int K = LabelSetT::K;
        using DistanceLabel = typename LabelSetT::DistanceLabel;

    public:
        PHASTDirectTransferDistancesFinder(const int numVertices,
                                         const CHEnvT &chEnv,
                                         const RPHASTEnvT &rphastEnv,
                                         const PDLocType& type)
                : ch(chEnv.getCH()),
                  rphastEnv(rphastEnv),
                  selection(type == PDLocType::PICKUP ? rphastEnv.getSourcesSelectionPhase() : rphastEnv.getTargetsSelectionPhase()),
                  query(type == PDLocType::PICKUP ? rphastEnv.getReverseRPHASTQuery() : rphastEnv.getForwardRPHASTQuery()),
                  distances(numVertices) {}


        // Runs selection phase for PD locs for many-to-many searches.
        void runSelectionForPdLocs(const std::vector<int> &pdLocRanks, const std::vector<int> &pdLocOffsets) {
            KASSERT(pdLocRanks.size() == pdLocOffsets.size());
            const auto numPdLocs = pdLocRanks.size();
            currentPdLocRanks = pdLocRanks;
            distances.init(numPdLocs);
            
            selection.run(pdLocRanks, pdLocOffsets);
        }

        // Runs query for given CH rank
        void runQueryForTransferRank(const int transferRank) {
            if (distances.knowsDistancesForTransferRank(transferRank))
                return;
            
            query.run(selection, transferRank);

            for (int i = 0; i < currentPdLocRanks.size(); i++) {
                const auto pdLocRank = currentPdLocRanks[i];
                int distance = query.getDistance(selection.fullToSubMapping[transferRank]);
                distances.updateDistanceIfSmaller(i, transferRank, distance);
            }
        }

        const DirectTransferDistances<LabelSetT> &getDistances() const {
            return distances;
        }

    private:

        const CH &ch;

        using SelctionPhase = RPHASTSelectionPhase<dij::NoCriterion>;
        RPHASTEnvT &rphastEnv;
        
        SelectionPhase selection;
        Query query;

        const std::vector<int> &currentPdLocRanks;

        DirectTransferDistances<LabelSetT> distances;

        unsigned int curTransferRank;
        unsigned int curPdLocBatchId;
    };
}