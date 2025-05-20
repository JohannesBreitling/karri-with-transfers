/// ******************************************************************************
/// MIT License
///
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

#include "Tools/CommandLine/CommandLineParser.h"
#include "DataStructures/Graph/Graph.h"
#include "Algorithms/Dijkstra/Dijkstra.h"
#include "DataStructures/Labels/BasicLabelSet.h"
#include "Tools/CommandLine/ProgressBar.h"
#include "DataStructures/Graph/Attributes/EdgeTailAttribute.h"

inline void printUsage() {
    std::cout <<
              "Usage: SampleHeuristicTransferPointsBetweenness -g <file> -b <file> -d <time> -o <file>\n"
              "Given a graph and the betweenness of each edge, computes a subset of edges with high betweenness\n"
              "and a minimum distance between each other that may be good candidates for taxi sharing transfer locations.\n"
              "  -g <file>         input graph in binary format\n"
              "  -b <file>         betweenness of each edge in binary format (see ComputeBetweenness)\n"
              "  -d <time>         minimum distance between each pair of edges in seconds (dflt: 300)\n"
              "  -o <file>         place output in <file>\n"
              "  -help             display this help and exit\n";
}

int main(int argc, char *argv[]) {
    try {
        CommandLineParser clp(argc, argv);
        if (clp.isSet("help")) {
            printUsage();
            return EXIT_SUCCESS;
        }

        // Parse the command-line options.
        const auto graphFileName = clp.getValue<std::string>("g");
        const auto betweennessFileName = clp.getValue<std::string>("b");
        const auto minDistance = clp.getValue<int>("d", 300) * 10;
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".bin"))
            outputFileName += ".bin";

        using Weight = TravelTimeAttribute;

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<>, EdgeAttrs<EdgeTailAttribute, Weight>>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        FORALL_VALID_EDGES(inputGraph, v, e) {
                inputGraph.edgeTail(e) = v;
            }
        std::cout << "done.\n";

        // Read betweenness from file
        std::cout << "Reading betweenness from file... " << std::flush;
        std::ifstream betweennessFile(betweennessFileName, std::ios::binary);
        if (!betweennessFile.good())
            throw std::invalid_argument("file not found -- '" + betweennessFileName + "'");
        std::vector<int64_t> betweenness;
        bio::read(betweennessFile, betweenness);
        betweennessFile.close();
        if (betweenness.size() != inputGraph.numEdges())
            throw std::invalid_argument(
                    "number of entries in betweenness file does not match number of edges in graph");
        std::cout << "done.\n";

        // Sort edges in descending order of betweenness
        std::cout << "Sorting edges by betweenness... " << std::flush;
        std::vector<int> edgesInOrder(inputGraph.numEdges());
        std::iota(edgesInOrder.begin(), edgesInOrder.end(), 0);
        std::sort(edgesInOrder.begin(), edgesInOrder.end(),
                  [&betweenness](int a, int b) { return betweenness[a] > betweenness[b]; });
        std::cout << "done.\n";


        // Initialize output to make sure file can be opened
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");

        using LabelSet = BasicLabelSet<0, ParentInfo::NO_PARENT_INFO>;
        struct StopWhenMinDistanceReachedOrExistingCandidateSeen {
            StopWhenMinDistanceReachedOrExistingCandidateSeen(int minDistance,
                                                              const std::vector<int> &offsetForCandidateTail,
                                                              uint8_t &seenOtherCandidate)
                    : minDistance(minDistance),
                      offsetForCandidateTail(offsetForCandidateTail),
                      seenOtherCandidate(seenOtherCandidate) {}

            bool operator()(const int v, LabelSet::DistanceLabel &dist,
                            const StampedDistanceLabelContainer<LabelSet::DistanceLabel> &) {
                // If we have reached the tail of another candidate edge, and the distance to the tail plus the
                // length of the other candidate is smaller than the minimum distance between candidates, the current
                // source edge is too close to this candidate.
                if (dist[0] + offsetForCandidateTail[v] < minDistance) {
                    seenOtherCandidate = static_cast<uint8_t>(true);
                    return true;
                }
                return dist[0] >= minDistance;
            }

        private:
            int minDistance;

            // If v is the tail of a candidate edge, offsetForCandidateTail contains the length of the shortest such
            // candidate edge. Otherwise, contains INFTY.
            const std::vector<int> &offsetForCandidateTail;

            uint8_t &seenOtherCandidate;
        };

        uint8_t seenOtherCandidate = false;
        std::vector<int> candidateEdges;
        std::vector<int> offsetForCandidateTail(inputGraph.numVertices(), INFTY);
        using Search = Dijkstra<InputGraph, Weight, LabelSet, StopWhenMinDistanceReachedOrExistingCandidateSeen,
                dij::NoCriterion, StampedDistanceLabelContainer>;
        Search search(inputGraph, StopWhenMinDistanceReachedOrExistingCandidateSeen(minDistance, offsetForCandidateTail,
                                                                                    seenOtherCandidate));
        std::cout << "Computing candidates... " << std::flush;
        ProgressBar progressBar(inputGraph.numEdges());
        progressBar.setDotOutputInterval(1);
        progressBar.setPercentageOutputInterval(5);
        for (int i = 0; i < inputGraph.numEdges(); ++i) {
            const int e = edgesInOrder[i];
            seenOtherCandidate = static_cast<uint8_t>(false);
            search.run(inputGraph.edgeHead(e)); // Run until minimum distance is reached or another candidate is seen
            if (!static_cast<bool>(seenOtherCandidate)) {
                candidateEdges.push_back(e);
                auto &offset = offsetForCandidateTail[inputGraph.edgeTail(e)];
                offset = std::min(offset, inputGraph.travelTime(e));
            }
            ++progressBar;
        }
        std::cout << " done." << std::endl;

        // Write candidates to file
        std::cout << "Writing " << candidateEdges.size() << " / " << inputGraph.numEdges()
                  << " candidate edges to file... " << std::flush;
        bio::write(out, candidateEdges);
        std::cout << "done.\n";

    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] << " -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}