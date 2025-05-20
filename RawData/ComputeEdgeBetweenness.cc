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

inline void printUsage() {
    std::cout <<
              "Usage: ComputeEdgeBetweenness -g <file> -o <file>\n"
              "Compute the betweenness value of every edge (number of shortest paths that contain the edge) in a\n"
              "given graph and write the result to binary file.\n"
              "  -g <file>         input graph in binary format\n"
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
        auto outputFileName = clp.getValue<std::string>("o");
        if (!endsWith(outputFileName, ".bin"))
            outputFileName += ".bin";

        using Weight = TravelTimeAttribute;

        // Read the vehicle network from file.
        std::cout << "Reading vehicle network from file... " << std::flush;
        using InputGraph = StaticGraph<VertexAttrs<>, EdgeAttrs<Weight>>;
        std::ifstream graphFile(graphFileName, std::ios::binary);
        if (!graphFile.good())
            throw std::invalid_argument("file not found -- '" + graphFileName + "'");
        InputGraph inputGraph(graphFile);
        graphFile.close();
        std::cout << "done.\n";


        // Initialize output
        std::ofstream out(outputFileName);
        if (!out.good())
            throw std::invalid_argument("file cannot be opened -- '" + outputFileName + "'");

        // Compute betweenness
        using LabelSet = BasicLabelSet<0, ParentInfo::FULL_PARENT_INFO>;

        // When given to Dijkstra search, every time a vertex is settled, it is appended to the search space vector.
        struct RememberSearchSpace {
            RememberSearchSpace(std::vector<int> &searchSpace) : searchSpace(searchSpace) {}
            bool operator()(const int v, LabelSet::DistanceLabel &, const StampedDistanceLabelContainer<LabelSet::DistanceLabel> &) {
                searchSpace.push_back(v);
                return false;
            }
        private:
            std::vector<int> &searchSpace;
        };

        using Search = Dijkstra<InputGraph, Weight, LabelSet, RememberSearchSpace, dij::NoCriterion, StampedDistanceLabelContainer>;
        std::vector<int> searchSpace;
        searchSpace.reserve(inputGraph.numVertices());
        Search search(inputGraph, RememberSearchSpace(searchSpace));

        std::vector<int64_t> betweenness(inputGraph.numEdges(), 0);
        std::vector<int> subTreeSize(inputGraph.numVertices(), 0);

        std::cout << "Computing betweenness using Dijkstra searches... " << std::flush;
        ProgressBar progressBar(inputGraph.numVertices());
        progressBar.setDotOutputInterval(1);
        progressBar.setPercentageOutputInterval(5);
        for (int v = 0; v < inputGraph.numVertices(); ++v) {
            searchSpace.clear();
            search.run(v); // Run until exhaustion to build shortest-path tree

            // Compute size of subtree rooted at each vertex. searchSpace contains the vertices in the order they were
            // settled. Traverse it backwards to guarantee subtree size of v is finished when scanning v.
            KASSERT(*(searchSpace.rend() - 1) == v);
            for (auto it = searchSpace.rbegin(); it != searchSpace.rend() - 1; ++it) {
                const int child = *it;
                KASSERT(child != v);
                ++subTreeSize[child]; // Add self
                const int parentEdge = search.getParentEdge(child);
                const int parentVertex = search.getParentVertex(child);
                KASSERT(parentVertex >= 0 && parentVertex < inputGraph.numVertices());
                subTreeSize[parentVertex] += subTreeSize[child];
                betweenness[parentEdge] += subTreeSize[child];
                subTreeSize[child] = 0; // Reset for next iteration
            }
            KASSERT(subTreeSize[v] == static_cast<int64_t>(searchSpace.size()) - 1);
            subTreeSize[v] = 0; // Reset for next iteration

            ++progressBar;
        }
        std::cout << " done." << std::endl;

        bio::write(out, betweenness);


    } catch (std::exception &e) {
        std::cerr << argv[0] << ": " << e.what() << '\n';
        std::cerr << "Try '" << argv[0] <<" -help' for more information.\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}