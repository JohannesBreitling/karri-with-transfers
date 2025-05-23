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

#pragma once

#include "Tools/Constants.h"

// Represents a vertex in a detour ellipse with according distances from previous stop to vertex and from vertex to
// next stop.
struct VertexInEllipse {

    VertexInEllipse(const int vertex, const int distToVertex, const int distFromVertex)
            : vertex(vertex), distToVertex(distToVertex), distFromVertex(distFromVertex) {};

    int vertex = INVALID_VERTEX;
    int distToVertex = INFTY;
    int distFromVertex = INFTY;
};

struct EdgeInEllipse {

    EdgeInEllipse(const int edge, const int distToTail, const int distFromHead)
            : edge(edge), distToTail(distToTail), distFromHead(distFromHead) {};

    int edge = INVALID_EDGE;
    int distToTail = INFTY;
    int distFromHead = INFTY;
};