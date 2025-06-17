#pragma once

#include <vector>
#include "EdgeEllipseContainer.h"

namespace karri {

    class NoOpEllipseReconstructor {

    public:
        NoOpEdgeEllipseContainer computeEllipses(const std::vector<int> &) {
            return {};
        }
    };

} // karri

