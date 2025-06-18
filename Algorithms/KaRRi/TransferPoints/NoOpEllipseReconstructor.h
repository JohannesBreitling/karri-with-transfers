#pragma once

#include <vector>
#include "EdgeEllipseContainer.h"

namespace karri {

    class NoOpEllipseReconstructor {

    public:
        template<typename StatsT>
        NoOpEdgeEllipseContainer computeEllipses(const std::vector<int> &, StatsT&) {
            return {};
        }
    };

} // karri

