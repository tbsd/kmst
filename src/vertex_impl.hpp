#pragma once

#include <cmath>

namespace graph {

  struct VertexImpl {
    int x = 0;
    int y = 0;
    bool inCurrentSolution = false;

    VertexImpl() = default;
    VertexImpl(int x, int y) : x(x), y(y) {}
  };

int manhattanDistance(const VertexImpl& point1, const VertexImpl& point2);

}
