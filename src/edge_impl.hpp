#pragma once

namespace graph {

struct EdgeImpl {
  int weight = 0;
  int mark = 0;

  EdgeImpl() = default;

  EdgeImpl(int weight) : weight(weight) {}
};

}
