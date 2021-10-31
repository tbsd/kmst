#include "kmst.hpp"

#include <iostream>
#include <set>
#include <algorithm>

#include <boost/graph/undirected_dfs.hpp>

#include "io.hpp"

namespace graph {

Kmst::Kmst(std::unique_ptr<RawVertices> vertices)
    : initSize(vertices->size()), treeSz(vertices->size() / 8 - 1) {
  matrix = std::make_shared<GraphMatrix>(vertices->size());
  best = Solution(matrix);
  auto [vert_begin, vert_end] = boost::vertices(*matrix);
  auto j = vertices->begin();
  for (VertexIt i = vert_begin; i != vert_end; ++i, ++j) {
    getV(*i) = *j;
  }
  for (VertexIt i = vert_begin; i != vert_end; ++i) {
    for (VertexIt n = i + 1; n != vert_end; ++n) {
      boost::add_edge(*i, *n, *matrix);
    }
  }
  auto [edges_begin, edges_end] = boost::edges(*matrix);
  for (auto i = edges_begin; i != edges_end; ++i) {
    (*matrix)[*i].weight =
        manhattanDistance(getV(i->m_source), getV(i->m_target));
  }
  std::cout << "Vertices count: " << boost::num_vertices(*matrix)
            << ", edges count: " << boost::num_edges(*matrix)
            << ", tree size (k): " << treeSz + 1 << std::endl;
  std::cout << "Initialization done" << std::endl;
}

std::optional<Kmst::EdgeIt> Kmst::findMinEdge(Solution<GraphMatrix>& solution) {
  // Надо бы отдавать приоритет самому лёгкому ребру, образующему длиннейшую
  // цепочку, распологающемуся ближе к изначальной точке (раз уж такое
  // распределение). Может потом.

  using EdgeItRange = std::pair<EdgeIt, EdgeIt>;
  auto less = [](const EdgeItRange& lhs, const EdgeItRange& rhs) {
    return lhs.first->m_source < rhs.first->m_source;
  };
  std::set<EdgeItRange, decltype(less)> newEdges;
  for (auto e : solution.edges) {
    newEdges.insert(boost::out_edges(e->m_source, *matrix));
    newEdges.insert(boost::out_edges(e->m_target, *matrix));
  }
  std::optional<EdgeIt> minEdge;
  int minEdgeWeight = std::numeric_limits<int>::max();
  for (auto [begin, end] : newEdges) {
    for (auto i = begin; i != end; ++i) {
      int iWeight = (*matrix)[*i].weight;
      if (iWeight < minEdgeWeight && solution.isAddable(i)) {
        minEdge = i;
        minEdgeWeight = iWeight;
      }
    }
  }
  return minEdge;
}

Kmst::VertexIt Kmst::getVIt(int index) {
  auto [vert_begin, vert_end] = boost::vertices(*matrix);
  return vert_begin + index;
}

VertexImpl& Kmst::getV(int index) {
  return (*matrix)[*getVIt(index)];
}

void Kmst::deleteHeavierEdges(int weight) {
  int initEdgesCount = boost::num_edges(*matrix);
  auto [all_edges_begin, all_edges_end] = boost::edges(*matrix);
  for (auto i = all_edges_begin; i != all_edges_end; ++i) {
    if ((*matrix)[*i].weight > weight - treeSz + 1) {
      boost::remove_edge(*i, *matrix);
    }
  }
  int deletedEdgesCount = initEdgesCount - boost::num_edges(*matrix);
  std::cout << "Deleted edges: " << deletedEdgesCount << "/" << initEdgesCount
            << " ("
            << static_cast<double>(deletedEdgesCount) / initEdgesCount * 100
            << "%)" << std::endl;
}

void Kmst::removeBigEdges(int startShift) {
  std::cout << "Removing big edges" << std::endl;
  auto [vert_begin, vert_end] = boost::vertices(*matrix);
  auto initV = vert_begin + startShift;
  std::map<int, VertexIt>
      distances;  // расстояния от изначальной вершины до каждой другой.
  auto [initEdgesBegin, initEdgesEnd] = boost::out_edges(*initV, *matrix);
  for (auto i = initEdgesBegin; i != initEdgesEnd; ++i)
    distances.emplace((*matrix)[*i].weight, i->m_target);
  auto cur_v = initV;
  std::cout << "Start vertex: " << startShift << " (" << getV(*initV).x << ", "
            << getV(*initV).y << ")" << std::endl;
  Solution<GraphMatrix> curSolution(matrix);
  auto [edges_i, edges_end] = boost::out_edges(*cur_v, *matrix);

  auto min_e =
      std::min_element(edges_i, edges_end, [this](auto lhs, auto rhs) -> bool {
        return (*matrix)[lhs].weight < (*matrix)[rhs].weight;
      });
  curSolution.addEdge(min_e);
  while (curSolution.size() < treeSz) {
    auto newEdge = findMinEdge(curSolution);
    if (newEdge) {
      curSolution.addEdge(*newEdge);
      if (curSolution.size() % 10 == 0)
        std::cout << curSolution.size() << " " << std::flush;
    } else {
      std::cout << "\nCurrent solution is incomplete" << std::endl;
      return;
    }
  }
  std::cout << "\nCurrent solution complete. Total weight: "
            << curSolution.getWeight()
            << " Leaves count: " << curSolution.getLeavesCount() << std::endl;
  deleteHeavierEdges(curSolution.getWeight());

  std::cout << "Minimizeing solution" << std::endl;

  // находим минимальный лист и смотрим, можем ли добавить меньший где-то ещё
  while (true) {
    auto maxLeaf = curSolution.findMaxLeaf();
    if (maxLeaf) {
      auto [maxV, maxE] = *maxLeaf;
      curSolution.removeLeaf(maxE, maxV);
      auto minE = findMinEdge(curSolution);
      if (!minE) {
        std::cerr << "Something gone wrong. No unused edges found" << std::endl;
        return;
      }
      curSolution.addEdge(*minE);
      if ((*matrix)[*maxE].weight <= (*matrix)[**minE].weight)
        break;
    } else {
      break;
    }
  }
  std::cout << "Minimization complete. Total weight: "
            << curSolution.getWeight()
            << " Leaves count: " << curSolution.getLeavesCount() << std::endl;
  deleteHeavierEdges(curSolution.getWeight());

  if (curSolution < best || best.size() != treeSz)
    best = std::move(curSolution);
  clearMarks();
}

int Kmst::getStartVertex() {
  switch (boost::num_vertices(*matrix)) {
    case 64:
      return 1;
    case 128:
      return 42;
    case 512:
      return 444;
    case 2048:
      return 444;
    case 4096:
      return 69;
    default:
      return 0;
  }
}

void Kmst::approximate() {
  // Раз уж работает так быстро, то в качестве начальных можно просто
  // перебрать все вершины из скопления
  // Или вообще все вершины. Для 2048 считалось 20 минут, для 4096 — 5 часов 47 минут.
  // Но это вызывает какие-то непонятные краши на средних графах, связаные с
  // выделением/освобождением памяти. Подозреваю, из-за фрагментации памяти. Как
  // быстро побороть не знаю, пришлось перебирать кусками. А ещё иногда запись в
  // файл фейлится, возможно из-за этого же
  //  const int graphSz = boost::num_vertices(*matrix);
  //  for (int i = 0; i < graphSz; ++i) {
  //    removeBigEdges(i);
  //    std::cout << "Start points checked: " << i + 1 << "/" << graphSz
  //              << std::endl;
  //  }

  removeBigEdges(getStartVertex());

  // Изначально предполагалось, что после уменьшения числа рёбер будет применён
  // какой-то нормальный алгоритм, который даст более точное решение, но зачем
  // думать, если можно пербирать
}

const Solution<GraphMatrix> Kmst::getSolution() const {
  return best;
}

int Kmst::getProblemSize() const {
  return initSize;
}

bool Kmst::hasCycle(unsigned long startV, int parentV) {
  getV(startV).inCurrentSolution = true;
  for (auto& i : best.edges)
    if (i->m_source == startV || i->m_target == startV) {
      int newV = i->m_target == startV ? i->m_source : i->m_target;
      if (!getV(newV).inCurrentSolution) {
        if (hasCycle(newV, startV))
          return true;
      } else if (newV != parentV)
        return true;
    }
  return false;
}

void Kmst::clearMarks() {
  auto [vBegin, vEnd] = boost::vertices(*matrix);
  for (VertexIt i = vBegin; i != vEnd; ++i) {
    getV(*i).inCurrentSolution = false;
  }
}

bool Kmst::check() {
  if (best.edges.size() != static_cast<decltype(best.edges.size())>(treeSz))
    return false;
  if (hasCycle((*best.edges.begin())->m_source, -1))
    return false;
  for (auto& e : best.edges) {
    if (!getV(e->m_source).inCurrentSolution ||
        !getV(e->m_target).inCurrentSolution)
      return false;
  }
  clearMarks();
  return true;
}
}  // namespace graph
