#pragma once

#include <filesystem>
#include <fstream>
#include <utility>

#include "solution.hpp"
#include "types.hpp"

namespace graph {
  // k-minimum spanning tree
  class Kmst {
    using Vertex = boost::graph_traits<GraphMatrix>::vertex_descriptor;
    using Edge = GraphMatrix::edge_descriptor;
    using EdgeIt = GraphMatrix::out_edge_iterator;
    using VertexIt = GraphMatrix::vertex_iterator;

    public:
    Kmst(std::unique_ptr<RawVertices> vertices);

    // Найти приближённое решение
    void approximate();

    const Solution<GraphMatrix> getSolution() const;

    int getProblemSize() const;

    // Соответствует ли решение условиям задачи
    bool check();

    private:
    /* Удалить слишком тяжёлые рёбра, точно не входящие в решение.
     * Жадно строим дерево, считаем его вес w, удаляем все рёбра с весом
     * больше w - treeSz + 1.
     * +1, т.к. нужно оставить деревья с таким же весом, но меньшим числом листьев
     */
    void removeBigEdges(int startShift);

    /* Получить индекс вершины, оптимальной для начала поиска решения.
     * Вершины изначально выбраны руками на основе полученных картинок (потом
     * сделал перебор по всем вершинам).
     * Для общего случая имеет смысл искать
     * скопления точек и в них выбирать центральную или с наименьшим ребом
     */
    int getStartVertex();

    // Вершина по индексу
    VertexImpl& getV(int index);

    // Итератор вершины по индексу
    VertexIt getVIt(int index);

    std::optional<EdgeIt> findMinEdge(Solution<GraphMatrix>& solution);

    void deleteHeavierEdges(int weight);

    bool hasCycle(unsigned long startV, int parentV);

    void clearMarks();

    const int initSize;
    const int treeSz;  // число рёбер в решении (k)
    std::shared_ptr<GraphMatrix> matrix;
    int bestWeight = std::numeric_limits<int>::max();
    int bestLeavesCount = std::numeric_limits<int>::max();
    Solution<GraphMatrix> best;
  };

  }  // namespace graph
