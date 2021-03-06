#include <filesystem>
#include <iostream>

#include "io.hpp"
#include "kmst.hpp"

void solve(const std::filesystem::path& inFile,
           const std::filesystem::path& outFile) {
  std::cout << "Now solveing: " << inFile << std::endl;
  graph::Kmst task{graph::io::read(inFile)};
  task.approximate();
  if (task.check())
    std::cout << "Check passed\n" << std::endl;
  else {
    std::cout << "Check failed, solution is invalid\n" << std::endl;
    return;
  }
  graph::io::write(task, outFile);
}

// Чтобы скомпилить, нужно положить библиотеку boost в ../lib/boost/
// python3 imgs.py рисует картинки по файам задач/решений
// Так было выяснено, что распределение вершин совсем не равномерное
int main() {
  solve("../Taxicab_64.txt", "../Kurbatov_64.txt");
  solve("../Taxicab_128.txt", "../Kurbatov_128.txt");
  solve("../Taxicab_512.txt", "../Kurbatov_512.txt");
  solve("../Taxicab_2048.txt", "../Kurbatov_2048.txt");
  solve("../Taxicab_4096.txt", "../Kurbatov_4096.txt");
  return 0;
}
