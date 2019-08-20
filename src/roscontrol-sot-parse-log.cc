#include <fstream>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cstring>

void usage(char* bin) {
  std::cerr << "Usage: " << bin << " [--separator sep] binary_file_name\n";
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }
  int iarg = 1;
  std::string sep(" ");
  while (iarg + 1 < argc - 1) {
    if (strcmp(argv[iarg], "--separator") == 0) {
      sep = argv[iarg + 1];
      iarg += 2;
    } else {
      usage(argv[0]);
      return 1;
    }
  }
  assert(iarg == argc - 1);

  std::ifstream in(argv[iarg], std::ios::binary);
  if (!in.is_open() || !in.good()) {
    std::cerr << "Couldn't open file " << argv[iarg] << '\n';
    return 2;
  }

  // Read headers
  std::size_t nVector = 0, vectorSize = 0;
  in.read((char*)&nVector, sizeof(std::size_t));
  in.read((char*)&vectorSize, sizeof(std::size_t));
  if (!in.good()) {
    std::cerr << "Couldn't parse file: " << argv[iarg] << '\n';
    return 3;
  }

  // Read datas
  double v;
  std::cout << std::setprecision(12) << std::setw(12) << std::setfill('0');

  for (std::size_t i = 0; i < nVector; ++i) {
    for (std::size_t j = 0; j < vectorSize; ++j) {
      in.read((char*)&v, sizeof(double));
      if (!in.good()) {
        std::cerr << "Stopped to parse at (" << i << ',' << j
                  << ") of file: " << argv[iarg] << '\n';
        return 4;
      }
      std::cout << v << sep;
    }
    std::cout << '\n';
  }
  return 0;
}
