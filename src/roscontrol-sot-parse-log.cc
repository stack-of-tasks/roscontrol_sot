#include <fstream>
#include <iostream>
#include <iomanip>

int main (int argc, char* argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " binary_file_name\n";
    return 1;
  }

  std::ifstream in (argv[1], std::ios::binary);
  if (!in.is_open() || !in.good()) {
    std::cerr << "Couldn't open file " << argv[1] << '\n';
    return 2;
  }

  // Read headers
  std::size_t nVector = 0, vectorSize = 0;
  in.read ((char*)&nVector   , sizeof(std::size_t));
  in.read ((char*)&vectorSize, sizeof(std::size_t));
  if (!in.good()) {
    std::cerr << "Couldn't parse file: " << argv[1] << '\n';
    return 3;
  }

  // Read datas
  double v;
  std::cout << std::setprecision(12) << std::setw(12) << std::setfill('0');

  for (std::size_t i=0; i < nVector; ++i) {
    for (std::size_t j=0; j < vectorSize; ++j) {
      in.read ((char*)&v, sizeof(double));
      if (!in.good()) {
        std::cerr << "Stopped to parse at (" << i << ',' << j
          << ") of file: " << argv[1] << '\n';
        return 4;
      }
      std::cout << v << ' ';
    }
    std::cout << '\n';
  }
  return 0;
}
