#include "iostream"

struct Hello {

  void print() {
    std::cout << "Hello, world!" << Hello::id << std::endl;
  }

  explicit Hello(int id) : id(id) {}

  int id;
};

int main() {
  Hello h(12);
  h.print();
  return 0;
}