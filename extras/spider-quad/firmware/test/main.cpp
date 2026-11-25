#include "check.h"
int main() {
  int failedTests = 0;
  for (auto &t : testRegistry()) {
    int before = testFailures();
    t.fn();
    if (testFailures() != before) { std::printf("✖ %s\n", t.name); ++failedTests; }
  }
  std::printf("tests %zu, failed %d\n", testRegistry().size(), failedTests);
  return failedTests ? 1 : 0;
}
