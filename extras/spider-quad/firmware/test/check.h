// Minimal test harness: TEST(name) { CHECK(...); }  — run by test/main.cpp.
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

struct TestCase { const char *name; std::function<void()> fn; };
inline std::vector<TestCase> &testRegistry() { static std::vector<TestCase> r; return r; }
inline int &testFailures() { static int f = 0; return f; }
struct TestAdd { TestAdd(const char *n, std::function<void()> f) { testRegistry().push_back({n, f}); } };

#define TEST_CAT2(a, b) a##b
#define TEST_CAT(a, b) TEST_CAT2(a, b)
#define TEST(name) static void TEST_CAT(test_, __LINE__)(); \
  static TestAdd TEST_CAT(add_, __LINE__)(name, TEST_CAT(test_, __LINE__)); \
  static void TEST_CAT(test_, __LINE__)()
#define CHECK(c) do { if (!(c)) { std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); ++testFailures(); } } while (0)
#define CHECK_NEAR(a, b, eps) do { double _a = (a), _b = (b); if (!(std::fabs(_a - _b) <= (eps))) { \
  std::printf("  FAIL %s:%d: %s = %.9g, expected %.9g\n", __FILE__, __LINE__, #a, _a, _b); ++testFailures(); } } while (0)
