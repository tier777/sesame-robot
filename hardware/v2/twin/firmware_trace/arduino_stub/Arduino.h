// Минимальная заглушка Arduino для записи поз прошивки на компьютере.
#pragma once
#include <cstdint>
#include <cstring>
#include <string>

typedef uint8_t byte;

class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  String(int v) : s_(std::to_string(v)) {}
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator==(const char* o) const { return s_ == o; }
  bool operator!=(const String& o) const { return s_ != o.s_; }
  bool operator!=(const char* o) const { return s_ != o; }
  String operator+(const String& o) const { return String(s_ + o.s_); }
  const char* c_str() const { return s_.c_str(); }
  std::size_t length() const { return s_.size(); }

 private:
  std::string s_;
};

struct SerialStub {
  template <typename T> void print(const T&) {}
  template <typename T> void println(const T&) {}
  void println() {}
};
static SerialStub Serial;

#define F(x) (x)

template <typename T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
