# Host tests for the spider quad motion core: make -C spider-quad/firmware/test
# The Command Line Tools SDK on this Mac fails to link; use Xcode's toolchain when present.
XCODE = /Applications/Xcode.app/Contents/Developer
ifneq ($(wildcard $(XCODE)),)
CXX = $(XCODE)/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang++
SYSROOT = -isysroot $(XCODE)/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
endif
CXXFLAGS = -std=c++17 -O1 -g -Wall -Wextra $(SYSROOT) -I../spider_quad -I.
SRC = $(wildcard ../spider_quad/src/core/*.cpp) $(wildcard *.cpp)

run: $(SRC) $(wildcard *.h) $(wildcard ../spider_quad/src/core/*.h)
	$(CXX) $(CXXFLAGS) $(SRC) -o run
	./run

.PHONY: run
