CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g -fPIC
OPT_CXXFLAGS = $(CXXFLAGS) -O3 -march=native

TARGET = build/libpf.so
SRCDIR = .
BUILDDIR = build

SOURCES = $(wildcard $(SRCDIR)/*.cc)
OBJECTS = $(patsubst $(SRCDIR)/%.cc,$(BUILDDIR)/%.o,$(SOURCES))

BENCH_SOURCES = main.cc
BENCH_OBJECTS = $(BUILDDIR)/bench.o

OPT_CXXFLAGS = $(CXXFLAGS) -O3 -march=native

all: $(TARGET) bench

bench: $(BUILDDIR)/bench $(filter-out $(BUILDDIR)/main.o,$(OBJECTS))

bench: $(BUILDDIR)/bench

bench_opt: $(BUILDDIR)/bench_opt

BENCH_LIBS = $(filter-out $(BUILDDIR)/main.o,$(OBJECTS))

$(BUILDDIR)/bench: $(BUILDDIR)/bench.o $(BENCH_LIBS) | $(BUILDDIR)
	$(CXX) -o $@ $^ -lstdc++

$(BUILDDIR)/bench_opt: $(BUILDDIR)/bench_opt.o $(BENCH_LIBS) | $(BUILDDIR)
	$(CXX) -o $@ $^ -lstdc++

$(BUILDDIR)/bench.o: $(SRCDIR)/$(BENCH_SOURCES) pf.h | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR)/bench_opt.o: $(SRCDIR)/$(BENCH_SOURCES) pf.h | $(BUILDDIR)
	$(CXX) $(OPT_CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS)
	$(CXX) -shared -o $@ $^

$(BUILDDIR)/%.o: $(SRCDIR)/%.cc pf.h | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean
