CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -fPIC

TARGET = build/libpf.so
SRCDIR = .
BUILDDIR = build

SOURCES = $(wildcard $(SRCDIR)/*.cc)
OBJECTS = $(patsubst $(SRCDIR)/%.cc,$(BUILDDIR)/%.o,$(SOURCES))

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) -shared -o $@ $^

$(BUILDDIR)/%.o: $(SRCDIR)/%.cc pf.h | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR)

.PHONY: all clean
