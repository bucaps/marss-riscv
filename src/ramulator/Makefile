SRCDIR := src
OBJDIR := obj
MAIN := $(SRCDIR)/Main.cpp
SRCS := $(filter-out $(MAIN) $(SRCDIR)/Gem5Wrapper.cpp, $(wildcard $(SRCDIR)/*.cpp))
OBJS := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SRCS))

# For shared library
POBJS := $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.po, $(SRCS))

# Ramulator currently supports g++ 5.1+ or clang++ 3.4+.  It will NOT work with
#   g++ 4.x due to an internal compiler error when processing lambda functions.
#CXX := clang++
CXX := g++
CXXFLAGS := -O2 -std=c++11 -g -Wall

.PHONY: all clean depend

all: depend ramulator

clean:
	rm -f ramulator libramulator.a libramulator.so
	rm -rf $(OBJDIR)

depend: $(OBJDIR)/.depend


$(OBJDIR)/.depend: $(SRCS)
	@mkdir -p $(OBJDIR)
	@rm -f $(OBJDIR)/.depend
	@$(foreach SRC, $(SRCS), $(CXX) $(CXXFLAGS) -DRAMULATOR -MM -MT $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SRC)) $(SRC) >> $(OBJDIR)/.depend ;)

ifneq ($(MAKECMDGOALS),clean)
-include $(OBJDIR)/.depend
endif

ramulator: $(MAIN) $(OBJS) $(SRCDIR)/*.h | depend
	$(CXX) $(CXXFLAGS) -DRAMULATOR -o $@ $(MAIN) $(OBJS)

libramulator.a: $(OBJS) $(OBJDIR)/Gem5Wrapper.o
	ar rcs -o $@ $(OBJS) $(OBJDIR)/Gem5Wrapper.o

libramulator.so: $(POBJS) $(OBJDIR)/Gem5Wrapper.po
	$(CXX) $(CXXFLAGS) -shared -Wl,-soname,$@ -o $@ $^

$(OBJS): | $(OBJDIR)

$(OBJDIR): 
	@mkdir -p $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -DRAMULATOR -c -o $@ $<

$(OBJDIR)/%.po: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -DRAMULATOR -fPIC -o $@ -c $<