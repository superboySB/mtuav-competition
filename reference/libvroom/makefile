CXX ?= g++

# for release
# CXXFLAGS = -I/root/workspace/vroom/src -std=c++17 -Wextra -Wpedantic -Wall -O3
# for debug
CXXFLAGS = -I/workspace/vroom/src -std=c++17 -Wextra -Wpedantic -Wall -g -O0

LDLIBS = -L/workspace/vroom/lib/ -lvroom -lpthread -lssl -lcrypto

# Checking for libglpk based on whether the header file is found as
# glpk does not provide a pkg-config setup.
GLPK_HEADER := $(strip $(wildcard /usr/include/glpk.h))
ifneq ($(GLPK_HEADER),)
	LDLIBS += -lglpk
endif

MAIN = ./libvroom-example
SRC = libvroom.cpp

all : $(MAIN)

$(MAIN) : $(SRC)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

clean :
	$(RM) $(MAIN)