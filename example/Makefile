CC = gcc
CXX = g++

CPPFLAGS = -I../src
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O4
CXXFLAGS = -g -Wall -O4 -std=c++14
LDFLAGS = -lpthread -lm

TARGETS := apriltag_demo opencv_demo

.PHONY: all
all: ../bin/apriltag_demo ../bin/opencv_demo ../bin/track_demo

../bin/apriltag_demo: apriltag_demo.o ../lib/libapriltag.a
	@mkdir -p ../bin
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

../bin/opencv_demo: opencv_demo.o ../lib/libapriltag.a
	@mkdir -p ../bin
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

../bin/track_demo: track_demo.o ../lib/libapriltag.a
	@mkdir -p ../bin
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv`

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
