CC = g++
CFLAGS_CV = $(shell pkg-config --cflags opencv4)
LIBS_CV = $(shell pkg-config --libs opencv4)
CFLAGS_EIGEN = $(shell pkg-config --cflags eigen3)
LIBS_EIGEN = $(shell pkg-config --libs eigen3)
LIBS_NLOPT= -lnlopt -lm

CFLAGS = $(CFLAGS_CV) $(CFLAGS_EIGEN) \
    -Wall \
    -Wextra \
    -Werror \
	-g

LIBS = $(LIBS_CV) $(LIBS_EIGEN) $(LIBS_NLOPT)

OBJECTS = \
	build/main.o \
	build/feature_processor.o \
	build/matching_points.o \
	build/movement.o \
	build/position.o \

INCLUDES=-Isrc/

all: bin/main

build/%.o: src/%.cpp
	@mkdir -p build
	@echo 'Building $<'
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

bin/main: $(OBJECTS)
	@mkdir -p bin
	@echo 'Linking $@'
	@$(CC) $(CFLAGS) $(OBJECTS) -o $@ $(LIBS)

gdb:
	gdb -ex "layout src" bin/main

clean:
	@rm -r bin
	@rm -r build
