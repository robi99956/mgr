CC = g++
CFLAGS_CV = $(shell pkg-config --cflags opencv4)
LIBS_CV = $(shell pkg-config --libs opencv4)

CFLAGS = $(CFLAGS_CV) \
    -Wall \
    -Wextra \
    -Werror

LIBS = $(LIBS_CV)

OBJECTS = \
	build/main.o \
	build/feature_processor.o \
	build/matching_points.o \
	build/movement.o \
	build/position.o \
	build/matrix_math.o \
	build/optimization.o

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

clean:
	@rm -r bin
	@rm -r build
