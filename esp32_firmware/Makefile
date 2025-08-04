# Makefile for TensorFlow Lite Model Test
# Usage: make run

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

# Check for existing TensorFlow Lite installation
TFLITE_STATIC_LIB = ./libtensorflow-lite.a
TFLITE_HEADERS = ./include/tensorflow/lite/interpreter.h

# Include directories - prioritize local build
ifeq ($(wildcard $(TFLITE_HEADERS)),)
    # Fallback to system paths if local build not found
    INCLUDES = -I/usr/local/lib/tensorflow/lite -I/usr/include/flatbuffers -I/usr/include/tensorflow
    LIBS = -ltensorflowlite -ldl -lpthread
    USING_SYSTEM = true
else
    # Use local build
    INCLUDES = -I./include -I./tensorflow
    LIBS = $(TFLITE_STATIC_LIB) -ldl -lpthread
    USING_SYSTEM = false
endif

# Source and target files
TARGET = tflite_test
SOURCE = tflite_test.cpp
MODEL = air_quality_model.tflite

# Default target
all: check-setup $(TARGET)

# Check if we have TensorFlow Lite available
check-setup:
ifeq ($(USING_SYSTEM),true)
    ifeq ($(wildcard /usr/include/tensorflow/lite/interpreter.h),)
		@echo "Error: TensorFlow Lite not found!"
		@echo "Please run 'make setup' to build TensorFlow Lite locally."
		@exit 1
    else
		@echo "Using system TensorFlow Lite installation"
    endif
else
	@echo "Using local TensorFlow Lite build"
endif

# Build the executable
$(TARGET): $(SOURCE)
	@echo "Compiling TensorFlow Lite test..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $(TARGET) $(SOURCE) $(LIBS)
	@echo "Build complete!"

# Setup target to build TensorFlow Lite locally
setup:
	@echo "Setting up TensorFlow Lite locally..."
	@if [ ! -d "tensorflow" ]; then \
		echo "Cloning TensorFlow (this may take a while)..."; \
		git clone https://github.com/tensorflow/tensorflow.git --depth 1; \
	fi
	@echo "Building TensorFlow Lite..."
	@cd tensorflow/tensorflow/lite && \
		mkdir -p build && cd build && \
		cmake .. -DTFLITE_ENABLE_XNNPACK=OFF -DCMAKE_BUILD_TYPE=Release && \
		make -j$(nproc)
	@echo "Copying library and headers..."
	@cp tensorflow/tensorflow/lite/build/libtensorflow-lite.a ./
	@mkdir -p include/tensorflow/lite
	@cp -r tensorflow/tensorflow/lite/*.h include/tensorflow/lite/ 2>/dev/null || true
	@cp -r tensorflow/tensorflow/lite/c include/tensorflow/lite/ 2>/dev/null || true
	@cp -r tensorflow/tensorflow/lite/core include/tensorflow/lite/ 2>/dev/null || true
	@cp -r tensorflow/tensorflow/lite/kernels include/tensorflow/lite/ 2>/dev/null || true
	@echo "Local TensorFlow Lite setup complete!"

# Install system dependencies
install-deps:
	@echo "Installing build dependencies..."
	sudo apt update
	sudo apt install -y build-essential cmake git
	sudo apt install -y libflatbuffers-dev
	@echo "Dependencies installed!"

# Run the test with default model
run: $(TARGET)
	@echo "Running TensorFlow Lite test..."
	@if [ ! -f $(MODEL) ]; then \
		echo "Warning: $(MODEL) not found!"; \
		echo "Running without model (test compilation only)..."; \
		./$(TARGET); \
	else \
		echo "Running with model: $(MODEL)"; \
		./$(TARGET) $(MODEL); \
	fi

# Run with custom model
run-model: $(TARGET)
	@read -p "Enter model path: " model; \
	if [ -f "$model" ]; then \
		./$(TARGET) "$model"; \
	else \
		echo "Error: Model file '$model' not found!"; \
		exit 1; \
	fi

# Test compilation only (no model needed)
test-compile: $(TARGET)
	@echo "Testing compilation..."
	./$(TARGET)

# Clean build artifacts
clean:
	rm -f $(TARGET)
	@echo "Build artifacts cleaned!"

# Clean everything including local TensorFlow build
clean-all: clean
	rm -rf tensorflow include libtensorflow-lite.a
	@echo "Everything cleaned!"

# Show build information
info:
	@echo "Build Configuration:"
	@echo "  Compiler: $(CXX)"
	@echo "  Flags: $(CXXFLAGS)"
	@echo "  Includes: $(INCLUDES)"
	@echo "  Libraries: $(LIBS)"
	@echo "  Using system TensorFlow Lite: $(USING_SYSTEM)"
	@echo "  Local static lib exists: $(wildcard $(TFLITE_STATIC_LIB))"
	@echo "  Local headers exist: $(wildcard $(TFLITE_HEADERS))"

# Help
help:
	@echo "TensorFlow Lite C++ Build System"
	@echo "================================="
	@echo ""
	@echo "Setup commands:"
	@echo "  make install-deps - Install system dependencies"
	@echo "  make setup       - Download and build TensorFlow Lite locally"
	@echo ""
	@echo "Build commands:"
	@echo "  make             - Build the executable (default)"
	@echo "  make all         - Same as make"
	@echo "  make info        - Show build configuration"
	@echo ""
	@echo "Run commands:"
	@echo "  make run         - Build and run with default model"
	@echo "  make run-model   - Build and run with custom model path"
	@echo "  make test-compile- Build and test without model"
	@echo ""
	@echo "Cleanup commands:"
	@echo "  make clean       - Remove built executable"
	@echo "  make clean-all   - Remove everything including TensorFlow source"
	@echo ""
	@echo "Note: If no local TensorFlow Lite build is found, the system"
	@echo "      will attempt to use system-installed libraries."
	@echo ""
	@echo "Model file: $(MODEL)"

.PHONY: all check-setup setup install-deps run run-model test-compile clean clean-all info help