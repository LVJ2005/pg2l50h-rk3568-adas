CXX ?= g++
BUILD := build
SOURCES := sources/main.cpp sources/frame_capture.cpp sources/recognition_control.cpp \
 sources/adas_runtime.cpp sources/adas_rules.cpp sources/traffic_detector.cpp \
 sources/detector_decode.cpp sources/lane_detector.cpp sources/lane_vision.cpp sources/rgb_display.cpp sources/diagnostic_snapshot.cpp
OBJECTS := $(patsubst sources/%.cpp,$(BUILD)/adas_%.o,$(SOURCES))
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv)
OPENCV_LIBS := $(shell pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv)
GTK_PACKAGE := $(shell pkg-config --exists gtk+-3.0 && echo gtk+-3.0 || echo gtk+-2.0)
GTK_CFLAGS := $(shell pkg-config --cflags $(GTK_PACKAGE) 2>/dev/null)
GTK_LIBS := $(shell pkg-config --libs $(GTK_PACKAGE) 2>/dev/null)
CXXFLAGS += -O3 -std=c++17 -Wall -Wextra -Wpedantic -pthread -Iincludes $(OPENCV_CFLAGS) $(GTK_CFLAGS)
LANE_SIMD ?= 1
ifeq ($(LANE_SIMD),0)
CXXFLAGS += -DADAS_LANE_DISABLE_SIMD
endif
LDLIBS += $(OPENCV_LIBS) $(GTK_LIBS) -lrknnrt -pthread

.PHONY: all clean test host-test check-deps
all: check-deps $(BUILD)/app
check-deps:
	@pkg-config --exists $(GTK_PACKAGE) || { echo "ERROR: GTK development files missing; install libgtk-3-dev or libgtk2.0-dev"; exit 1; }
$(BUILD)/app: $(OBJECTS)
	$(CXX) $^ -o $@ $(LDLIBS)
$(BUILD)/adas_%.o: sources/%.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@
$(BUILD)/adas_core_test: sources/tests/adas_core_test.cpp sources/adas_rules.cpp sources/adas_runtime.cpp sources/detector_decode.cpp sources/lane_detector.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) -O2 -std=c++17 -Wall -Wextra -pthread -Iincludes $(filter %.cpp,$^) -o $@
$(BUILD)/adas_preprocess_test: sources/tests/adas_preprocess_test.cpp sources/traffic_detector.cpp sources/detector_decode.cpp sources/lane_detector.cpp sources/lane_vision.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDLIBS)
test: $(BUILD)/adas_core_test $(BUILD)/adas_preprocess_test $(BUILD)/lane_structure_test
	./$(BUILD)/adas_core_test
	./$(BUILD)/adas_preprocess_test
	./$(BUILD)/lane_structure_test
host-test: $(BUILD)/adas_core_test $(BUILD)/lane_structure_test
	$(CXX) $(CXXFLAGS) -DADAS_HOST_TEST sources/tests/adas_preprocess_test.cpp sources/traffic_detector.cpp sources/detector_decode.cpp sources/lane_detector.cpp sources/lane_vision.cpp sources/tests/rknn_stub.cpp -o $(BUILD)/adas_preprocess_host_test $(OPENCV_LIBS) -pthread
	$(CXX) $(CXXFLAGS) $(SOURCES) sources/tests/rknn_stub.cpp -o $(BUILD)/app_host_test $(OPENCV_LIBS) $(GTK_LIBS) -pthread
	./$(BUILD)/adas_core_test
	./$(BUILD)/adas_preprocess_host_test
	./$(BUILD)/lane_structure_test
clean:
	rm -f $(OBJECTS) $(BUILD)/app $(BUILD)/adas_core_test $(BUILD)/adas_preprocess_test

.PHONY: display-test
display-test:
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) sources/tests/rgb_display_test.cpp sources/rgb_display.cpp -o $(BUILD)/rgb_display_test $(OPENCV_LIBS) $(GTK_LIBS) -pthread
	GDK_BACKEND=x11 xvfb-run -a ./$(BUILD)/rgb_display_test
	QT_QPA_PLATFORM=xcb xvfb-run -a ./$(BUILD)/rgb_display_test opencv

.PHONY: lane-replay
lane-replay: $(BUILD)/lane_replay
$(BUILD)/lane_replay: sources/tests/lane_replay.cpp sources/lane_detector.cpp sources/lane_vision.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) sources/tests/lane_replay.cpp sources/lane_detector.cpp sources/lane_vision.cpp -o $@ $(OPENCV_LIBS) -pthread

.PHONY: lane-image-test
lane-image-test: $(BUILD)/lane_dataset_test
	./$(BUILD)/lane_dataset_test sources/tests/data/lane_20261019 $(BUILD)/lane_image_report.json --assert
$(BUILD)/lane_dataset_test: sources/tests/lane_dataset_test.cpp sources/lane_detector.cpp sources/lane_vision.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) sources/tests/lane_dataset_test.cpp sources/lane_detector.cpp sources/lane_vision.cpp -o $@ $(OPENCV_LIBS) -pthread

$(BUILD)/lane_structure_test: sources/tests/lane_structure_test.cpp sources/lane_detector.cpp sources/lane_vision.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) sources/tests/lane_structure_test.cpp sources/lane_detector.cpp sources/lane_vision.cpp -o $@ $(OPENCV_LIBS) -pthread

.PHONY: lane-benchmark
lane-benchmark: $(BUILD)/lane_benchmark
$(BUILD)/lane_benchmark: sources/tests/lane_benchmark.cpp sources/lane_detector.cpp sources/lane_vision.cpp $(wildcard includes/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DADAS_LANE_TIMINGS sources/tests/lane_benchmark.cpp sources/lane_detector.cpp sources/lane_vision.cpp -o $@ $(OPENCV_LIBS) -pthread
