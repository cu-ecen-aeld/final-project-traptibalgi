CXX = g++
CXXFLAGS = -O0 -g -I/usr/include/opencv4
LDFLAGS = `pkg-config --cflags --libs opencv4`
TARGET = capture

SRC = capture.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) -pthread $(LDFLAGS)

clean:
	rm -f $(TARGET)
