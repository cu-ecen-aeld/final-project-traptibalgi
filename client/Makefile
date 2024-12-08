CXX = $(CROSS_COMPILE)g++
CXXFLAGS = -O0 -g -I/usr/include/opencv4 -I/usr/local/include -I/usr/include
LDFLAGS = `pkg-config --cflags --libs opencv4` -L/usr/local/lib -lwiringPi -L/usr/lib
TARGET = trafficlight

SRC = trafficlight.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) -pthread $(LDFLAGS)

clean:
	rm -f $(TARGET)



