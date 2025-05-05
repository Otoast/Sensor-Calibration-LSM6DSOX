# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++20 -g -pedantic -Wall -Weffc++ -Wextra -Werror \
           -fsanitize=address,undefined -fno-omit-frame-pointer \
		   -I/usr/local/include

LINKS = -lbcm2835

NEEDED_OBJECTS = sensor_reading.o registers.o



main: main.cpp $(NEEDED_OBJECTS)
	$(CXX) $^ $(CXXFLAGS) -o $@ $(LINKS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm *.o main sensor_calibration


sensor_calibration: sensor_calib.cpp $(NEEDED_OBJECTS)
	$(CXX) $^ $(CXXFLAGS) -o $@ $(LINKS)
