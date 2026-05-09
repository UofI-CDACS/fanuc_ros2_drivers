CXX ?= g++
CFLAGS =`pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
INCLUDE= -I../../include

all: main
main:  
	$(CXX) $(CFLAGS)  -o main  main.cpp $(LIBS)   $(INCLUDE)   -lMVSDK
clean:
	rm -f *.o 
	rm -f main 
