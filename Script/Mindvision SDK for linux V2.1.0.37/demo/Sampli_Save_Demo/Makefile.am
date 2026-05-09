AUTOMAKE_OPTIONS=foreign
bin_PROGRAMS=demo
demo_SOURCES=main.cpp   
demo_LDFLAGS= -L/lib

#demo_LDFLAGS = -fPIC -shared
INCLUDES=-I./inc   -I../../include 
demo_LDADD =      $(INTLLIBS)  -lMVSDK -lrt -lpthread
