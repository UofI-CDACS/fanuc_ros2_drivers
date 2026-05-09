AUTOMAKE_OPTIONS=foreign
bin_PROGRAMS=multicamera_demo
multicamera_demo_SOURCES=multicamera_demo.cpp  ./src/callbacks.cpp    ./src/Demo.cpp   ./src/interface.cpp   ./src/support.cpp        
#multicamera_demo_LDFLAGS= -L../../lib/gcc-4.4.3/
multicamera_demo_LDFLAGS= -L/lib
INCLUDES=-I./inc   -I../../include @PACKAGE_CFLAGS@ 
multicamera_demo_LDADD = @PACKAGE_LIBS@  -lpthread -lrt  -lgthread-2.0    $(INTLLIBS) -lMVSDK
