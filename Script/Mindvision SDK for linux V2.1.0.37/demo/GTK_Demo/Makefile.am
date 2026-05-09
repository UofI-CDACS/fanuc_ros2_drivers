AUTOMAKE_OPTIONS=foreign
bin_PROGRAMS=GTK_demo
GTK_demo_SOURCES=GTK_demo.cpp  ./src/callbacks.cpp    ./src/Demo.cpp   ./src/interface.cpp   ./src/support.cpp        
GTK_demo_LDFLAGS= -L../../lib/


INCLUDES=-I./inc   -I../../include @PACKAGE_CFLAGS@ 
GTK_demo_LDADD = @PACKAGE_LIBS@  -lpthread -lrt  -lgthread-2.0    $(INTLLIBS) -lMVSDK
