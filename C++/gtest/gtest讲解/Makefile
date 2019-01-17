#set the google test dir
GTEST_DIR = 

#set the user dir
USER_DIR = 

#the exectuable file
TARGET = sample11

#include
INCLUDE += -I$(GTEST_DIR)/include

#source files
SOURCE += 

#libraries
LIBS += -L$(GTEST_DIR)/lib -lgtest -lgtest_main \
	-lpthread

# Flags passed to the C++ compiler.
CXXFLAGS += -g -O0 -Wall -Wextra

#compile
All : 
	$(CXX) $(CXXFLAGS) $(addprefix $(USER_DIR)/,$(SOURCE)) $(INCLUDE) $(LIBS) -o $(USER_DIR)/bin/$(TARGET)

#clean up
clean :
	rm -f $(TESTS) *.o

