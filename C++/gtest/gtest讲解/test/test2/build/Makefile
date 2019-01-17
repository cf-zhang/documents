#set the google test dir
GTEST_DIR = /home/cfzhang/opensource/googletest/googletest

#set the user dir
USER_DIR = /home/cfzhang/test/test2

#the exectuable file
TARGET = sample12

#include
INCLUDE += -I$(GTEST_DIR)/include

#source files
SOURCE += src/sample2.cc\
		  src/sample2_unittest.cc

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

