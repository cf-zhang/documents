cmake_minimum_required(VERSION 2.8)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
message(${GTEST_INCLUDE_DIRS})

# Link runTests with what we want to test and the GTest and pthread library
add_executable(simple_test testing_purpose.cpp)
target_link_libraries(simple_test ${GTEST_LIBRARIES} pthread)
