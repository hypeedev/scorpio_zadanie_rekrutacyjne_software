cmake_minimum_required(VERSION 3.10)

project(scorpio_recruitment_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Exercise library
add_library(backend SHARED
  src/backend/motor.cpp
  src/backend/basic_tester.cpp
)
target_include_directories(backend PUBLIC
  include/backend_interface/
  include/backend/
)
target_include_directories(backend PRIVATE include/solution_interface/)


# Solution library
file(GLOB solution_SRC "src/solution/*.cpp")
add_library(solution SHARED ${solution_SRC})
target_include_directories(solution PRIVATE
  include/solution/
  include/backend_interface/
)
target_include_directories(solution PUBLIC include/solution_interface/)


# Executable for the exercise
add_executable(scorpio_recruitment_task src/main.cpp)
target_include_directories(scorpio_recruitment_task PRIVATE include/)
target_link_libraries(scorpio_recruitment_task PRIVATE backend solution)


# Unit tests
if(BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
  file(GLOB test_SRC "unit_tests/test_*.cpp")
  add_executable(tests ${test_SRC}
    src/backend/motor.cpp
    src/backend/basic_tester.cpp
  )
  target_include_directories(tests PRIVATE
    include/backend_interface/
    include/backend/
    include/
  )
  target_link_libraries(tests PRIVATE GTest::GTest GTest::Main solution)
  gtest_discover_tests(tests)
endif()
