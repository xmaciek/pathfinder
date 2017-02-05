cmake_minimum_required( VERSION 3.5.1 )
project( PathFinder_unit_test )

find_package( GTest REQUIRED )


set( UTEST_BINNAME "pathfinder_unit_test" )
add_executable( ${UTEST_BINNAME} test.cpp )
target_link_libraries( ${UTEST_BINNAME} gtest )
