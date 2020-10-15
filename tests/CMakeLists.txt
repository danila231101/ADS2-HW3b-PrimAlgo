include_directories(../src)
#include_directories(../src/ugraph)

include_directories(.)

add_executable(tests
    # list of tests
    ugraph_test.cpp
    lbl_ugraph_test.cpp
    ugraph_algos_test.cpp
    ugraph_dotwriter_test.cpp

    # list of sources
    ../src/ugraph/ugraph.hpp
    ../src/ugraph/lbl_ugraph.hpp
    ../src/ugraph/ugraph_algos.hpp
    ../src/grviz/ugraph_dotwriter.hpp
    
    # gtest sources
    gtest/gtest-all.cc
    gtest/gtest_main.cc
)

# add pthread for unix systems
if (UNIX)
    target_link_libraries(tests pthread)
endif ()
