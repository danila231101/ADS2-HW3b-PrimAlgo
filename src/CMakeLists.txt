#### the list of targets ####
add_executable(ugraph
        ugraph/main.cpp        
        ugraph/ugraph.hpp
        ugraph/lbl_ugraph.hpp
        ugraph/ugraph_algos.hpp
        #
        grviz/gen_dot_writer.hpp
        grviz/ugraph_dotwriter.hpp
    )

