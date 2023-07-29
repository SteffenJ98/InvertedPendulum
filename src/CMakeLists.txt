project(InvertedPendulum)

add_library(controller_lib 
        STATIC controller.cpp)
     

target_include_directories(controller_lib 
PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/../include)


