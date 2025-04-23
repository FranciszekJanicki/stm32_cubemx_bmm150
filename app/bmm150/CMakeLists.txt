add_library(bmm150 STATIC)

target_sources(bmm150 PRIVATE 
    "bmm150.cpp"
    "bmm150_utility.cpp"
)

target_include_directories(bmm150 PUBLIC 
)

target_link_libraries(bmm150 PUBLIC
)

target_compile_options(bmm150 PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)
