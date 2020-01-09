cmake_minimum_required(VERSION 3.1)
project( visloc  LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ----- Find dependencies -----
# OpenCV
find_package( OpenCV 3.0 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# Photographer
file(GLOB Photographer
    "src/Photographer/*.h"
    "src/Photographer/*.cpp"
)
add_executable( Photographer ${Photographer} )
target_link_libraries( Photographer ${OpenCV_LIBS} )

# vo
file(GLOB vo
    "src/vo/*.h"
    "src/vo/*.cpp"
)
add_executable( vo ${vo} )
target_link_libraries( vo ${OpenCV_LIBS} )
