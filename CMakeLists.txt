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

# mono-ranging
file(GLOB mono_ranging
    "src/mono-ranging/*.h"
    "src/mono-ranging/main.cpp"
)
add_executable( mono_ranging ${mono_ranging} )
target_link_libraries( mono_ranging ${OpenCV_LIBS} )

# gen_cali_img
file(GLOB gen_cali_img
    "src/mono-ranging/*.h"
    "src/mono-ranging/gen_cali_img.cpp"
)
add_executable( gen_cali_img ${gen_cali_img} )
target_link_libraries( gen_cali_img ${OpenCV_LIBS} )

# cali
file(GLOB cali
    "src/mono-ranging/*.h"
    "src/mono-ranging/cali.cpp"
)
add_executable( cali ${cali} )
target_link_libraries( cali ${OpenCV_LIBS} )


# pose
file(GLOB pose
    "src/mono-ranging/*.h"
    "src/mono-ranging/pose.cpp"
)
add_executable( pose ${pose} )
target_link_libraries( pose ${OpenCV_LIBS} )
