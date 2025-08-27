cmake_minimum_required(VERSION 3.16)
project(exemplecpp)

# Version du langage
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type Debug
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Flags pour debug
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -pthread")

# Dossiers
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(INC_DIR "${CMAKE_SOURCE_DIR}/inc")
set(LIB_DIR "${CMAKE_SOURCE_DIR}/lib")
set(EXT_LIB_DIR "/home/jules/libs")

# Fichiers source
file(GLOB_RECURSE SOURCES "${SRC_DIR}/*.cpp")

# Affiche les sources trouvées pour debug
message(STATUS "Sources trouvées: ${SOURCES}")



# Inclure les headers
include_directories(
    ${INC_DIR}
    /usr/include/eigen3
    /usr/local/include/uWebSockets
    /usr/local/include
    ${EXT_LIB_DIR}/websocketpp
    /home/jules/code/drone/external/uWebSockets/uSockets/src 
)

# Ajouter le dossier contenant uSockets.a
link_directories(/usr/local/lib)
link_directories(/usr/include)
# Créer l'exécutable
add_executable(exemplecpp ${SOURCES})

find_package(OpenCV REQUIRED)
target_include_directories(exemplecpp PRIVATE
    ${OpenCV_INCLUDE_DIRS}s
)
target_link_libraries(exemplecpp
    uWS
    z
    ssl
    crypto
    pthread
    rt
    wiringPi
    GeographicLib
    ${OpenCV_LIBS}
)
