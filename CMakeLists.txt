cmake_minimum_required(VERSION 3.16)
project(exemplecpp)

# Version du langage
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


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
include_directories(${INC_DIR} /usr/include/eigen3
${EXT_LIB_DIR}/websocketpp )

# Créer l'exécutable
add_executable(exemplecpp ${SOURCES})

# Lier les bibliothèques nécessaires
target_link_libraries(exemplecpp
    rt
    pthread
    wiringPi
)

# (optionnel) chemin des libs
link_directories(${LIB_DIR})