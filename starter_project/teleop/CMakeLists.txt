## Generate messages in the 'msg' folder
# Uncomment this after adding your first msg to the msg folder

## Starter Project
# CMake is a build scripting system
# It is a program whose sole purpose is to generate Makefiles (or Ninja build files)
# It is used extensively in industry
project(mrover VERSION 2025.0.0 LANGUAGES C CXX)

file(GLOB_RECURSE STARTER_PROJ_MESSAGE_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS 
        ${CMAKE_CURRENT_LIST_DIR}/msg/*.msg
)

list(APPEND MROVER_MESSAGE_FILE_PATHS ${STARTER_PROJ_MESSAGE_PATHS})