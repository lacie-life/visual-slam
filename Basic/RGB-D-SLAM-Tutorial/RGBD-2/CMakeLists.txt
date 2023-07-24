cmake_minimum_required(VERSION 3.19)
# Create Project
project( conver )


# Set StartUp Project (Option)
# (This setting is able to enable by using CMake 3.6.0 RC1 or later.)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
# Find PCL
find_package( PCL 1.8 REQUIRED )

# Find OpenCV
find_package( OpenCV REQUIRED )

if( PCL_FOUND AND OpenCV_FOUND )
    # [C/C++]>[General]>[Additional Include Directories]
    include_directories( ${PCL_INCLUDE_DIRS} )
    include_directories( ${OpenCV_INCLUDE_DIRS} )

    # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
    add_definitions( ${PCL_DEFINITIONS} )

    # For Use Not PreCompiled Features
    #add_definitions( -DPCL_NO_PRECOMPILE )

    # [Linker]>[General]>[Additional Library Directories]
    link_directories( ${PCL_LIBRARY_DIRS} )
    link_directories( ${OpenCV_LIB_DIR} )

    add_executable(${PROJECT_NAME}  main.cpp )
    # [Linker]>[Input]>[Additional Dependencies]
    target_link_libraries( ${PROJECT_NAME} ${PCL_LIBRARIES} )
    target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
endif()