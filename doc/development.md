Development Guide
==================
The [```piperbase```](../piper/base) library implements the basic functionality needed to load robots, problems, etc and interface the trajectory controller for the robot via ROS, and can be used by any algorithm within PIPER. Each remaining directory in [```/piper```](../piper) is associated to interfacing a given algorithm to any robot via ROS. This modular construction allows for easy and selective installation based on what algorithms are desired to be used.

To add your own algorithm say ```myAlgo``` to the PIPER package you need to create a module for it by following these steps:

- Add any dependencies say ```myDepend``` and the installation set up to the [CMakeLists.txt](../CMakeLists.txt) file
  - Create an option to install your module ```myAlgo_interface```
  
    ```cmake
    # options for which interfaces need to be built and installed
    ...
    option(BUILD_myAlgo_INTERFACE "whether to build interface for myAlgo" OFF)
    ```

  - Add option when building all interfaces
  
    ```cmake
    ################################################
    ##                    all                     ##
    ################################################
    if(BUILD_ALL_INTERFACE)
      ...
      set(BUILD_myAlgo_INTERFACE ON)
    endif()
    ```
  
  - Add module installation settings
  
    ```cmake
    ################################################
    ##              myAlgo_interface              ##
    ################################################
    if(BUILD_myAlgo_INTERFACE)
      find_package(myDepend REQUIRED)
      include_directories(${myDepend_INCLUDE_DIR})
      set(myDepend_LIBRARIES myDepend)

      include_directories(piper/myAlgo_interface/)

      file(GLOB myAlgo_INTERFACE_SRC "piper/myAlgo_interface/*.cpp")
      add_executable(myAlgo_interface ${myAlgo_INTERFACE_SRC})

      target_link_libraries(myAlgo_interface ${myDepend_LIBRARIES} piperbase)

      install(TARGETS myAlgo_interface
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )

      install(DIRECTORY piper/myAlgo_interface/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
    )
    endif()
    ```

- Then, add all the source files to this directory: ```/piper/myAlgo_interface```

- Finally, to compile do the following in the catkin workspace where PIPER was installed

  ```bash
  catkin_make -DBUILD_myAlgo_INTERFACE:OPTION=ON
  ```

---
[Back to Documentation home](index.md)
