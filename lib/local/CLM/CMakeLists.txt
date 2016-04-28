include_directories(${BOOST_INCLUDE_DIR})

SET(SOURCE
    src/CCNF_patch_expert.cpp
	src/CLM.cpp
    src/CLM_utils.cpp
	src/CLMTracker.cpp
    src/DetectionValidator.cpp
	src/Patch_experts.cpp
	src/PAW.cpp
    src/PDM.cpp
	src/SVR_patch_expert.cpp
	src/stdafx.cpp
)

SET(HEADERS
    include/CCNF_patch_expert.h
	include/CLM.h
    include/CLM_utils.h
	include/CLMParameters.h
	include/CLMTracker.h
    include/DetectionValidator.h
	include/Patch_experts.h	
    include/PAW.h
	include/PDM.h
	include/SVR_patch_expert.h
	include/stdafx.h
)

include_directories(./include)
include_directories(${CLM_SOURCE_DIR}/include)

add_library( CLM ${SOURCE} ${HEADERS})

target_link_libraries(CLM opencv_calib3d opencv_objdetect)

## Mark library for installation
install(TARGETS CLM
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(FILES ${HEADERS} 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY model/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/model)
