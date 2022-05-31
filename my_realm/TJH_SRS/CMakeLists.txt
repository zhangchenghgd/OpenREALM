SET(ProjectName TJH_SRS)

PROJECT(${ProjectName})

find_package(GDAL REQUIRED)

# ==============================================================================
# Eigen
# ==============================================================================
# - external by default
# - external if EIGEN_INCLUDE_DIR_HINTS is defined
# - internal if Eigen not found
# ==============================================================================
find_package(Eigen3 CONFIG REQUIRED)
# if (NOT Eigen3_FOUND)
#   set(EIGEN3_INCLUDE_DIR TJH_3rdPartyLibDir/eigen/include)
# else()
#   set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# endif()

# All Source Files here
set(SourceFiles
	GDAL_Config.cpp
	SrsDef.cpp
	Coord.cpp
	ENU.cpp
	ECEF.cpp
	UTM.cpp
	Pose.cpp
	RTSTransform.cpp
	FourParaTransform.cpp
	BursaTransform.cpp
	SrsTransform.cpp)

# All non moc header files here
set(HeaderFiles
	${TJH_SRS_VERSION_HEADER}
	${TJH_SRS_CONFIG_HEADER}
	GDAL_Config.h
	SrsExport.h
	Coord.h
	SrsDef.h
	ENU.h
	ECEF.h
	UTM.h
	Pose.h
	TransformBase.h
	RTSTransform.h
	FourParaTransform.h
	BursaTransform.h
	SrsTransform.h)
    
add_definitions(-DTJH_SRS_LIB -D_USE_MATH_DEFINES)

if(TJH_SRS_USE_GDAL2)
	add_definitions(-DUSE_GDAL2)
endif(TJH_SRS_USE_GDAL2)

add_library(${ProjectName} SHARED ${HeaderFiles} ${SourceFiles})


target_include_directories(${ProjectName} PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../
	${CMAKE_BINARY_DIR}/include
	${GDAL_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIR})

target_link_libraries(${ProjectName}
	${GDAL_LIBRARIES})

 
set_target_properties(${ProjectName}  
    PROPERTIES
	FOLDER my_realm)

install(TARGETS ${ProjectName}
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib)

install(FILES ${HeaderFiles}  DESTINATION include/${ProjectName})

