
set(PROJECT_NAME myrealm_core)

project(${PROJECT_NAME} LANGUAGES C CXX)

if(MSVC)
	find_package(SuiteSparse CONFIG)
else()
	find_package(SuiteSparse)
endif()

find_package(openvslam REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(fbow)

find_package(PCL 1.7 REQUIRED)
find_package(CGAL REQUIRED)
find_package(glog  CONFIG REQUIRED)


find_package(OpenSceneGraph 3.6.5 REQUIRED osgDB osgGA osgUtil osgText osgViewer osgWidget)

set(JoFmvTK_DIR "F:/OpenREALM2/jouav_media_kit/JoFmvTK-release-win64-gdgt-20220321-01")
set(JoFmvTK_INCLUDE_DIR "${JoFmvTK_DIR}/include")
set(JoFmvTK_LIB "${JoFmvTK_DIR}/lib64/jo_fmv_tk_api.lib")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HEAD_FILES
	MyREALM_Core_Exports.h
	NodeParas.h
	MyREALM.h
	GPSPoseInter.h
	grabber_exiv2_node.h
	JonFmvTKNode.h
	StageNode.h
	Scene3DNode.h
	FrameViewNode.h
	MyPublisher.h
	MySubscriber.h
	Conversions.h
	TrajSceneNode.h
	SparseSceneNode.h
	FacesSceneNode.h
	)
	
set(SRC_FILES
	NodeParas.cpp
	MyREALM.cpp
	GPSPoseInter.cpp
	grabber_exiv2_node.cpp
	JonFmvTKNode.cpp
	StageNode.cpp
	Scene3DNode.cpp
	FrameViewNode.cpp
	MyPublisher.cpp
	MySubscriber.cpp
	Conversions.cpp
	TrajSceneNode.cpp
	SparseSceneNode.cpp
	FacesSceneNode.cpp
	)

add_definitions(-DNOMINMAX -D_USE_MATH_DEFINES -DGLOG_NO_ABBREVIATED_SEVERITIES -DMyREALM_Core_DLL_EXPORT)

add_library(${PROJECT_NAME} SHARED ${HEAD_FILES} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
	PUBLIC
	"${CURRENT_SOURCE_DIRECTORY}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
	"${JoFmvTK_INCLUDE_DIR}"
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	realm_core
	realm_io
	realm_densifier_base
	realm_ortho
	realm_vslam_base
	realm_stages
	${OpenCV_LIBRARIES}
	${OPENSCENEGRAPH_LIBRARIES}
	glog::glog
	TJH_SRS
	${JoFmvTK_LIB}
 )
 
set_target_properties(${PROJECT_NAME}  
	PROPERTIES
	FOLDER my_realm)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
