
set(PROJECT_NAME myrealm_scene)

project(${PROJECT_NAME} LANGUAGES C CXX)

find_package(GDAL REQUIRED)

find_package(OpenCV 4 REQUIRED)

find_package(OpenSceneGraph 3.6.5 REQUIRED osgDB osgGA osgUtil osgText osgTerrain osgViewer osgWidget)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#查找设置当前文件夹中所有的头文件
FILE(GLOB_RECURSE  HEAD_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.h")
#查找当前文件夹中的所有源代码文件，也可以通过Set命令将所有文件设置为一个变量
FILE(GLOB_RECURSE  SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

FILE(GLOB_RECURSE  RC_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.rc")

source_group("Source Files" FILES ${SRC_FILES})

source_group("Header Files" FILES ${HEAD_FILES})

source_group("Resource Files" FILES ${RC_FILES})

add_definitions(-DNOMINMAX -D_USE_MATH_DEFINES -DGLOG_NO_ABBREVIATED_SEVERITIES -DMyREALM_Scene_DLL_EXPORT)

add_library(${PROJECT_NAME} SHARED ${HEAD_FILES} ${SRC_FILES} ${RC_FILES})

target_include_directories(${PROJECT_NAME} 
	PUBLIC
	"${CURRENT_SOURCE_DIRECTORY}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
	${GDAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
	PUBLIC
	realm_io
	${OPENSCENEGRAPH_LIBRARIES}
	${GDAL_LIBRARIES}
	${OpenCV_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}  
	PROPERTIES
	FOLDER my_realm)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
