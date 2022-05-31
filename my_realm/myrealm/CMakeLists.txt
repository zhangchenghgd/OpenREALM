
set(PROJECT_NAME myrealm)

project(${PROJECT_NAME} LANGUAGES C CXX)

find_package(Qt5 COMPONENTS Core Gui Widgets OpenGL)
find_library(OSGQOPENGL_LIBRARY osgQOpenGL)
find_library(OSGQOPENGL_LIBRARY_DEBUG osgQOpenGLd)

find_package(OpenCV CONFIG REQUIRED core imgproc imgcodecs)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#查找设置当前文件夹中所有的头文件
FILE(GLOB_RECURSE  HEAD_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.h")
#查找当前文件夹中的所有源代码文件，也可以通过Set命令将所有文件设置为一个变量
FILE(GLOB_RECURSE  SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

FILE(GLOB_RECURSE  UI_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.ui")

FILE(GLOB_RECURSE  QRC_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.qrc")

FILE(GLOB_RECURSE  RC_FILES ${CMAKE_CURRENT_SOURCE_DIR} "*.rc")


source_group("Source Files" FILES ${SRC_FILES})

source_group("Header Files" FILES ${HEAD_FILES})

source_group("Ui Files" FILES ${UI_FILES})

source_group("QResource Files" FILES ${QRC_FILES})

source_group("Resource Files" FILES ${RC_FILES})

add_executable(${PROJECT_NAME} ${HEAD_FILES} ${SRC_FILES} ${UI_FILES} ${QRC_FILES} ${RC_FILES})

target_include_directories(${PROJECT_NAME} 
	PUBLIC
	"${CURRENT_SOURCE_DIRECTORY}"
	"${OPENSCENEGRAPH_INCLUDE_DIR}"
	"${OpenCV_INCLUDE_DIRS}"
)

target_link_libraries(${PROJECT_NAME} 
	PRIVATE
	myrealm_core
	myrealm_viewer
	Qt5::Core
	Qt5::Gui
	Qt5::Widgets
	Qt5::OpenGL
	debug  ${OSGQOPENGL_LIBRARY_DEBUG}
	optimized ${OSGQOPENGL_LIBRARY}
	${OpenCV_LIBS}
)

set_target_properties(${PROJECT_NAME}  
	PROPERTIES
	FOLDER my_realm)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:\"mainCRTStartup\"")

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
