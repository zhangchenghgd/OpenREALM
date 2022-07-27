
set(PROJECT_NAME myrealm_scene_test)

project(${PROJECT_NAME} LANGUAGES C CXX)

find_package(Qt5 COMPONENTS Core)

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
	PRIVATE
	"${CURRENT_SOURCE_DIRECTORY}"
)

target_link_libraries(${PROJECT_NAME} 
	PRIVATE
	Qt5::Core
	myrealm_scene
)

set_target_properties(${PROJECT_NAME}  
	PROPERTIES
	FOLDER my_realm)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
