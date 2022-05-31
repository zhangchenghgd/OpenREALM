
set(PROJECT_NAME myrealm_viewer)

project(${PROJECT_NAME} LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HEAD_FILES
    MyViewerExports.h
	MyViewer.h
	)
	
set(SRC_FILES
   MyViewer.cpp
   )

add_definitions(-DNOMINMAX -D_USE_MATH_DEFINES -DMYREALM_VIRWER_DLL_EXPORT)

add_library(${PROJECT_NAME} SHARED ${HEAD_FILES} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
    PUBLIC
    "${CURRENT_SOURCE_DIRECTORY}"
)

# target_link_libraries(${PROJECT_NAME} 
#     PUBLIC 
# )

 
set_target_properties(${PROJECT_NAME}  
    PROPERTIES
	FOLDER my_realm)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
