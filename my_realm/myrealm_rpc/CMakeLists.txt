
set(PROJECT_NAME myrealm_rpc)

project(${PROJECT_NAME})

find_package(ZLIB REQUIRED)

find_package(OpenSSL REQUIRED)

find_package(Protobuf CONFIG REQUIRED)

find_package(srpc CONFIG REQUIRED)

find_library(LZ4_LIBRARY NAMES lz4)

find_package(Snappy CONFIG REQUIRED)

set(SRPC_LIBS 
	"${VCPKG_ROOT_DIR}/installed/x64-windows/lib/srpc.lib"
	"${VCPKG_ROOT_DIR}/installed/x64-windows/lib/workflow.lib"
	"${VCPKG_ROOT_DIR}/installed/x64-windows/lib/snappy.lib"
)

# 生成SRPC文件
function(srpc_generate_protobuf header_files proto_file)
	set(srpc_gen "${VCPKG_INSTALLED_DIR}/x64-windows/tools/srpc/srpc_generator.exe")
	execute_process(
		COMMAND  ${srpc_gen} protobuf ${proto_file} ${CMAKE_CURRENT_BINARY_DIR}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

	get_filename_component(fl_name ${proto_file} NAME_WE)

	set(SRPC_H "${CMAKE_CURRENT_BINARY_DIR}/${fl_name}.srpc.h")
	if(EXISTS ${SRPC_H})
		set(${header_files} ${SRPC_H} PARENT_SCOPE)
	endif()
endfunction()

FILE(GLOB_RECURSE  HDR_FILES  ${CMAKE_CURRENT_SOURCE_DIR} "*.h")

FILE(GLOB_RECURSE  SRC_FILES  ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

FILE(GLOB_RECURSE  PROTO_FILES  ${CMAKE_CURRENT_SOURCE_DIR}/proto "*.proto")

set(PROTO_SRCS "")
set(PROTO_HDRS "")

foreach(PROT_FILE ${PROTO_FILES})
	protobuf_generate_cpp(EXP_PROTO_SRCS EXP_PROTO_HDRS ${PROT_FILE})
	srpc_generate_protobuf(EXP_RPC_HDRS ${PROT_FILE})

	list(APPEND PROTO_SRCS ${EXP_PROTO_SRCS})
	list(APPEND PROTO_HDRS ${EXP_PROTO_HDRS} ${EXP_RPC_HDRS})

endforeach()


source_group("Source Files" FILES ${PROTO_SRCS} ${SRC_FILES})

source_group("Header Files" FILES ${PROTO_HDRS} ${HDR_FILES})

source_group("Proto Files" FILES ${PROTO_FILES})

add_library(${PROJECT_NAME} ${PROTO_SRCS} ${PROTO_HDRS} ${PROTO_FILES} ${HDR_FILES} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} 
	PUBLIC
	${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME} 
	PUBLIC
	protobuf::libprotobuf-lite
	protobuf::libprotobuf
	protobuf::libprotoc
	${SRPC_LIBS}
	ZLIB::ZLIB
	Snappy::snappy
	${LZ4_LIBRARY}
	OpenSSL::SSL 
	OpenSSL::Crypto
	ws2_32
	wsock32
	realm_core
)

set_target_properties(${PROJECT_NAME}  
	PROPERTIES
	FOLDER my_realm)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib)
	
# install(FILES ${HEAD_FILES} DESTINATION include/${PROJECT_NAME})
