# The slices library contains all of the compiled slice definitions
find_package(protobuf)

FILE( GLOB PROTO_FILES *.proto)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(protobuf)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS tmm.proto)

source_group("Protocol Buffer Definitions" FILES ${PROTO_FILES})

include(addlib)

include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROTOBUF_INCLUDE_DIR})
add_library(oneellip_proto SHARED ${PROTO_SRCS} ${PROTO_HDRS} ${PROTO_FILES} )
addlib(oneellip_proto SHARED)
storebin(oneellip_proto) # Put binaries in the usual directory
#set(slices-libs ${ICE_LIBRARIES})
#target_link_libraries(oneellip_slices ${slices-libs})

