INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER oracle.proto)
ADD_LIBRARY(proto ${PROTO_HEADER} ${PROTO_SRC})
# copy Protobuf variables so that the parent CMakeLists has these, too
# set(Protobuf_INCLUDE_DIRS ${Protobuf_INCLUDE_DIRS} PARENT_SCOPE)
# set(Protobuf_LIBRARIES ${Protobuf_LIBRARIES} PARENT_SCOPE)