set(protobuf_files configurationdata.proto discovery.proto health.proto healthinfo.proto networkinfo.proto nvramcontents.proto radarmodel.proto softwareversions.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${protobuf_files})

add_library(iasdk_protobuf STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(iasdk_protobuf ${PROTOBUF_LIBRARY})
