add_library(jnissd_pedestrian_tracker SHARED jni_ssd_tracker.cpp)

set (CMAKE_C_CFLAGS "${CMAKE_C_CFLAGS} -fPIC")
set (CMAKE_CXX_CFLAGS "${CMAKE_CXX_CFLAGS} -fPIC")

target_link_libraries(jnissd_pedestrian_tracker ssd_pedestrian_tracker)

include_directories(jnibasic_pedestrian_tracker PUBLIC ${PROJECT_SOURCE_DIR}/include)
include_directories(jnibasic_pedestrian_tracker PUBLIC ${JAVA_HOME}/include)

if (MSVC)
    include_directories(jnissd_pedestrian_tracker PUBLIC ${JAVA_HOME}/include/windows)
else ()
    include_directories(jnissd_pedestrian_tracker PUBLIC ${JAVA_HOME}/include/linux)
endif ()

install(TARGETS jnissd_pedestrian_tracker DESTINATION lib)
