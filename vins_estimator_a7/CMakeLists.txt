cmake_minimum_required(VERSION 3.10)


set(TOP_DIR /home/changq/nanopi-m1-plus)
set(TOOLCHAIN_DIR ${TOP_DIR}/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf)
set(THIRDPARTY_DIR ${TOP_DIR}/3rdparty)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armhf)
set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/arm-none-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/arm-none-linux-gnueabihf-g++)

project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=armv7-a -pthread")

find_package(Eigen3)

include_directories(
${THIRDPARTY_DIR}/include
${THIRDPARTY_DIR}/include/opencv4
${EIGEN3_INCLUDE_DIR}
)

set(Boost_LIBS
${THIRDPARTY_DIR}/lib/libboost_atomic.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_chrono.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_container.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_context.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_contract.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_coroutine.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_date_time.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_exception.a
${THIRDPARTY_DIR}/lib/libboost_fiber.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_filesystem.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_graph.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_iostreams.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_locale.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_log_setup.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_log.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_c99f.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_c99l.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_c99.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_tr1f.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_tr1l.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_math_tr1.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_prg_exec_monitor.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_program_options.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_random.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_regex.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_serialization.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_stacktrace_basic.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_stacktrace_noop.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_system.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_test_exec_monitor.a
${THIRDPARTY_DIR}/lib/libboost_thread.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_timer.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_type_erasure.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_unit_test_framework.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_wave.so.1.71.0
${THIRDPARTY_DIR}/lib/libboost_wserialization.so.1.71.0
)


set(OpenCV_LIBS
${THIRDPARTY_DIR}/lib/libopencv_calib3d.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_core.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_dnn.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_features2d.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_flann.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_gapi.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_highgui.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_imgcodecs.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_imgproc.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_ml.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_objdetect.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_photo.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_stitching.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_videoio.so.4.2
${THIRDPARTY_DIR}/lib/libopencv_video.so.4.2
)

set(OpenCV_DEP_LIBS
${THIRDPARTY_DIR}/lib/libjpeg.so.8.2.2
${THIRDPARTY_DIR}/lib/libz.so.1.2.11
)

set(ATLAS
${THIRDPARTY_DIR}/lib/libblas.so.3.10.3
${THIRDPARTY_DIR}/lib/libatlas.so.3.10.3
${THIRDPARTY_DIR}/lib/libf77blas.so.3.10.3
${THIRDPARTY_DIR}/lib/liblapack.so.3.10.3
)

set(Suitesparse
${THIRDPARTY_DIR}/lib/libamd.so.2.4.6
${THIRDPARTY_DIR}/lib/libcamd.so.2.4.6
${THIRDPARTY_DIR}/lib/libccolamd.so.2.9.6
${THIRDPARTY_DIR}/lib/libcholmod.so.3.0.14
${THIRDPARTY_DIR}/lib/libcolamd.so.2.9.6
${THIRDPARTY_DIR}/lib/libmetis.so.5.1.0
${THIRDPARTY_DIR}/lib/libcxsparse.so.3.2.0
${THIRDPARTY_DIR}/lib/libspqr.so.2.0.9
${THIRDPARTY_DIR}/lib/libsuitesparseconfig.so.5.7.1
)

set(Ceres_LIBS
${THIRDPARTY_DIR}/lib/libceres.so.3
)

add_executable(vins_estimator
    src/estimator_node.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projection_factor.cpp
    src/factor/projection_td_factor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    
    src/feature_tracker.cpp
    src/camera_model/Camera.cc
    src/camera_model/CameraFactory.cc
    src/camera_model/EquidistantCamera.cc
    src/camera_model/PinholeCamera.cc
    )


target_link_libraries(vins_estimator ${OpenCV_LIBS} ${OpenCV_DEP_LIBS} ${Ceres_LIBS} ${Suitesparse} ${ATLAS}) 


