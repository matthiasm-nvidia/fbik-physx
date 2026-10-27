cmake_minimum_required(VERSION 3.20)
project(fbik_physx VERSION 0.1.0 LANGUAGES C CXX)

# The pinned native SDK loaders use the release CRT in both configurations.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT WIN32)
    message(FATAL_ERROR "This viewer uses the Windows x64 native SDK packages.")
endif()
foreach(sdk ovstage ovrtx ovphysx)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/deps/${sdk}/lib/cmake/${sdk}/${sdk}Config.cmake")
        message(FATAL_ERROR "Missing ${sdk}. Run setup_deps.bat before building.")
    endif()
    find_package(${sdk} CONFIG REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/deps/${sdk}/lib/cmake/${sdk}" NO_DEFAULT_PATH)
endforeach()

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/glfw)
find_package(OpenGL REQUIRED)

add_library(viewer_core STATIC
    src/viewer/Camera.cpp
    src/viewer/StageCamera.cpp
    src/viewer/StageData.cpp
    src/viewer/PhysicsSimulation.cpp
    src/viewer/RtxView.cpp
    src/viewer/RecentFiles.cpp)
target_include_directories(viewer_core PUBLIC src/viewer)
target_link_libraries(viewer_core PUBLIC ovstage::ovstage ovphysx::ovphysx ovrtx::ovrtx_static glfw OpenGL::GL)

add_library(viewer_ui STATIC
    src/viewer/viewer_imgui_theme.cpp
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_impl_glfw.cpp)
target_include_directories(viewer_ui PUBLIC third_party/imgui src/viewer)
target_link_libraries(viewer_ui PUBLIC glfw OpenGL::GL)

add_executable(fbik_viewer src/viewer/main.cpp src/viewer/FileDialog.cpp)
target_link_libraries(fbik_viewer PRIVATE viewer_core viewer_ui shell32 ole32)
add_custom_command(TARGET fbik_viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/resources/fonts"
        "$<TARGET_FILE_DIR:fbik_viewer>/resources/fonts"
    COMMENT "Copy viewer fonts")

include(CTest)
if(BUILD_TESTING)
    add_executable(recent_files_tests tests/recent_files_tests.cpp src/viewer/RecentFiles.cpp)
    target_include_directories(recent_files_tests PRIVATE src/viewer)
    add_test(NAME recent_files COMMAND recent_files_tests "${CMAKE_CURRENT_BINARY_DIR}/recent-tests-$<CONFIG>")
    add_test(NAME viewer_help COMMAND fbik_viewer --help)
    add_test(NAME viewer_invalid_argument COMMAND fbik_viewer --unknown)
    set_tests_properties(viewer_invalid_argument PROPERTIES WILL_FAIL TRUE)
    option(VIEWER_TEST_RTX "Build and register the GPU viewer smoke test" OFF)
    if(VIEWER_TEST_RTX)
        add_executable(viewer_smoke tests/viewer_smoke.cpp)
        target_link_libraries(viewer_smoke PRIVATE viewer_core viewer_ui)
        add_test(NAME viewer_rtx COMMAND viewer_smoke "${CMAKE_CURRENT_SOURCE_DIR}")
        set_tests_properties(viewer_rtx PROPERTIES TIMEOUT 600 LABELS gpu RUN_SERIAL TRUE)
    endif()
endif()

# Deploy once per configuration; parallel executable links must not race on DLLs.
file(GLOB_RECURSE runtime_files CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/bin/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/plugins/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/schemas/*"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/scenes/*")
set(runtime_dir "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
add_custom_command(OUTPUT "${runtime_dir}/physics-runtime.stamp"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${runtime_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/bin" "${runtime_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/plugins" "${runtime_dir}/plugins"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/schemas" "${runtime_dir}/schemas"
    # PhysX resolves these beside ovphysx.dll (the SDK's original bin/../plugins layout changes here).
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/plugins/PhysXGpu_64.dll" "${runtime_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/deps/ovphysx/plugins/PhysXDevice64.dll" "${runtime_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:ovstage::ovstage>" "${runtime_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/resources/scenes" "${runtime_dir}/resources/scenes"
    COMMAND ${CMAKE_COMMAND} -E touch "${runtime_dir}/physics-runtime.stamp"
    DEPENDS ${runtime_files} "$<TARGET_FILE:ovstage::ovstage>"
    VERBATIM)
add_custom_target(viewer_runtime DEPENDS "${runtime_dir}/physics-runtime.stamp")
add_dependencies(fbik_viewer viewer_runtime)
if(TARGET viewer_smoke)
    add_dependencies(viewer_smoke viewer_runtime)
endif()
if(BUILD_TESTING)
    add_executable(physics_tests tests/physics_tests.cpp)
    target_link_libraries(physics_tests PRIVATE viewer_core)
    add_dependencies(physics_tests viewer_runtime)
    add_test(NAME physics COMMAND physics_tests "${CMAKE_CURRENT_SOURCE_DIR}")
    set_tests_properties(physics PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
endif()
