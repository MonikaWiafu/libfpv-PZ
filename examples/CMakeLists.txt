add_executable(fpv_minimal minimal.c)
target_link_libraries(fpv_minimal PRIVATE libfpv::fpv)

# The standalone "game": its own SDL3 (video + audio) + OpenGL 3.3.
if(TARGET SDL3::SDL3-static)
  add_executable(fpv_standalone standalone/main.cpp)
  target_include_directories(fpv_standalone PRIVATE standalone ${PROJECT_SOURCE_DIR}/docs/renderers)
  target_link_libraries(fpv_standalone PRIVATE libfpv::fpv SDL3::SDL3-static)
  if(WIN32)
    target_link_options(fpv_standalone PRIVATE -static-libgcc -static-libstdc++ -static)
    set_target_properties(fpv_standalone PROPERTIES WIN32_EXECUTABLE OFF)
  endif()
endif()
