add_executable(fpv_minimal minimal.c)
target_link_libraries(fpv_minimal PRIVATE libfpv::fpv)
