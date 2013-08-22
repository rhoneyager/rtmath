include (common)

add_executable (rtmath-getm getm.cpp)
set_target_properties( rtmath-getm PROPERTIES FOLDER "Apps/Analysis")
target_link_libraries(rtmath-getm ${rtmath_ddscat_base-libs} rtmath_core rtmath_ddscat_base)
INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin COMPONENT Applications)


