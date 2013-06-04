include (common)

add_executable (rtmath-getm getm.cpp)
set_target_properties( rtmath-getm PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-getm ${rtmath_ddscat-libs} rtmath_core rtmath_ddscat)
INSTALL(TARGETS rtmath-getm RUNTIME DESTINATION bin COMPONENT Applications)


