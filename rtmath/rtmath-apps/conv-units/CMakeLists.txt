include (common)

add_executable (rtmath-conv-units conv.cpp)
set_target_properties( rtmath-conv-units PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-conv-units ${rtmath_core-libs} rtmath_core)
INSTALL(TARGETS rtmath-conv-units RUNTIME DESTINATION bin COMPONENT Applications)


