include (common)

add_executable (rtmath-combo combo2.cpp)
set_target_properties( rtmath-combo PROPERTIES FOLDER "Apps")
target_link_libraries(rtmath-combo ${rtmath_core-libs} rtmath_core )
INSTALL(TARGETS rtmath-combo RUNTIME DESTINATION bin COMPONENT Applications)


