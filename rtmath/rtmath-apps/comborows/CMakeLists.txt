include (common)

add_executable (rtmath-comborows comborows.cpp)
set_target_properties( rtmath-comborows PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-comborows ${rtmath_core-libs} rtmath_core )
INSTALL(TARGETS rtmath-comborows RUNTIME DESTINATION bin COMPONENT Applications)


