include (common)

add_executable (rtmath-rpline rpline.cpp)
set_target_properties( rtmath-rpline PROPERTIES FOLDER "Apps/Core")
target_link_libraries(rtmath-rpline ${rtmath_core-libs} rtmath_core )
INSTALL(TARGETS rtmath-rpline RUNTIME DESTINATION bin COMPONENT Applications)


