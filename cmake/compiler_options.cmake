function(phynity_target_defaults target)
	# Apply consistent language level propagation.
	get_target_property(_type ${target} TYPE)
	if(_type STREQUAL "INTERFACE_LIBRARY")
		set(_scope INTERFACE)
	else()
		set(_scope PUBLIC)
	endif()

	target_compile_features(${target} ${_scope} cxx_std_20)

	# Link-time optimization is skipped for interface libraries because IPO
	# properties are not supported on them.
	if(PHYNITY_ENABLE_LTO AND NOT _type STREQUAL "INTERFACE_LIBRARY")
		set_property(TARGET ${target} PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
	endif()
endfunction()
