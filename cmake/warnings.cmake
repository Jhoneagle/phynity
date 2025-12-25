function(phynity_apply_warnings target)
	if(NOT PHYNITY_ENABLE_WARNINGS)
		return()
	endif()

	get_target_property(_type ${target} TYPE)
	if(_type STREQUAL "INTERFACE_LIBRARY")
		set(_scope INTERFACE)
	else()
		set(_scope PUBLIC)
	endif()

	if(MSVC)
		target_compile_options(${target} ${_scope}
			/W4
			/permissive-
			/Zc:__cplusplus
		)
		if(PHYNITY_WARNINGS_AS_ERRORS)
			target_compile_options(${target} ${_scope} /WX)
		endif()
	else()
		target_compile_options(${target} ${_scope}
			-Wall
			-Wextra
			-Wpedantic
			-Wconversion
			-Wsign-conversion
		)
		if(PHYNITY_WARNINGS_AS_ERRORS)
			target_compile_options(${target} ${_scope} -Werror)
		endif()
	endif()
endfunction()
