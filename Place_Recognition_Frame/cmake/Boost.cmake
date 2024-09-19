find_package(Boost REQUIRED QUIET)

include_directories(
	${Boost_INCLUDE_DIRS}
)

list(APPEND ALL_TARGET_LIBRARIES ${Boost_LIBRARIES})
