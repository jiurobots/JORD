find_package(Eigen3 REQUIRED)

include_directories(
	${EIGEN3_INCLUDE_DIRS}
)

list(APPEND ALL_TARGET_LIBRARIES ${Sophus_LIBRARIES})