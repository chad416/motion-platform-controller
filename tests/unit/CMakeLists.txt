cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mcue_unit)

set(MCUE ${CMAKE_CURRENT_SOURCE_DIR}/../../mcue)
set(APP ${CMAKE_CURRENT_SOURCE_DIR}/../../app/src)

target_include_directories(app PRIVATE src ${MCUE}/include ${APP}/proto)

target_sources(app PRIVATE
	src/test_linalg.c
	src/test_kin.c
	src/test_workspace.c
	src/test_filters.c
	src/test_washout.c
	src/test_shaping.c
	src/test_servo.c
	src/test_ctl.c
	src/test_proto.c
	${MCUE}/src/linalg.c
	${MCUE}/src/geometry.c
	${MCUE}/src/kin.c
	${MCUE}/src/workspace.c
	${MCUE}/src/filters.c
	${MCUE}/src/washout.c
	${MCUE}/src/shaping.c
	${MCUE}/src/plant.c
	${MCUE}/src/servo.c
	${MCUE}/src/ctl.c
	${APP}/proto/proto.c
)
