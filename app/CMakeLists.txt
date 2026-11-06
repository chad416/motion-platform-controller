cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(motion_fw LANGUAGES C)

set(MCUE ${CMAKE_CURRENT_SOURCE_DIR}/../mcue)

target_include_directories(app PRIVATE
	${MCUE}/include
	src/hal
	src/motion
	src/proto
	src/can
	src/host
	src/sys
)

target_sources(app PRIVATE
	src/main.c
	src/hal/act_sim.c
	src/motion/motion.c
	src/proto/proto.c
	src/can/canlink.c
	src/host/host.c
	src/sys/wdt.c
	src/shell/shell_cmds.c
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
)
