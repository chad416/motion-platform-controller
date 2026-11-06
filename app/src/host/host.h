/*
 * Host interface over CAN. SPEC F09, F10.
 * Every 20 ms: answer queued CMD frames (CMD_ACK), then send FAULT (on a new
 * event), STATUS, POSE_T/R, LEG_POS_A/B, LEG_CUR_A/B; TIMING once a second.
 */
#ifndef MOTION_HOST_H_
#define MOTION_HOST_H_

int host_start(void);

#endif /* MOTION_HOST_H_ */
