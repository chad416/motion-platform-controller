/* CAN transport: TX queue + thread, RX filters, bus state. */
#ifndef MOTION_CANLINK_H_
#define MOTION_CANLINK_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/can.h>

struct canlink_stats {
	uint32_t tx_ok;
	uint32_t tx_errors;
	uint32_t tx_overflows; /* frames dropped because the TX queue was full */
	uint32_t bus_off_count;
	uint32_t rx_vehicle;   /* VEHICLE_F / VEHICLE_W frames */
	uint32_t rx_cmds;
	uint32_t rx_dropped;   /* CMD frames dropped because the queue was full */
	enum can_state state;
	bool up;
};

/* FDCAN1, classic CAN 500 kbit/s: filters for VEHICLE_F/W and CMD, TX thread. */
int canlink_init(void);

/* Queue a frame. Never blocks. 0; -EINVAL if len > 8; -ENOBUFS (counted) if the queue is full. */
int canlink_send(uint16_t id, const uint8_t *data, uint8_t len);

/* CMD frames (struct can_frame) for the host thread. */
struct k_msgq *canlink_cmd_queue(void);

void canlink_get_stats(struct canlink_stats *out);

#endif /* MOTION_CANLINK_H_ */
