
#ifndef __SIMCARD_DEF_H__
#define __SIMCARD_DEF_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <sys/types.h>
#endif

#define SIMCARD_MAX_DATA_LENGTH 512

enum {
    SIMCARD_CONTAINER_TYPE_UNKNOWN  = 0,
	SIMCARD_CONTAINER_TYPE_DATA     = 1,
	SIMCARD_CONTAINER_TYPE_RESET    = 2,
	SIMCARD_CONTAINER_TYPE_SPEED    = 3,
};

struct simcard_data {
	struct simcard_data_header {
		u_int32_t type;
		u_int32_t length;
	} __attribute__((packed)) header;
	union {
		u_int8_t data[SIMCARD_MAX_DATA_LENGTH];
		u_int32_t reset;
		u_int32_t speed;
	} __attribute__((packed)) body;
} __attribute__((packed));

#endif //__SIMCARD_DEF_H__
