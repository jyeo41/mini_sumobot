#ifndef EDGE_DETECT_H_
#define EDGE_DETECT_H_
#include <stdint.h>

/* Enum to map all the possible sensor edge detecting combinations. Only accounts for 1 or 2 sensor combinations. */
typedef enum {
	EDGE_DETECT_LOCATION_NONE,
	EDGE_DETECT_LOCATION_FRONT_LEFT,
	EDGE_DETECT_LOCATION_FRONT_RIGHT,
	EDGE_DETECT_LOCATION_BACK_LEFT,
	EDGE_DETECT_LOCATION_BACK_RIGHT,
	EDGE_DETECT_LOCATION_FRONT,
	EDGE_DETECT_LOCATION_BACK,
	EDGE_DETECT_LOCATION_LEFT,
	EDGE_DETECT_LOCATION_RIGHT,
	EDGE_DETECT_LOCATION_DIAGONAL_LEFT,
	EDGE_DETECT_LOCATION_DIAGONAL_RIGHT,
}edge_detect_location_e;

void edge_detect_initialize(void);
edge_detect_location_e edge_detect_lookup(void);
void edge_detect_location_print(edge_detect_location_e edge);

#endif /* EDGE_DETECT_H_ */
