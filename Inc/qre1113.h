#ifndef QRE1113_H_
#define QRE1113_H_
#include <stdint.h>

typedef struct {
	uint16_t front_left;
	uint16_t front_right;
	uint16_t back_left;
	uint16_t back_right;
}qre1113_voltages_t;

void qre1113_initialize(void);
void qre1113_get_voltages(qre1113_voltages_t* voltages);
void qre1113_print_voltages(const qre1113_voltages_t* voltages);

#endif
