#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

typedef enum {
	IR_RECEIVER_CMD_TURN_ON,
	IR_RECEIVER_CMD_TURN_OFF,
}ir_receiver_cmd_e;

void ir_receiver_initialize(void);
ir_receiver_cmd_e ir_receiver_get_cmd(void);

#endif /* IR_RECEIVER_H_ */
