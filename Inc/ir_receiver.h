#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

/* Enum mapping of all the buttons on the IR remote.
 * Manually map the 8 bits of the command byte for each unique button press.
 */
typedef enum {
	IR_RECEIVER_CMD_0 = 0x30,
	IR_RECEIVER_CMD_1 = 0x08,
	IR_RECEIVER_CMD_2 = 0x88,
	IR_RECEIVER_CMD_3 = 0x48,
	IR_RECEIVER_CMD_4 = 0x28,
	IR_RECEIVER_CMD_5 = 0xA8,
	IR_RECEIVER_CMD_6 = 0x68,
	IR_RECEIVER_CMD_7 = 0x18,
	IR_RECEIVER_CMD_8 = 0x98,
	IR_RECEIVER_CMD_9 = 0x58,
	IR_RECEIVER_CMD_UP = 0xA0,
	IR_RECEIVER_CMD_DOWN = 0xB0,
	IR_RECEIVER_CMD_LEFT = 0x10,
	IR_RECEIVER_CMD_RIGHT = 0x50,
	IR_RECEIVER_CMD_VOL_DECREASE = 0x00,
	IR_RECEIVER_CMD_VOL_INCREASE = 0x40,
	IR_RECEIVER_CMD_PLAY_PAUSE = 0x80,
	IR_RECEIVER_CMD_SETUP = 0x20,
	IR_RECEIVER_CMD_STOP_OR_MODE = 0x60,
	IR_RECEIVER_CMD_ENTER_OR_SAVE = 0x90,
	IR_RECEIVER_CMD_UNDO = 0x70,
	IR_RECEIVER_CMD_NONE = 0xFF,
}ir_receiver_cmd_e;

void ir_receiver_initialize(void);
const char* ir_receiver_get_cmd(void);

#endif /* IR_RECEIVER_H_ */
