#ifndef _I2C_H
#define _I2C_H

#define PINSDA PINB0
#define PINSCL PINB2

#define PSDA PB0
#define PSCL PB2

#define DDSDA DDB0
#define DDSCL DDB2

void i2c_init();
unsigned char (*handle_read)(void);
void (*handle_write)(char v);

typedef unsigned char i2caddr_t;

typedef enum i2cstate {
        JUST_DEBUG = 0,
        IDLE = 1,
        SEQ_STARTED,
        SEQ_RESTARTED,
        ACKING_ADDRESS,
        REGISTER_SELECT,
        ACKING_REGISTER_SELECT,
        NACKING,
        READING_WRITE,
        ACKING_WRITE,
        ACKING_READ_REQUEST,
        WRITING,
        READING_ACK,
} i2cstate_t;

static i2caddr_t myaddr = 0x2a;
static volatile i2caddr_t selected_reg = 0;
static volatile char debug_note = 0;
static volatile i2cstate_t i2c = IDLE;

#endif /* _I2C_H */
