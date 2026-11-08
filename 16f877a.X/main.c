#include "main.h"

#define MAX_CRUISE_SPEED 3000U

static void send_motor_command(SPI_Commands_t command, int16_t value) {
    SPI_PACKET->cmd = (unsigned char)command;
    SPI_PACKET->val = value;
    spi_transfer_packet();
}

static void display_speed(uint16_t cruise_speed) {
    uint16_t rpm = (cruise_speed * 3U) / 10U;

    hmi_print_number((int16_t)rpm, 0, 4);
    disp_buf[4] = ' ';
    disp_buf[5] = 'R';
    disp_buf[6] = 'P';
    disp_buf[7] = 'M';
}

static void display_direction(uint8_t motor_direction) {
    hmi_print_string(motor_direction == DIR_CCW ? "BACKWARD" : " FORWARD");
}

void main(void) {
    uint16_t cruise_speed = 0;
    uint8_t motor_direction = DIR_CW;

    system_init();
    send_motor_command(CMD_SET_MOTOR_SPEED, (int16_t)cruise_speed);
    send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
    display_speed(cruise_speed);

    while (1) {
        analog_update();
        unsigned char key = citeste_tastatura();

        switch (key) {
            case '0':
                cruise_speed = (cruise_speed >= 500) ? cruise_speed - 500 : 0;
                break;
            case '1':
                cruise_speed = (cruise_speed >= 50) ? cruise_speed - 50 : 0;
                break;
            case '2':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - 50)
                    ? cruise_speed + 50
                    : MAX_CRUISE_SPEED;
                break;
            case '3':
                cruise_speed = (cruise_speed <= MAX_CRUISE_SPEED - 500)
                    ? cruise_speed + 500
                    : MAX_CRUISE_SPEED;
                break;
            case '4':
                motor_direction = DIR_CCW;
                send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
                display_direction(motor_direction);
                continue;
            case '5':
                motor_direction = DIR_CW;
                send_motor_command(CMD_SET_MOTOR_DIR, (int16_t)motor_direction);
                display_direction(motor_direction);
                continue;
            default:
                continue;
        }

        send_motor_command(CMD_SET_MOTOR_SPEED, (int16_t)cruise_speed);
        display_speed(cruise_speed);
    }
}
