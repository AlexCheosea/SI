#include "main.h"

static void send_motor_command(SPI_Commands_t command, int16_t value) {
    SPI_PACKET->cmd = (unsigned char)command;
    SPI_PACKET->val = value;
    spi_transfer_packet();
}

static void display_speed(uint16_t cruise_speed) {
    hmi_print_number((int16_t)cruise_speed, 0, DISP_BUF_SIZE);
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
        unsigned char key = citeste_tastatura();

        switch (key) {
            case '0':
                cruise_speed = (cruise_speed >= 100) ? cruise_speed - 100 : 0;
                break;
            case '1':
                cruise_speed = (cruise_speed >= 10) ? cruise_speed - 10 : 0;
                break;
            case '2':
                cruise_speed += 10;
                break;
            case '3':
                cruise_speed += 100;
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
