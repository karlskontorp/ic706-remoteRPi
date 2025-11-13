/*
 * Copyright (c) 2014, Alexandru Csete
 * All rights reserved.
 *
 * This software is licensed under the terms and conditions of the
 * Simplified BSD License. See license.txt for details.
 *
 */
#include <gpiod.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>               /* O_WRONLY */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
// ************ RPI ENDRING START ************
#include <time.h>               // Trengs for usleep i riktig implementasjon
// ************ RPI ENDRING SLUTT ************

#include "common.h"

struct gpiod_line_request *gpiod_request = NULL;

// Definisjon av chip-navnet
// RPi 5 har vanligvis chip 0. RPi 4 kan ha chip 0 eller 1.
#define GPIO_CHIP_NAME "gpiochip0" 
// (Hvis dette feiler, bytter vi til gpiochip1)

/* Print an array of chars as HEX numbers */

/* Configure serial interface to raw mode with specified attributes */
int set_serial_config(int fd, int speed, int parity, int blocking)
{
    struct termios  tty;

    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;       // disable break processing
    tty.c_lflag = 0;             // no signaling chars, no echo,
    // no canonical processing

    /* no remapping, no delays */
    tty.c_oflag = 0;

    /* 0.5 sec read timeout */
    tty.c_cc[VMIN] = blocking ? 1 : 0;
    tty.c_cc[VTIME] = 5;

    /* shut off xon/xoff ctrl */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    /* ignore modem controls and enable reading */
    tty.c_cflag |= (CLOCAL | CREAD);

    /* parity */
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcsetattr", errno);
        return -1;
    }

    return 0;
}

int create_server_socket(int port)
{
    struct sockaddr_in serv_addr;
    int               sock_fd = -1;
    int               yes = 1;

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd == -1)
    {
        fprintf(stderr, "Error creating socket: %d: %s\n", errno,
                strerror(errno));

        return -1;
    }

    if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
        fprintf(stderr, "Error setting SO_REUSEADDR: %d: %s\n", errno,
                strerror(errno));

    /* bind socket to host address */
    memset(&serv_addr, 0, sizeof(struct sockaddr_in));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
    if (bind(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
    {
        fprintf(stderr, "bind() error: %d: %s\n", errno, strerror(errno));

        close(sock_fd);
        return -1;
    }

    if (listen(sock_fd, 1) == -1)
    {
        fprintf(stderr, "listen() error: %d: %s\n", errno, strerror(errno));

        close(sock_fd);
        return -1;
    }

    return sock_fd;
}

int read_data(int fd, struct xfr_buf *buffer)
{
    uint8_t         *buf = buffer->data;
    int              type = PKT_TYPE_INCOMPLETE;
    size_t           num;

    /* read data */
    num = read(fd, &buf[buffer->wridx], RDBUF_SIZE - buffer->wridx);

    if (num > 0)
    {
        buffer->wridx += num;

        /* There is at least one character in the buffer.
         *
         * If buf[0] = 0xFE then this is a regular packet. Check if
         * buf[end] = 0xFD, if yes, the packet is complete and return
         * the packet type.
         *
         * If buf[0] = 0x00 and wridx = 1 then this is an EOS packet.
         * If buf[0] = 0x00 and wridx > 1 then this is an invalid
         * packet (does not start with 0xFE).
         */
        if (buf[0] == 0xFE)
        {
            if (buf[buffer->wridx - 1] == 0xFD)
                type = buf[1];
            else
                type = PKT_TYPE_INCOMPLETE;
        }
        else if ((buf[0] == 0x00) && (buffer->wridx == 1))
        {
            type = PKT_TYPE_EOS;
        }
        else
        {
            type = PKT_TYPE_INVALID;
        }
    }
    else if (num == 0)
    {
        type = PKT_TYPE_EOF;
        fprintf(stderr, "Received EOF from FD %d\n", fd);
    }
    else
    {
        type = PKT_TYPE_INVALID;
        fprintf(stderr, "Error reading from FD %d: %d: %s\n", fd, errno,
                strerror(errno));
    }

    return type;
}

int transfer_data(int ifd, int ofd, struct xfr_buf *buffer)
{
    uint8_t           init1_resp[] = { 0xFE, 0xF0, 0xFD };
    uint8_t           init2_resp[] = { 0xFE, 0xF1, 0xFD };
    int               pkt_type;

    pkt_type = read_data(ifd, buffer);
    switch (pkt_type)
    {
    case PKT_TYPE_KEEPALIVE:
        /* emulated on server side; do not forward */
        buffer->wridx = 0;
        buffer->valid_pkts++;
        break;
    case PKT_TYPE_INIT1:
        /* Sent by the first unit that is powered on.
            Expects PKT_TYPE_INIT1 + PKT_TYPE_INIT2 in response. */
        buffer->write_errors += write(ifd, init1_resp, 3) != 3;
        buffer->write_errors += write(ifd, init2_resp, 3) != 3;
        buffer->wridx = 0;
        buffer->valid_pkts++;
        break;

    case PKT_TYPE_INIT2:
        /* Sent by the panel when powered on and the radio is already on.
            Expects PKT_TYPE_INIT2 in response. */
        buffer->write_errors += write(ifd, init2_resp, 3);
        buffer->wridx = 0;
        buffer->valid_pkts++;
        break;

    case PKT_TYPE_PWK:
        /* Power on/off message sent by panel; leave handling to server */
#if DEBUG
        print_buffer(ifd, ofd, buffer->data, buffer->wridx);
#endif
        buffer->wridx = 0;
        buffer->valid_pkts++;
        break;

    case PKT_TYPE_INCOMPLETE:
        break;

    case PKT_TYPE_INVALID:
        buffer->invalid_pkts++;
        buffer->wridx = 0;
        break;

    default:
        /* we also "send" on EOF packet because buffer may not be empty */
#if DEBUG
        print_buffer(ifd, ofd, buffer->data, buffer->wridx);
#endif
        buffer->write_errors +=
             write(ofd, buffer->data, buffer->wridx) != buffer->wridx;

        buffer->wridx = 0;
        buffer->valid_pkts++;
    }

    return pkt_type;
}

uint64_t time_ms(void)
{
    struct timeval tval;
    gettimeofday(&tval, NULL);
    return (uint64_t)tval.tv_sec * 1000 + tval.tv_usec / 1000;
}

uint64_t time_us(void)
{
    struct timeval  tval;

    gettimeofday(&tval, NULL);

    return 1e6 * tval.tv_sec + tval.tv_usec;
}

int send_keepalive(int fd)
{
    char            msg[] = { 0xFE, 0x0B, 0x00, 0xFD };

    return (write(fd, msg, 4) != 4);
}

void send_pwr_message(int fd, int poweron)
{
    char            msg[] = { 0xFE, 0xA0, 0x00, 0xFD };

    if (poweron)
        msg[2] = 0x01;

    if (write(fd, msg, 4) == -1)
        fprintf(stderr, "Error sending PWR message %d (%s)\n", errno,
                strerror(errno));
}

// ************ RPI ENDRING START: PWK_INIT for UTGANG (GPIO_PWK=17) ************
// Original koden var satt opp for √• lytte p√• GPIO7 (INNGANG).
// Din bruk er √• sende PWK som en UTGANG. Derfor erstatter vi logikken
// med en enkel kaller til gpio_init_out(GPIO_PWK).
#include <gpiod.h>  // Behold dette p√• linje 9

// ************ RPI ENDRING START: PWK_INIT for UTGANG (GPIO_PWK=4) ************
int pwk_init(void)
{
    // Kall gpio_init_out for  sette opp som output
    if (gpio_init_out(GPIO_PWK) < 0) {
        return -1;
    }
    return 0;
}
// ************ RPI ENDRING SLUTT ************

// ************ RPI ENDRING START: gpio_init_out ************
// Setter opp GPIO som output (v2 API √• returnerer 0 ved suksess, -1 ved feil)
int gpio_init_out(unsigned int gpio)
{
    struct gpiod_chip *chip;
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *line_cfg;
    struct gpiod_request_config *req_cfg;
    unsigned int offsets[1] = { gpio };

    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) return -1;

    settings = gpiod_line_settings_new();
    if (!settings) { gpiod_chip_close(chip); return -1; }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings) < 0) {
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }

    req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }
    gpiod_request_config_set_consumer(req_cfg, "ic706_gpio");

    gpiod_request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!gpiod_request) {
        fprintf(stderr, "Kunne ikke request GPIO %u\n", gpio);
        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return -1;
    }

    // Ikke frigj¯r her! Hold request Âpen
    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    gpiod_chip_close(chip);  // Chip kan lukkes, request lever
    return 0;
}
// ************ RPI ENDRING SLUTT ************

// ************ RPI ENDRING START: gpio_set_value ************
// Setter verdi (v2 API ñ returnerer 0 ved suksess, -1 ved feil)
int gpio_set_value(unsigned int gpio, unsigned int value)
{
    if (!gpiod_request) return -1;
    return gpiod_line_request_set_value(gpiod_request, gpio,
           value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}// ************ RPI ENDRING SLUTT ************
// ************ RPI ENDRING SLUTT ************
