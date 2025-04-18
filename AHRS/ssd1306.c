/*
 * Copyright (c) 2015, Vladimir Komendantskiy
 * MIT License
 *
 * SSD1306 is a 128x64 dot matrix display driver and controller by Solomon
 * Systech. It is used by HelTec display modules.
 *
 * Reference:
 *
 * [1] SSD1306 Advance Information. 128x64 Dot Matrix OLED/PLED Segment/Common
 *     Driver with Controller. (Solomon Systech)
 */

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include "stdio.h"
#include "font.h"

// real-time features
#include <sys/mman.h>
#include <sched.h>

#include "ssd1306.h"

#define SSD1306_PACKET_SIZE 32

int ssd1306_close(struct display_info* disp)
{
	if (close(disp->file) < 0)
		return -1;

	return 0;
}

void cleanup(int status, void* disp)
{
	ssd1306_close((struct display_info*)disp);
}

int ssd1306_open(struct display_info* disp, char* filename)
{
	disp->file = open(filename, O_RDWR);
	if (disp->file < 0)
		return -1;

	if (ioctl(disp->file, I2C_SLAVE, disp->address) < 0)
		return -2;

	on_exit(cleanup, (void*)disp);

	return 0;
}

int ssd1306_send(struct display_info* disp, struct sized_array* payload)
{
	if (write(disp->file, payload->array, payload->size) != payload->size)
		return -1;

	return 0;
}

int ssd1306_init(struct display_info* disp)
{
	struct sched_param sch;
	int status = 0;
	struct sized_array payload;

	sch.sched_priority = 49;

	status = sched_setscheduler(0, SCHED_FIFO, &sch);
	if (status < 0)
		return status;

	status = mlockall(MCL_CURRENT | MCL_FUTURE);
	if (status < 0)
		return status;

	payload.size = sizeof(display_config);
	payload.array = display_config;

	status = ssd1306_send(disp, &payload);
	if (status < 0)
		return status;

	memset(disp->buffer, 0, sizeof(disp->buffer));

	return 0;
}

int ssd1306_send_buffer(struct display_info* disp)
{
	int offset = 0, status = 0;
	struct sized_array payload;

	for (offset = 0; offset < 1024; offset += SSD1306_PACKET_SIZE) {
		unsigned char packet[SSD1306_PACKET_SIZE + 1];

		packet[0] = SSD1306_CTRL_BYTE_DATA_STREAM;
		memcpy(packet + 1, disp->buffer + offset, SSD1306_PACKET_SIZE);

		payload.size = SSD1306_PACKET_SIZE + 1;
		payload.array = packet;

		status = ssd1306_send(disp, &payload);
		if (status < 0)
			return -1;
	}

	return 0;
}

void _swap_int16_t(int16_t *a, int16_t *b)
{
	int16_t t = *a;
	*a = *b;
	*b = t;
}

void writePixel(struct display_info* disp, int16_t x, int16_t y) {
  if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
	return;
  }
  disp->buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y & 7));
}

void ssd1306_writeLine(struct display_info* disp , int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(&x0, &y0);
    _swap_int16_t(&x1, &y1);
  }

  if (x0 > x1) {
    _swap_int16_t(&x0, &x1);
    _swap_int16_t(&y0, &y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      writePixel(disp, y0, x0);
    } else {
      writePixel(disp, x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void sd1306_write_char(struct display_info *disp, char c, int x, int y) {
	int i, j;
	int offset = 0;
	int width = 11;
	int height = 18;
	uint32_t b = 0;
	const uint8_t *bitmap = NULL;

	if (c < 0x20 || c > 0x7F) {
		return; // Invalid character
	}

	offset = c - 0x20; // Offset for ASCII characters
	
    for(i = 0; i < height; i++) {
        b = Font11x18[(c - 32) * height + i];
        for(j = 0; j < width; j++) {
            if((b << j) & 0x8000)  {
                writePixel(disp, (x + j), (y + i));
            }
        }
    }
}

void sd1306_write_string(struct display_info *disp, const char *str, int x, int y) {
	while (*str) {
		sd1306_write_char(disp, *str, x, y);
		x += 11; // Move to the next character position
		str++;
	}
}
