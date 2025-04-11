CC=gcc

all:
	$(CROSS_COMPILE)$(CC) -o AHRS AHRS.c madgwickFilter.c ssd1306.c -lm
clean:
	rm AHRS

.PHONY: all clean
