TARGET=hw7Damico

SOURCES=import_registers.c \
        enable_pwm_clock.c \
	wait_key.c \
        hw7Damico.c

OBJECTS=$(patsubst %.c,%.o,$(SOURCES))

all: $(OBJECTS)
	gcc $(OBJECTS) -lpthread -o $(TARGET) -lm

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -g -c $< -o $@ 
