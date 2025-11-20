PROBE_DEV = /dev/ttyACM0

CROSS = arm-none-eabi-
CC = $(CROSS)gcc
GDB = $(CROSS)gdb

LINKER_SCRIPT = nano33sense_rev2.ld
MEMORY_MAP = nano33sense_rev2.map

C_FLAGS = -mcpu=cortex-m4 -mthumb -O0 -g -Wall -Wextra -std=gnu11 
LD_FLAGS = -nostdlib -T $(LINKER_SCRIPT) -Wl,-Map=$(MEMORY_MAP)
GDB_FLAGS = -ex "target extended-remote $(PROBE_DEV)"
RUN_FLAGS = -ex "target extended-remote $(PROBE_DEV)" -ex "monitor swdp_scan" -ex "att 1" -ex "load" -ex "run"

SRC = main.c  nano33sense_rev2_startup.c
OBJ = $(SRC:.c=.o)
TARGET = nano33sense_rev2.elf

$(TARGET): $(OBJ)
	$(CC) $(LD_FLAGS) -o $@ $^

$(OBJ): %.o: %.c
	$(CC) $(C_FLAGS) -c $< -o $@

debug:
	$(GDB) $(GDB_FLAGS) $(TARGET)

run: $(TARGET)
	$(GDB) $(RUN_FLAGS) $(TARGET)

clean:
	rm -f $(OBJ) $(TARGET) $(MEMORY_MAP)
