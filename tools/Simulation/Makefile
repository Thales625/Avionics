FIRMWARE_DIR := ../../firmware
LIB_DIR := ../../lib
FLIGHT_LOGIC_DIR := $(LIB_DIR)/flight_logic

all: main

libavionics.so: $(FLIGHT_LOGIC_DIR)/flight_logic.c $(FLIGHT_LOGIC_DIR)/flight_logic.h
	gcc -Wall -fPIC -DSIMULATION_BUILD -shared -o libavionics.so -I$(LIB_DIR)/math_helper $(FLIGHT_LOGIC_DIR)/flight_logic.c

bindings: $(FLIGHT_LOGIC_DIR)/flight_logic.h libavionics.so
	ctypesgen -Dbool=int -DSIMULATION_BUILD -D__extension__= -l ./libavionics.so -o flight_logic_bindings.py -I$(LIB_DIR)/math_helper $(FLIGHT_LOGIC_DIR)/flight_logic.h

main: libavionics.so bindings
	python main.py

.PHONY: clean
clean:
	rm -f libavionics.so flight_logic_bindings.py
	rm -rf __pycache__
