FIRMWARE_DIR := ../../firmware
LIB_DIR := ../../lib

all: main

libavionics.so: $(FIRMWARE_DIR)/flight_logic.c $(FIRMWARE_DIR)/flight_logic.h
	gcc -Wall -fPIC -DSIMULATION_BUILD -shared -o libavionics.so -I$(LIB_DIR)/math_helper $(FIRMWARE_DIR)/flight_logic.c

bindings: $(FIRMWARE_DIR)/flight_logic.h libavionics.so
	ctypesgen -Dbool=int -DSIMULATION_BUILD -D__extension__= -l ./libavionics.so -o flight_logic_bindings.py -I$(LIB_DIR)/math_helper $(FIRMWARE_DIR)/flight_logic.h

main: libavionics.so bindings
	python main.py

.PHONY: clean
clean:
	rm -f libavionics.so flight_logic_bindings.py
	rm -rf __pycache__
