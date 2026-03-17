FIRMWARE_DIR := ../../firmware

libavionics.so: $(FIRMWARE_DIR)/flight_logic.c $(FIRMWARE_DIR)/flight_logic.h
	gcc -Wall -fPIC -DSIMULATION_BUILD -shared -o libavionics.so $(FIRMWARE_DIR)/flight_logic.c

main: libavionics.so
	python main.py
