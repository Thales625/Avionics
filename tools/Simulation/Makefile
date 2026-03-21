FIRMWARE_DIR := ../../firmware

libavionics.so: $(FIRMWARE_DIR)/flight_logic.c $(FIRMWARE_DIR)/flight_logic.h
	gcc -Wall -fPIC -DSIMULATION_BUILD -shared -o libavionics.so $(FIRMWARE_DIR)/flight_logic.c

bindings: $(FIRMWARE_DIR)/flight_logic.h libavionics.so
	ctypesgen -Dbool=int -DSIMULATION_BUILD -D__extension__= -l ./libavionics.so -o flight_logic_bindings.py $(FIRMWARE_DIR)/flight_logic.h

main: libavionics.so bindings
	python main.py
