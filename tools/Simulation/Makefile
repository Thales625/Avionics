LIB_DIR := ../../lib
FLIGHT_LOGIC_DIR := $(LIB_DIR)/flight_logic
BINDINGS_DIR := bindings

$(BINDINGS_DIR)/libavionics.so: $(FLIGHT_LOGIC_DIR)/flight_logic.c $(FLIGHT_LOGIC_DIR)/flight_logic.h
	gcc -Wall -fPIC -DSIMULATION_BUILD -shared -o $(BINDINGS_DIR)/libavionics.so -I$(LIB_DIR)/math_helper $(FLIGHT_LOGIC_DIR)/flight_logic.c

bindings: $(FLIGHT_LOGIC_DIR)/flight_logic.h $(BINDINGS_DIR)/libavionics.so
	ctypesgen -Dbool=int -DSIMULATION_BUILD -D__extension__= -l $(BINDINGS_DIR)/libavionics.so -o $(BINDINGS_DIR)/flight_logic_bindings.py -I$(LIB_DIR)/math_helper $(FLIGHT_LOGIC_DIR)/flight_logic.h

# ENTRIES
main: $(BINDINGS_DIR)/libavionics.so bindings
	python -m main

clean:
	rm -f $(BINDINGS_DIR)/libavionics.so $(BINDINGS_DIR)/flight_logic_bindings.py

.PHONY: clean