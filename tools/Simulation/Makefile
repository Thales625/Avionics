libavionics.so: avionics.c avionics.h
	gcc -Wall -fPIC -shared -o libavionics.so avionics.c

main: libavionics.so
	python main.py