CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard ../control_library/*.c)

main: $(SRC)
	gcc -o $@ $@.c $^ $(CFLAGS) $(LIBS) -lm 

drive: $(SRC)
	gcc -o $@ $@.c $^ $(CFLAGS) $(LIBS) -lm 

turn: $(SRC)
	gcc -o $@ $@.c $^ $(CFLAGS) $(LIBS) -lm 

ser:
	gcc -o $@ c_ser_test.c -lm

clean:
	rm main drive turn ser
