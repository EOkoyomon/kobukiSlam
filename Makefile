CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard control_library/*.c)

explore: $(SRC)
	gcc -o $@ $@.c $^ $(CFLAGS) $(LIBS) -lm 

clean:
	rm explore
