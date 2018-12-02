CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard control_library/*.c)
CFLAGS += "-I/usr/include/python2.7"

explore: $(SRC)
	gcc -o $@ $@.c $^  $(LIBS) -lm -lpython2.7 $(CFLAGS)
clean:
	rm explore
