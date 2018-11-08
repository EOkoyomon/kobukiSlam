CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard *.c)

main: $(SRC)
	gcc -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm main
