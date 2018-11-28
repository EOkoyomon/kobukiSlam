CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard *.c)

main: $(filter-out drive.c , $(SRC))
	gcc -lm -o $@ $^ $(CFLAGS) $(LIBS)

drive: $(filter-out main.c , $(SRC))
	gcc -lm -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm main drive
