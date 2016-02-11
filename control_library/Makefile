CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard *.c)

main: $(filter-out drive.c , $(SRC))
	gcc -o $@ $^ $(CFLAGS) $(LIBS) -lm 

drive: $(filter-out main.c , $(SRC))
	gcc -o $@ $^ $(CFLAGS) $(LIBS) -lm 

clean:
	rm main drive
