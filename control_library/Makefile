CC=gcc

# Should be equivalent to your list of C files, if you don't build selectively
SRC=$(wildcard *.c)

main: $(filter-out drive.c turn.c, $(SRC))
	gcc -o $@ $^ $(CFLAGS) $(LIBS) -lm 

drive: $(filter-out main.c turn.c, $(SRC))
	gcc -o $@ $^ $(CFLAGS) $(LIBS) -lm 

turn: $(filter-out main.c drive.c , $(SRC))
	gcc -o $@ $^ $(CFLAGS) $(LIBS) -lm 

clean:
	rm main drive turn
