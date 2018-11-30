#include <stdio.h>
#include "../control_library/kobuki_library.h"

int main(void) {
	if (kobukiLibraryInit()) {
		printf("We good\n.");
	}
	return 0;
}
