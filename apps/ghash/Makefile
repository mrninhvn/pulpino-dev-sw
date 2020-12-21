all: test

test32: ghash.h ghash_ctmul32.c main.c
	gcc -std=c99 -DBR_MUL32 ghash_ctmul32.c main.c -o $@
test: ghash.h ghash_ctmul.c main.c
	gcc -std=c99 ghash_ctmul.c main.c -o $@
test64: ghash.h ghash_ctmul64.c main.c
	gcc -std=c99 -DBR_MUL64 ghash_ctmul64.c main.c -o $@
.PHONY: clean
clean:
	-rm -f test test32 test64
