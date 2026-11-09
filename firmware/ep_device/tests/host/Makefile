# Host test of the RHS2116 stream/stim scheduling and task-level API: make -C tests/host
# The driver comes from the rhs2116 module; stim.c from this app.
DRV := ../../../modules/rhs2116

CFLAGS += -std=c11 -g -O1 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
          -fsanitize=address,undefined -Izephyr_stub -I$(DRV)/include -I$(DRV)/src -I../../src

SRCS := ../../src/stim.c $(DRV)/src/rhs2116_ctrl.c
HDRS := $(wildcard ../../src/*.h $(DRV)/src/*.h $(DRV)/include/rhs2116/*.h)

test_stream: test_stream.c $(DRV)/src/rhs2116_stream.c $(SRCS) $(HDRS) zephyr_stub/shim.h
	$(CC) $(CFLAGS) -o $@ test_stream.c $(SRCS)

.PHONY: run clean
run: test_stream
	./test_stream

clean:
	rm -f test_stream
