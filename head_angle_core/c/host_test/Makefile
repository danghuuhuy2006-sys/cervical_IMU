# Lõi được biên dịch với cờ nghiêm ngặt (giống component ESP-IDF, thêm -Werror);
# chương trình phát lại chỉ dùng cờ thường vì printf thăng float lên double.
CORE   = ../components/head_angle_core
STRICT = -std=c99 -O2 -ffp-contract=off -Wall -Wextra -Wdouble-promotion -Werror -I$(CORE)/include
SRCS   = $(wildcard $(CORE)/src/*.c)
OBJS   = $(patsubst $(CORE)/src/%.c,build/%.o,$(SRCS))

replay: replay.c $(OBJS)
	gcc -std=c99 -O2 -ffp-contract=off -Wall -Wextra -I$(CORE)/include -o $@ replay.c $(OBJS) -lm

build/%.o: $(CORE)/src/%.c | build
	gcc $(STRICT) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -rf build replay test_core

test_core: test_core.c $(OBJS)
	gcc -std=c99 -O2 -ffp-contract=off -Wall -Wextra -I$(CORE)/include -o $@ test_core.c $(OBJS) -lm

test: test_core
	./test_core
