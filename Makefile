PREFIX ?= /usr/local

CC = gcc
AR = ar

CFLAGS = -std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function
CFLAGS += -Isrc -Ofast -fno-strict-overflow

APRILTAG_SRCS := $(shell ls src/*.c src/common/*.c)
APRILTAG_HEADERS := $(shell ls src/*.h src/common/*.h)
APRILTAG_OBJS := $(APRILTAG_SRCS:%.c=%.o)
TARGETS := lib/libapriltag.a lib/libapriltag.so

.PHONY: all
all: $(TARGETS)
	@$(MAKE) -C example all

.PHONY: install
install: lib/libapriltag.so
	@chmod +x src/install.sh
	@./src/install.sh $(PREFIX)/lib lib/libapriltag.so
	@./src/install.sh $(PREFIX)/include/apriltag $(APRILTAG_HEADERS)
	@sed 's:^prefix=$$:prefix=$(PREFIX):' < src/apriltag.pc.in > src/apriltag.pc
	@./src/install.sh $(PREFIX)/lib/pkgconfig src/apriltag.pc
	@rm src/apriltag.pc
	@ldconfig

lib/libapriltag.a: $(APRILTAG_OBJS)
	@mkdir -p lib
	@echo "   [$@]"
	@$(AR) -cq $@ $(APRILTAG_OBJS)

lib/libapriltag.so: $(APRILTAG_OBJS)
	@mkdir -p lib
	@echo "   [$@]"
	@$(CC) -fPIC -shared -o $@ $^

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o common/*.o $(TARGETS)
	@$(MAKE) -C example clean
