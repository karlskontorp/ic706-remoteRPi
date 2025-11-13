#
# Makefile for IC-706 on Raspberry Pi 
#
CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=gnu99
LDFLAGS =  # Kan legges til senere
LDLIBS = -lm -lpthread

# === Definer pakker F�RST ===
PKG_LIBS = portaudio-2.0 opus libgpiod

# === Legg til cflags og libs ETTER definisjon ===
CFLAGS += $(shell pkg-config --cflags $(PKG_LIBS) 2>/dev/null || echo "")
LDLIBS += $(shell pkg-config --libs $(PKG_LIBS) 2>/dev/null || echo "-lportaudio -lopus -lgpiod")

# === Ekstra linker-flags ===
LDLIBS += -lgpiod -lpthread

# === Targets ===
IS_MAIN = ic706_server
IC_MAIN = ic706_client
AS_MAIN = audio_server
AC_MAIN = audio_client

IS_OBJS = ic706_server.o common.o
IC_OBJS = ic706_client.o common.o
AS_OBJS = audio_server.o audio_util.o common.o
AC_OBJS = audio_client.o audio_util.o common.o

all: $(IS_MAIN) $(IC_MAIN) $(AS_MAIN) $(AC_MAIN)

# === VIKTIG: Bruk $(LDFLAGS) og $(LDLIBS) i linker-trinnet! ===
$(IS_MAIN): $(IS_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(IC_MAIN): $(IC_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(AS_MAIN): $(AS_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(AC_MAIN): $(AC_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	$(RM) *.o $(IS_MAIN) $(IC_MAIN) $(AS_MAIN) $(AC_MAIN)

.PHONY: all clean