TARGET = adns-connect

C_SRCS = main.c adns.c i2c.c socket-server.c

INLCUDES = -I.

#C_CFLAGS = -Wall -pedantic -O0 -std=c99
C_CFLAGS = -Wall
C_DFLAGS =
C_LDFLAGS =

C_EXT = c
C_OBJS = $(patsubst %.$(C_EXT), %.o, $(C_SRCS))

C = gcc

all: $(TARGET)

$(TARGET): $(CPP_OBJS) $(C_OBJS)
	$(C) $(C_CFLAGS) $(C_LDFLAGS) -o $(TARGET) $(C_OBJS)

$(C_OBJS): %.o: %.$(C_EXT)
	$(C) $(C_CFLAGS) $(C_DFLAGS) $(INCLUDES) -c $< -o $@ 

clean:
	$(RM) $(TARGET) $(C_OBJS)
