CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra
CPPFLAGS += $(shell pkg-config --cflags libavcodec libavformat libavutil libswscale)
LDLIBS += -lvencoder -lMemAdapter -lAWIspApi -lisp -lm $(shell pkg-config --libs libavcodec libavformat libavutil libswscale)

2pybot_hwstream: 2pybot_hwstream.c camera_output.h camera_control.h imx708_profile.h
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $< $(LDLIBS) -o $@.new
	mv -f $@.new $@

.PHONY: clean
clean:
	rm -f 2pybot_hwstream 2pybot_hwstream.new
