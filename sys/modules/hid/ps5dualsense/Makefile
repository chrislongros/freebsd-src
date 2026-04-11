.PATH: ${SRCTOP}/sys/dev/hid

KMOD=	ps5dualsense
SRCS=	ps5dualsense.c
SRCS+=	opt_hid.h opt_kbd.h
SRCS+=	bus_if.h device_if.h usbdevs.h

.include <bsd.kmod.mk>
