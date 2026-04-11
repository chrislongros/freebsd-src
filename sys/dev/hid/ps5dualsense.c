/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Vladimir Kondratyev <wulf@FreeBSD.org>
 * Copyright (c) 2026 Christos Longros <chris.longros@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Sony PS5 DualSense controller driver
 * https://controllers.fandom.com/wiki/Sony_DualSense
 * https://github.com/torvalds/linux/blob/master/drivers/hid/hid-playstation.c
 *
 * Supports lightbar RGB LED and player indicator LEDs over USB.
 * Based on the PS4 DualShock 4 driver (ps4dshock.c).
 */

#include "opt_hid.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/stat.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>

#define	HID_DEBUG_VAR	ps5ds_debug
#include <dev/hid/hgame.h>
#include <dev/hid/hid.h>
#include <dev/hid/hidbus.h>
#include <dev/hid/hidquirk.h>
#include <dev/hid/hidmap.h>
#include "usbdevs.h"

#ifdef HID_DEBUG
static int ps5ds_debug = 1;

static SYSCTL_NODE(_hw_hid, OID_AUTO, ps5dualsense, CTLFLAG_RW, 0,
		"Sony PS5 DualSense Gamepad");
SYSCTL_INT(_hw_hid_ps5dualsense, OID_AUTO, debug, CTLFLAG_RWTUN,
		&ps5ds_debug, 0, "Debug level");
#endif

/*
 * Fixed HID report descriptor for the DualSense gamepad.
 * The hardware descriptor uses vendor-specific usage pages (Microsoft 0xff00)
 * which the generic hgame driver cannot parse.  We replace it with a standard
 * Game Pad descriptor so the evdev layer sees proper axis and button mappings.
 */
static const uint8_t	ps5ds_rdesc[] = {
	0x05, 0x01,		/* Usage Page (Generic Desktop Ctrls)	*/
	0x09, 0x05,		/* Usage (Game Pad)			*/
	0xA1, 0x01,		/* Collection (Application)		*/
	0x85, 0x01,		/*   Report ID (1)			*/
	0x09, 0x30,		/*   Usage (X)  - left stick X		*/
	0x09, 0x31,		/*   Usage (Y)  - left stick Y		*/
	0x09, 0x33,		/*   Usage (Rx) - right stick X		*/
	0x09, 0x34,		/*   Usage (Ry) - right stick Y		*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x26, 0xFF, 0x00,	/*   Logical Maximum (255)		*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x95, 0x04,		/*   Report Count (4)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x09, 0x32,		/*   Usage (Z)  - L2 trigger		*/
	0x09, 0x35,		/*   Usage (Rz) - R2 trigger		*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x26, 0xFF, 0x00,	/*   Logical Maximum (255)		*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x95, 0x02,		/*   Report Count (2)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x06, 0x00, 0xFF,	/*   Usage Page (Vendor Defined 0xFF00)	*/
	0x09, 0x20,		/*   Usage (0x20)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x26, 0xFF, 0x00,	/*   Logical Maximum (255)		*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs) - counter	*/
	0x05, 0x01,		/*   Usage Page (Generic Desktop)	*/
	0x09, 0x39,		/*   Usage (Hat switch)			*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x07,		/*   Logical Maximum (7)		*/
	0x35, 0x00,		/*   Physical Minimum (0)		*/
	0x46, 0x3B, 0x01,	/*   Physical Maximum (315)		*/
	0x65, 0x14,		/*   Unit (Eng Rot: Degree)		*/
	0x75, 0x04,		/*   Report Size (4)			*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x81, 0x42,		/*   Input (Data,Var,Abs,Null State)	*/
	0x65, 0x00,		/*   Unit (None)			*/
	0x45, 0x00,		/*   Physical Maximum (0)		*/
	0x05, 0x09,		/*   Usage Page (Button)		*/
	0x19, 0x01,		/*   Usage Minimum (0x01)		*/
	0x29, 0x0F,		/*   Usage Maximum (0x0F)		*/
	0x15, 0x00,		/*   Logical Minimum (0)		*/
	0x25, 0x01,		/*   Logical Maximum (1)		*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x95, 0x0F,		/*   Report Count (15)			*/
	0x81, 0x02,		/*   Input (Data,Var,Abs)		*/
	0x75, 0x01,		/*   Report Size (1)			*/
	0x95, 0x01,		/*   Report Count (1)			*/
	0x81, 0x01,		/*   Input (Const) - padding		*/
	/* Output report (64 bytes total) for LED/rumble control */
	0x06, 0x00, 0xFF,	/*   Usage Page (Vendor Defined)	*/
	0x09, 0x01,		/*   Usage (0x01)			*/
	0x85, 0x02,		/*   Report ID (2)			*/
	0x75, 0x08,		/*   Report Size (8)			*/
	0x95, 0x3F,		/*   Report Count (63)			*/
	0x91, 0x02,		/*   Output (Data,Var,Abs)		*/
	0xC0,			/* End Collection			*/
};

/*
 * DualSense USB output report structure.
 * Report ID 0x02, 64 bytes total (1 byte report ID + 63 bytes data).
 *
 * Byte offsets are relative to the start of the data (after report ID):
 *   [0]     valid_flag0
 *   [1]     valid_flag1 - controls which features are updated
 *   [2]     motor_right - rumble
 *   [3]     motor_left  - rumble
 *   [38]    valid_flag2 - lightbar setup control
 *   [41]    lightbar_setup
 *   [42]    led_brightness
 *   [43]    player_leds - 5 indicator LEDs (bits 0-4)
 *   [44]    lightbar_red
 *   [45]    lightbar_green
 *   [46]    lightbar_blue
 */
#define	PS5DS_OUTPUT_REPORT_USB_SIZE	64

/* valid_flag0 bits (required for USB output reports) */
#define	PS5DS_FLAG0_COMPATIBLE_VIBRATION	0x01
#define	PS5DS_FLAG0_HAPTICS_SELECT		0x02

/* valid_flag1 bits */
#define	PS5DS_FLAG1_LIGHTBAR	0x04
#define	PS5DS_FLAG1_PLAYER_LEDS	0x10

/* valid_flag2 bits */
#define	PS5DS_FLAG2_LIGHTBAR_SETUP	0x02

/* lightbar_setup values */
#define	PS5DS_LIGHTBAR_SETUP_LIGHT_ON	0x01
#define	PS5DS_LIGHTBAR_SETUP_LIGHT_OUT	0x02

/* Byte offsets within output report data (after report ID byte) */
#define	PS5DS_OUT_VALID_FLAG0		0
#define	PS5DS_OUT_VALID_FLAG1		1
#define	PS5DS_OUT_VALID_FLAG2		38
#define	PS5DS_OUT_LIGHTBAR_SETUP	41
#define	PS5DS_OUT_LED_BRIGHTNESS	42
#define	PS5DS_OUT_PLAYER_LEDS		43
#define	PS5DS_OUT_LIGHTBAR_R		44
#define	PS5DS_OUT_LIGHTBAR_G		45
#define	PS5DS_OUT_LIGHTBAR_B		46

/* Player LED patterns (from Linux hid-playstation.c) */
static const uint8_t ps5ds_player_led_patterns[] = {
	0x04,	/* Player 1: --x-- */
	0x0A,	/* Player 2: -x-x- */
	0x15,	/* Player 3: x-x-x */
	0x1B,	/* Player 4: xx-xx */
	0x1F,	/* Player 5: xxxxx */
};

static const struct ps5ds_led {
	int	r;
	int	g;
	int	b;
} ps5ds_leds[] = {
	{ 0x00, 0x00, 0x40 },	/* Blue   */
	{ 0x40, 0x00, 0x00 },	/* Red    */
	{ 0x00, 0x40, 0x00 },	/* Green  */
	{ 0x20, 0x00, 0x20 },	/* Pink   */
	{ 0x02, 0x01, 0x00 },	/* Orange */
	{ 0x00, 0x01, 0x01 },	/* Teal   */
	{ 0x01, 0x01, 0x01 }	/* White  */
};

enum ps5ds_led_state {
	PS5DS_LED_OFF,
	PS5DS_LED_ON,
	PS5DS_LED_CNT,
};

struct ps5ds_softc {
	struct hidmap	hm;

	struct sx	lock;
	enum ps5ds_led_state	led_state;
	struct ps5ds_led	led_color;
	int		player_leds;	/* 0-4: player number, -1: off */
};

#define	PS5DS_OFFSET(field) offsetof(struct ps5ds_softc, field)
enum {
	PS5DS_SYSCTL_LED_STATE =	PS5DS_OFFSET(led_state),
	PS5DS_SYSCTL_LED_COLOR_R =	PS5DS_OFFSET(led_color.r),
	PS5DS_SYSCTL_LED_COLOR_G =	PS5DS_OFFSET(led_color.g),
	PS5DS_SYSCTL_LED_COLOR_B =	PS5DS_OFFSET(led_color.b),
	PS5DS_SYSCTL_PLAYER_LEDS =	PS5DS_OFFSET(player_leds),
#define	PS5DS_SYSCTL_LAST		PS5DS_SYSCTL_PLAYER_LEDS
};

#define PS5DS_MAP_BTN(number, code)		\
	{ HIDMAP_KEY(HUP_BUTTON, number, code) }
#define PS5DS_MAP_ABS(usage, code)		\
	{ HIDMAP_ABS(HUP_GENERIC_DESKTOP, HUG_##usage, code) }
#define PS5DS_MAP_FLT(usage, code)		\
	{ HIDMAP_ABS(HUP_GENERIC_DESKTOP, HUG_##usage, code), .flat = 15 }
#define	PS5DS_MAP_GCB(usage, callback)		\
	{ HIDMAP_ANY_CB(HUP_GENERIC_DESKTOP, HUG_##usage, callback) }

static hidmap_cb_t	ps5ds_final_cb;

static const struct hidmap_item ps5ds_map[] = {
	PS5DS_MAP_FLT(X,		ABS_X),
	PS5DS_MAP_FLT(Y,		ABS_Y),
	PS5DS_MAP_ABS(Z,		ABS_Z),
	PS5DS_MAP_ABS(RZ,		ABS_RZ),
	PS5DS_MAP_FLT(RX,		ABS_RX),
	PS5DS_MAP_FLT(RY,		ABS_RY),
	PS5DS_MAP_GCB(HAT_SWITCH,	hgame_hat_switch_cb),
	PS5DS_MAP_BTN(1,		BTN_WEST),
	PS5DS_MAP_BTN(2,		BTN_SOUTH),
	PS5DS_MAP_BTN(3,		BTN_EAST),
	PS5DS_MAP_BTN(4,		BTN_NORTH),
	PS5DS_MAP_BTN(5,		BTN_TL),
	PS5DS_MAP_BTN(6,		BTN_TR),
	PS5DS_MAP_BTN(7,		BTN_TL2),
	PS5DS_MAP_BTN(8,		BTN_TR2),
	PS5DS_MAP_BTN(9,		BTN_SELECT),
	PS5DS_MAP_BTN(10,		BTN_START),
	PS5DS_MAP_BTN(11,		BTN_THUMBL),
	PS5DS_MAP_BTN(12,		BTN_THUMBR),
	PS5DS_MAP_BTN(13,		BTN_MODE),
	/* Click button is handled by touchpad driver */
	/* PS5DS_MAP_BTN(14,		BTN_LEFT), */
	PS5DS_MAP_BTN(15,		KEY_MUTE),
	{ HIDMAP_FINAL_CB(				ps5ds_final_cb) },
};

/* DualSense product ID: 0x0ce6 */
static const struct hid_device_id ps5ds_devs[] = {
	{ HID_BVP(BUS_USB, USB_VENDOR_SONY, 0x0ce6),
	  HID_TLC(HUP_GENERIC_DESKTOP, HUG_GAME_PAD) },
};

static int
ps5ds_final_cb(HIDMAP_CB_ARGS)
{
	struct evdev_dev *evdev = HIDMAP_CB_GET_EVDEV();

	if (HIDMAP_CB_GET_STATE() == HIDMAP_CB_IS_ATTACHING) {
		evdev_support_prop(evdev, INPUT_PROP_DIRECT);
		evdev_set_cdev_mode(evdev, UID_ROOT, GID_GAMES,
		    S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	}

	/* Do not execute callback at interrupt handler and detach */
	return (ENOSYS);
}

static int
ps5ds_write(struct ps5ds_softc *sc)
{
	uint8_t buf[PS5DS_OUTPUT_REPORT_USB_SIZE];
	bool led_on;
	uint8_t player_pattern;

	memset(buf, 0, sizeof(buf));
	buf[0] = 0x02;	/* report ID */

	led_on = sc->led_state != PS5DS_LED_OFF;

	/* USB output reports require valid_flag0 or the controller ignores them */
	buf[1 + PS5DS_OUT_VALID_FLAG0] =
	    PS5DS_FLAG0_COMPATIBLE_VIBRATION | PS5DS_FLAG0_HAPTICS_SELECT;

	/* Enable lightbar and player LED updates */
	buf[1 + PS5DS_OUT_VALID_FLAG1] =
	    PS5DS_FLAG1_LIGHTBAR | PS5DS_FLAG1_PLAYER_LEDS;

	/* Enable lightbar hardware */
	buf[1 + PS5DS_OUT_VALID_FLAG2] = PS5DS_FLAG2_LIGHTBAR_SETUP;
	buf[1 + PS5DS_OUT_LIGHTBAR_SETUP] =
	    PS5DS_LIGHTBAR_SETUP_LIGHT_ON | PS5DS_LIGHTBAR_SETUP_LIGHT_OUT;
	buf[1 + PS5DS_OUT_LED_BRIGHTNESS] = 0x02;	/* max brightness */

	/* Player LEDs */
	if (sc->player_leds >= 0 &&
	    sc->player_leds < (int)nitems(ps5ds_player_led_patterns))
		player_pattern = ps5ds_player_led_patterns[sc->player_leds];
	else
		player_pattern = 0;
	buf[1 + PS5DS_OUT_PLAYER_LEDS] = player_pattern;

	/* Lightbar RGB */
	buf[1 + PS5DS_OUT_LIGHTBAR_R] = led_on ? sc->led_color.r : 0;
	buf[1 + PS5DS_OUT_LIGHTBAR_G] = led_on ? sc->led_color.g : 0;
	buf[1 + PS5DS_OUT_LIGHTBAR_B] = led_on ? sc->led_color.b : 0;

	return (hid_write(sc->hm.dev, buf, sizeof(buf)));
}

static int
ps5ds_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ps5ds_softc *sc;
	int error, arg;

	if (oidp->oid_arg1 == NULL || oidp->oid_arg2 < 0 ||
	    oidp->oid_arg2 > PS5DS_SYSCTL_LAST)
		return (EINVAL);

	sc = oidp->oid_arg1;
	sx_xlock(&sc->lock);

	/* Read the current value. */
	arg = *(int *)((char *)sc + oidp->oid_arg2);
	error = sysctl_handle_int(oidp, &arg, 0, req);

	/* Sanity check. */
	if (error || !req->newptr)
		goto unlock;

	switch (oidp->oid_arg2) {
	case PS5DS_SYSCTL_LED_STATE:
		if (arg < 0 || arg >= PS5DS_LED_CNT)
			error = EINVAL;
		break;
	case PS5DS_SYSCTL_LED_COLOR_R:
	case PS5DS_SYSCTL_LED_COLOR_G:
	case PS5DS_SYSCTL_LED_COLOR_B:
		if (arg < 0 || arg > UINT8_MAX)
			error = EINVAL;
		break;
	case PS5DS_SYSCTL_PLAYER_LEDS:
		if (arg < -1 || arg > 4)
			error = EINVAL;
		break;
	default:
		error = EINVAL;
	}

	/* Update. */
	if (error == 0) {
		*(int *)((char *)sc + oidp->oid_arg2) = arg;
		ps5ds_write(sc);
	}
unlock:
	sx_unlock(&sc->lock);

	return (error);
}

static void
ps5ds_identify(driver_t *driver, device_t parent)
{

	/* Overload PS5 DualSense gamepad rudimentary report descriptor */
	if (HIDBUS_LOOKUP_ID(parent, ps5ds_devs) != NULL)
		hid_set_report_descr(parent, ps5ds_rdesc,
		    sizeof(ps5ds_rdesc));
}

static int
ps5ds_probe(device_t dev)
{
	struct ps5ds_softc *sc = device_get_softc(dev);

	hidmap_set_debug_var(&sc->hm, &HID_DEBUG_VAR);
	return (
	    HIDMAP_PROBE(&sc->hm, dev, ps5ds_devs, ps5ds_map, NULL)
	);
}

static int
ps5ds_attach(device_t dev)
{
	struct ps5ds_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(dev);
	int error, unit;

	sx_init(&sc->lock, "ps5dualsense");

	unit = device_get_unit(dev);
	sc->led_state = PS5DS_LED_ON;
	sc->led_color = ps5ds_leds[unit % nitems(ps5ds_leds)];
	sc->player_leds = unit < (int)nitems(ps5ds_player_led_patterns)
	    ? unit : -1;

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "led_state", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY, sc,
	    PS5DS_SYSCTL_LED_STATE, ps5ds_sysctl, "I",
	    "LED state: 0 - off, 1 - on.");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "led_color_r", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY, sc,
	    PS5DS_SYSCTL_LED_COLOR_R, ps5ds_sysctl, "I",
	    "Lightbar color. Red component.");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "led_color_g", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY, sc,
	    PS5DS_SYSCTL_LED_COLOR_G, ps5ds_sysctl, "I",
	    "Lightbar color. Green component.");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "led_color_b", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY, sc,
	    PS5DS_SYSCTL_LED_COLOR_B, ps5ds_sysctl, "I",
	    "Lightbar color. Blue component.");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "player_leds", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY, sc,
	    PS5DS_SYSCTL_PLAYER_LEDS, ps5ds_sysctl, "I",
	    "Player indicator LEDs: -1 - off, 0-4 - player number.");

	error = hidmap_attach(&sc->hm);
	if (error)
		return (error);

	sx_xlock(&sc->lock);
	ps5ds_write(sc);
	sx_xunlock(&sc->lock);

	return (0);
}

static int
ps5ds_detach(device_t dev)
{
	struct ps5ds_softc *sc = device_get_softc(dev);

	sc->led_state = PS5DS_LED_OFF;
	sc->player_leds = -1;
	ps5ds_write(sc);
	hidmap_detach(&sc->hm);
	sx_destroy(&sc->lock);

	return (0);
}

static device_method_t ps5ds_methods[] = {
	DEVMETHOD(device_identify,	ps5ds_identify),
	DEVMETHOD(device_probe,		ps5ds_probe),
	DEVMETHOD(device_attach,	ps5ds_attach),
	DEVMETHOD(device_detach,	ps5ds_detach),

	DEVMETHOD_END
};

DEFINE_CLASS_0(ps5dualsense, ps5ds_driver, ps5ds_methods,
    sizeof(struct ps5ds_softc));
DRIVER_MODULE(ps5dualsense, hidbus, ps5ds_driver, NULL, NULL);

MODULE_DEPEND(ps5dualsense, hid, 1, 1, 1);
MODULE_DEPEND(ps5dualsense, hidbus, 1, 1, 1);
MODULE_DEPEND(ps5dualsense, hidmap, 1, 1, 1);
MODULE_DEPEND(ps5dualsense, hgame, 1, 1, 1);
MODULE_DEPEND(ps5dualsense, evdev, 1, 1, 1);
MODULE_VERSION(ps5dualsense, 1);
HID_PNP_INFO(ps5ds_devs);
