/*
 * EPDHelper.h
 * Copyright (C) 2019-2022 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef EPDHELPER_H
#define EPDHELPER_H

#if defined(USE_EPAPER)
#define ENABLE_GxEPD2_GFX       1
#include <GxEPD2_BW.h>
#endif /* USE_EPAPER */

#define EPD_EXPIRATION_TIME     5 /* seconds */

#define NO_DATA_TEXT            "NO DATA"
#define NO_FIX_TEXT             "NO FIX"

#define NAVBOX1_TITLE           "SAT"
#define NAVBOX2_TITLE           "BAT"
#define NAVBOX3_TITLE           "ACFTS"
#define NAVBOX4_TITLE           "ALARM"
#define NAVBOX5_TITLE           "RX"
#define NAVBOX6_TITLE           "TX"

//CK:added for Vario View
#define VARIOBOX1_TITLE           "ALT"
#define VARIOBOX2_TITLE           "LFT"
#define VARIOBOX3_TITLE           ""
#define VARIOBOX4_TITLE           ""
#define VARIOBOX5_TITLE           "SPD"
#define VARIOBOX6_TITLE           "WND"

#define isTimeToEPD()           (millis() - EPDTimeMarker > 1000)
#define maxof2(a,b)             (a > b ? a : b)

#define EPD_RADAR_V_THRESHOLD   50      /* metres */

#define TEXT_VIEW_LINE_LENGTH   13     /* characters */
#define TEXT_VIEW_LINE_SPACING  12     /* pixels */
#define CONF_VIEW_LINE_LENGTH   18     /* characters */
#define CONF_VIEW_LINE_SPACING  12     /* pixels */
#define INFO_1_LINE_SPACING     7      /* pixels */


//#define EPD_HIBERNATE         {}
#define EPD_HIBERNATE           display->hibernate()
//#define EPD_HIBERNATE         display->powerOff()

//#define	EPD_POWEROFF		      {}
#define EPD_POWEROFF            display->powerOff()

enum
{
	EPD_UPDATE_NONE = 0,
	EPD_UPDATE_SLOW,
	EPD_UPDATE_FAST
};

enum
{
	VIEW_MODE_VARIO = 0,
  VIEW_MODE_STATUS,
	VIEW_MODE_RADAR,
	VIEW_MODE_TEXT,
	VIEW_MODE_BARO,
	VIEW_MODE_TIME,
	VIEW_MODE_IMU,
	VIEW_MODE_CONF,
	VIEW_MODES_COUNT,
	VIEW_CHANGE_SETTINGS,     // only reached via double-click on VIEW_MODE_CONF
	VIEW_SAVE_SETTINGS,
	VIEW_REBOOT,
	VIEW_DONE
};

/*
 * 'Radar view' scale factor (outer circle diameter)
 *
 * Metric and Mixed:
 *  LOWEST - 20 KM diameter ( 10 KM radius)
 *  LOW    - 10 KM diameter (  5 KM radius)
 *  MEDIUM -  5 KM diameter (2.5 KM radius)
 *  HIGH   -  2 KM diameter (  1 KM radius)
 *
 * Imperial:
 *  LOWEST - 10 NM diameter (  5 NM radius)
 *  LOW    -  5 NM diameter (2.5 NM radius)
 *  MEDIUM -  2 NM diameter (  1 NM radius)
 *  HIGH   -  1 NM diameter (0.5 NM radius)
 */
enum
{
	ZOOM_LOWEST,
	ZOOM_LOW,
	ZOOM_MEDIUM,
	ZOOM_HIGH
};

enum
{
	UNITS_METRIC,
	UNITS_IMPERIAL,
	UNITS_MIXED     // almost the same as metric, but all the altitudes are in feet
};

enum
{
	PROTOCOL_NONE,
	PROTOCOL_NMEA, /* FTD-12 */
	PROTOCOL_GDL90,
	PROTOCOL_MAVLINK_1,
	PROTOCOL_MAVLINK_2,
	PROTOCOL_D1090,
	PROTOCOL_UATRADIO
};

enum
{
	ID_REG,
	ID_TAIL,
	ID_MAM,
	ID_TYPE
};

enum
{
	VOICE_0=0,
	VOICE_1,
	VOICE_2,
	VOICE_3
};

enum
{
	ANTI_GHOSTING_OFF,
	ANTI_GHOSTING_AUTO,
	ANTI_GHOSTING_2MIN,
	ANTI_GHOSTING_5MIN,
	ANTI_GHOSTING_10MIN
};

enum
{
	TRAFFIC_FILTER_OFF,
	TRAFFIC_FILTER_500M,
	TRAFFIC_FILTER_1500M
};

enum
{
	DB_NONE,
	DB_AUTO,
	DB_FLN,
	DB_OGN,
	DB_ICAO
};

enum
{
	ROTATE_0,
	ROTATE_90,
	ROTATE_180,
	ROTATE_270
};

typedef struct UI_Settings {
    uint8_t  adapter;

    uint8_t  connection:4;
    uint8_t  units:2;
    uint8_t  zoom:2;

    uint8_t  protocol;
    uint8_t  baudrate;
    char     server  [18];
    char     key     [18];

    uint8_t  rotate:2;
    uint8_t  orientation:1;
    uint8_t  adb:3;
    uint8_t  epdidpref:2;

    uint8_t  viewmode:3;
    uint8_t  voice:2;
    uint8_t  antighost:3;

    uint8_t  filter:4;
    uint8_t  power_save:4;

    uint32_t team;

    uint8_t  resvd12;
    uint8_t  resvd3;
    uint8_t  resvd4;
    uint8_t  resvd5;
    uint8_t  resvd6;
    uint8_t  resvd7;
    uint8_t  resvd8;
    uint8_t  resvd9;
} __attribute__((packed)) ui_settings_t;

typedef struct navbox_struct
{
  char      title[16];
  uint16_t  x;
  uint16_t  y;
  uint16_t  width;
  uint16_t  height;
  int32_t   value;
//  int32_t   prev_value;
  uint32_t  timestamp;
} navbox_t;

bool EPD_setup(bool);
void EPD_loop();
void EPD_fini(int, bool);
void EPD_info1();
void EPD_info2(int, char *, char *, char *);

void EPD_Mode(bool same_page);
void EPD_Up();
void EPD_Down();
void EPD_Message(const char *, const char *);

void EPD_vario_setup();
void EPD_vario_loop();
void EPD_vario_next();
void EPD_vario_prev();

void EPD_status_setup();
void EPD_status_loop();
void EPD_status_next();
void EPD_status_prev();

void EPD_radar_setup();
void EPD_radar_loop();
void EPD_radar_zoom();
void EPD_radar_unzoom();

void EPD_text_setup();
void EPD_text_loop();
void EPD_text_next();
void EPD_text_prev();

void EPD_conf_setup();
void EPD_conf_loop();
void EPD_conf_next();
void EPD_conf_prev();

void EPD_chgconf_save();
void EPD_chgconf_loop();
void EPD_chgconf_page();
void EPD_chgconf_next();
void EPD_chgconf_prev();

void EPD_baro_setup();
void EPD_baro_loop();
void EPD_baro_next();
void EPD_baro_prev();

void EPD_imu_setup();
void EPD_imu_loop();
void EPD_imu_next();
void EPD_imu_prev();

void EPD_time_setup();
void EPD_time_loop();
void EPD_time_next();
void EPD_time_prev();

void updateAltitudeHistory(int32_t altitude);
void drawAltitudeGraph();
void drawWindArrow(uint16_t centerX, uint16_t centerY, uint16_t length, float angleDegrees);

#if defined(USE_EPAPER)
EPD_Task_t EPD_Task(void *);
extern GxEPD2_GFX *display;
#endif /* USE_EPAPER */

extern unsigned long EPDTimeMarker;
extern int EPD_view_mode;
extern int EPD_prev_view;
extern bool conf_initialized;
extern bool EPD_vmode_updated;
extern uint16_t EPD_pages_mask;
extern volatile uint8_t EPD_update_in_progress;
extern const char *Aircraft_Type[];
extern const char *Region_Label[];
extern ui_settings_t ui_settings;
//extern ui_settings_t *ui;

static uint8_t sleep_icon_128x128[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00,
  0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00,
  0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00,
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00,
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00,
  0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00,
  0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00,
  0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00,
  0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00,
  0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x00,
  0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00,
  0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00,
  0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00,
  0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00,
  0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00,
  0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80,
  0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80,
  0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80,
  0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0,
  0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0,
  0x03, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0,
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0,
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0,
  0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x00, 0x00, 0x18, 0x00, 0x00, 0x06, 0x00, 0x00, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x1f, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x7c, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x3f, 0x00, 0x00, 0xfc, 0x00, 0x00, 0xfc, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x3f, 0xc0, 0x00, 0xff, 0x00, 0x00, 0xff, 0x00, 0x03, 0xfc, 0x00, 0x00, 0xf8,
  0x3f, 0x00, 0x00, 0x3f, 0xf0, 0x03, 0xff, 0x00, 0x00, 0xff, 0xc0, 0x0f, 0xfc, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x00, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x03, 0xff, 0xff, 0x00, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x00, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0xfc,
  0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc,
  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
  0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x1f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8,
  0x0f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0,
  0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0,
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0,
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0,
  0x03, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x07, 0xc0,
  0x03, 0xf0, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x0f, 0xc0,
  0x03, 0xf0, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x0f, 0xc0,
  0x01, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x1f, 0x80,
  0x01, 0xf8, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xe0, 0x07, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x1f, 0x80,
  0x01, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x3f, 0x80,
  0x00, 0xfc, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x3f, 0x00,
  0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00,
  0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00,
  0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00,
  0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00,
  0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x00,
  0x00, 0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf8, 0x00,
  0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00,
  0x00, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00,
  0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xe0, 0x00,
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00,
  0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00,
  0x00, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00,
  0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00,
  0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00,
  0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfc, 0x00, 0x00,
  0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00,
  0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00,
  0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#endif /* EPDHELPER_H */
