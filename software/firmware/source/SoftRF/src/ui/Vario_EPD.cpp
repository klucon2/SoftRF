/*
 * View_Vario_EPD.cpp
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

// this is modified from v1.1.

#include "../system/SoC.h"

#if defined(USE_EPAPER)

#include "../driver/EPD.h"
#include "../TrafficHelper.h"
#include "../driver/Battery.h"
#include "../driver/Settings.h"
#include "../driver/RF.h"
#include "../driver/GNSS.h"
#include <protocol.h>
#include "../protocol/data/NMEA.h"
#include "../Wind.h"

#include <gfxfont.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>

#define ALTITUDE_HISTORY_SIZE 100 // Number of data points to store

static int32_t altitude_history[ALTITUDE_HISTORY_SIZE]; // Buffer for altitude values
static uint32_t time_history[ALTITUDE_HISTORY_SIZE];    // Buffer for timestamps
static uint8_t history_index = 0;                      // Current index in the buffer


extern uint32_t tx_packets_counter, rx_packets_counter;

static navbox_t variobox1;
static navbox_t variobox2;
static navbox_t variobox3;
static navbox_t variobox4;
static navbox_t variobox5;
static navbox_t variobox6;

void EPD_vario_setup()
{
  
  // Initialize altitude_history with 0
  for (uint8_t i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
    altitude_history[i] = ThisAircraft.pressure_altitude;
  }
  
  memcpy(variobox1.title, VARIOBOX1_TITLE, strlen(VARIOBOX1_TITLE));
  variobox1.x = 0;
  variobox1.y = 0;
  variobox1.width  = display->width() ;
  variobox1.height = display->height() / 6;
  variobox1.value      = 0;
//  variobox1.prev_value = variobox1.value;
  variobox1.timestamp  = millis();

  memcpy(variobox2.title, VARIOBOX2_TITLE, strlen(VARIOBOX2_TITLE));
  variobox2.x = variobox1.width;
  variobox2.y = variobox1.y;
  variobox2.width  = 0;
  variobox2.height = variobox1.height;
  variobox2.value      = 0;
//  variobox2.prev_value = variobox2.value;
  variobox2.timestamp  = millis();

  memcpy(variobox3.title, VARIOBOX3_TITLE, strlen(VARIOBOX3_TITLE));
  variobox3.x = variobox1.x;
  variobox3.y = variobox1.y + variobox1.height;
  variobox3.width  = variobox1.width;
  variobox3.height = variobox1.height*3;
  variobox3.value = 0;                      // was aircaft ID
//  variobox3.prev_value = variobox3.value;
  variobox3.timestamp  = millis();

  memcpy(variobox4.title, VARIOBOX4_TITLE, strlen(VARIOBOX4_TITLE));
  variobox4.x = variobox3.width;
  variobox4.y = variobox3.y;
  variobox4.width  = 0;
  variobox4.height = variobox3.height;
  variobox4.value = 0;                      // was settings->rf_protocol;
//  variobox4.prev_value = variobox4.value;
  variobox4.timestamp  = millis();

  memcpy(variobox5.title, VARIOBOX5_TITLE, strlen(VARIOBOX5_TITLE));
  variobox5.x = variobox3.x;
  variobox5.y = variobox3.y + variobox3.height;
  variobox5.width  = variobox1.width/2;
  variobox5.height = variobox1.height*2;
  variobox5.value  = 0;
//  variobox5.prev_value = variobox5.value;
  variobox5.timestamp  = millis();

  memcpy(variobox6.title, VARIOBOX6_TITLE, strlen(VARIOBOX6_TITLE));
  variobox6.x = variobox5.width;
  variobox6.y = variobox5.y;
  variobox6.width  = variobox5.width;
  variobox6.height = variobox5.height;
  variobox6.value  = 0;
//  variobox6.prev_value = variobox6.value;
  variobox6.timestamp  = millis();
}

static void EPD_Draw_NavBoxes()
{
  char buf[16];
  uint32_t disp_value;

  int16_t  tbx, tby;
  uint16_t tbw, tbh;

#if defined(USE_EPD_TASK)
  if (EPD_update_in_progress == EPD_UPDATE_NONE) {
//  if (SoC->Display_lock()) {
#else
  {
#endif
    uint16_t top_navboxes_x = variobox1.x;
    uint16_t top_navboxes_y = variobox1.y;
    uint16_t top_navboxes_w = variobox1.width + variobox2.width;
    uint16_t top_navboxes_h = maxof2(variobox1.height, variobox2.height);

    display->fillScreen(GxEPD_WHITE);

    display->drawRoundRect( variobox1.x + 1, variobox1.y + 1,
                            variobox1.width - 2, variobox1.height - 2,
                            4, GxEPD_BLACK);

    display->drawRoundRect( variobox2.x + 1, variobox2.y + 1,
                            variobox2.width - 2, variobox2.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(variobox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox1.x + 5, variobox1.y + 5 + tbh);
    
    display->print(variobox1.title);

    /*display->getTextBounds(variobox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox2.x + 5, variobox2.y + 5 + tbh);
    display->print(variobox2.title);*/

    display->setFont(&FreeMonoBold18pt7b);

    display->getTextBounds((String)variobox1.value, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor((variobox1.width-tbw)/2, ((variobox1.height-tbh)/2 + tbh));
    display->print((String)variobox1.value +"m");

    /*display->setCursor(variobox2.x + 15, variobox2.y + 52);
    display->print(variobox2.value);*/

    uint16_t middle_navboxes_x = variobox3.x;
    uint16_t middle_navboxes_y = variobox3.y;
    uint16_t middle_navboxes_w = variobox3.width + variobox4.width;
    uint16_t middle_navboxes_h = maxof2(variobox3.height, variobox4.height);

    display->drawRoundRect( variobox3.x + 1, variobox3.y + 1,
                            variobox3.width - 2, variobox3.height - 2,
                            4, GxEPD_BLACK);
    /*display->drawRoundRect( variobox4.x + 1, variobox4.y + 1,
                            variobox4.width - 2, variobox4.height - 2,
                            4, GxEPD_BLACK);*/

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(variobox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox3.x + 5, variobox3.y + 5 + tbh);
    display->print(variobox3.title);

    /*display->getTextBounds(variobox4.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox4.x + 5, variobox4.y + 5 + tbh);
    display->print(variobox4.title);*/

    display->setFont(&FreeMonoBold18pt7b);

    //display->setCursor(variobox3.x + 25, variobox3.y + 50);
    //display->print(variobox3.value);

//  display->setFont(&FreeMonoBold18pt7b);

    /*display->setCursor(variobox4.x + 25, variobox4.y + 50);
    snprintf(buf, sizeof(buf), "%d", variobox4.value);
    display->print(buf);*/

    uint16_t bottom_navboxes_x = variobox5.x;
    uint16_t bottom_navboxes_y = variobox5.y;
    uint16_t bottom_navboxes_w = variobox5.width + variobox4.width;
    uint16_t bottom_navboxes_h = maxof2(variobox5.height, variobox6.height);

    display->drawRoundRect( variobox5.x + 1, variobox5.y + 1,
                            variobox5.width - 2, variobox5.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( variobox6.x + 1, variobox6.y + 1,
                            variobox6.width - 2, variobox6.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    display->getTextBounds(variobox5.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox5.x + 5, variobox5.y + 5 + tbh);
    display->print(variobox5.title);
    display->getTextBounds("km/h", 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox5.x + variobox5.width-tbw-8, variobox5.y + variobox5.height-5);
    display->print("km/h");

    display->getTextBounds(variobox6.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox6.x + 5, variobox6.y + 5 + tbh);
    display->print(variobox6.title);

    display->setFont(&FreeMonoBold18pt7b);

    display->getTextBounds((String)variobox5.value, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor((variobox5.width-tbw)/2, (variobox1.height +variobox3.height +((variobox5.height-tbh)/2) + tbh));
    display->print(variobox5.value);

    display->getTextBounds((String)variobox6.value, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(variobox5.width+((variobox6.width-tbw)/2), (variobox1.height +variobox3.height +((variobox6.height-tbh)/2) + tbh));
    //display->print((String)variobox6.value +"°");


    // Calculate relative wind direction
    float relativeWindDirection = wind_direction - ThisAircraft.heading;
    if (relativeWindDirection < 0) {
        relativeWindDirection += 360.0;
    }

    // Define the center of variobox6 for the arrow
    uint16_t arrowCenterX = variobox6.x + variobox6.width / 2;
    uint16_t arrowCenterY = variobox6.y + variobox6.height / 2;
    uint16_t arrowLength = variobox6.height *0.65; // Arrow length relative to box size

    // Draw the wind direction arrow
    drawWindArrow(arrowCenterX, arrowCenterY, arrowLength, relativeWindDirection);

#if defined(USE_EPD_TASK)
    /* a signal to background EPD update task */
    EPD_update_in_progress = EPD_UPDATE_FAST;
//    SoC->Display_unlock();
//    yield();
#else
    display->display(true);
#endif
  }
}

void EPD_vario_loop()
{
  if (isTimeToEPD()) {

    int32_t current_altitude = ThisAircraft.pressure_altitude;
    
    variobox1.value = current_altitude;
    //variobox2.value = ThisAircraft.vs*(_GPS_FEET_PER_METER * 60); // m/s
    //variobox3.value = ThisAircraft.vs*(_GPS_FEET_PER_METER * 60); // m/s
    //variobox4.value = alarm_level;
    variobox5.value = ThisAircraft.speed*(_GPS_KMPH_PER_KNOT); // knots
    variobox6.value = wind_direction;

    updateAltitudeHistory(current_altitude);

    EPD_Draw_NavBoxes();
    drawAltitudeGraph();

    EPDTimeMarker = millis();
  }
}

void EPD_vario_next()
{

}

void EPD_vario_prev()
{

}






void updateAltitudeHistory(int32_t altitude) {
  // Store the current altitude and timestamp in the history buffers
  altitude_history[history_index] = altitude;
  time_history[history_index] = millis();
  history_index = (history_index + 1) % ALTITUDE_HISTORY_SIZE; // Circular buffer
}

void drawAltitudeGraph() {
  // Define graph area
  const uint16_t graph_x = variobox3.x + 2;
  const uint16_t graph_y = variobox3.y + 2;
  const uint16_t graph_width = variobox3.width - 4; // Adjust width as needed
  const uint16_t graph_height = variobox3.height - 4;

  // Clear the graph area
  //display->fillRect(graph_x, graph_y, graph_width, graph_height, GxEPD_WHITE);
  //display->drawRect(graph_x, graph_y, graph_width, graph_height, GxEPD_BLACK);

  // Find min and max altitude for scaling
  int32_t min_altitude = altitude_history[0];
  int32_t max_altitude = altitude_history[0];
  for (uint8_t i = 1; i < ALTITUDE_HISTORY_SIZE; i++) {
      if (altitude_history[i] < min_altitude) min_altitude = altitude_history[i];
      if (altitude_history[i] > max_altitude) max_altitude = altitude_history[i];
  }

  // Add a margin of 5 meters to min and max values
  min_altitude -= 5;
  max_altitude += 5;

  // Avoid division by zero
  if (max_altitude == min_altitude) max_altitude++;

  // Draw the graph
  for (uint8_t i = 1; i < ALTITUDE_HISTORY_SIZE; i++) {
      uint8_t prev_index = (history_index + i - 1) % ALTITUDE_HISTORY_SIZE;
      uint8_t curr_index = (history_index + i) % ALTITUDE_HISTORY_SIZE;

      uint16_t x1 = graph_x + (graph_width * (i - 1)) / ALTITUDE_HISTORY_SIZE;
      uint16_t y1 = graph_y + graph_height - ((altitude_history[prev_index] - min_altitude) * graph_height) / (max_altitude - min_altitude);
      uint16_t x2 = graph_x + (graph_width * i) / ALTITUDE_HISTORY_SIZE;
      uint16_t y2 = graph_y + graph_height - ((altitude_history[curr_index] - min_altitude) * graph_height) / (max_altitude - min_altitude);

      display->drawLine(x1, y1, x2, y2, GxEPD_BLACK);
  }
}

// Function to draw a wind direction arrow
void drawWindArrow(uint16_t centerX, uint16_t centerY, uint16_t length, float angleDegrees) {
  // Convert angle to radians
  float angleRadians = angleDegrees * M_PI / 180.0;

  // Calculate the starting and ending points of the arrow symmetrically around the center
  uint16_t startX = centerX - (length / 2) * cos(angleRadians);
  uint16_t startY = centerY + (length / 2) * sin(angleRadians);
  uint16_t endX = centerX + (length / 2) * cos(angleRadians);
  uint16_t endY = centerY - (length / 2) * sin(angleRadians); // Y-axis is inverted in most displays

  // Draw the main arrow line (thicker by drawing multiple parallel lines)
  for (int8_t offset = -2; offset <= 2; offset++) { // Adjust thickness by changing range
    display->drawLine(startX + offset, startY, endX + offset, endY, GxEPD_BLACK);
    display->drawLine(startX, startY + offset, endX, endY + offset, GxEPD_BLACK);
  }

  // Draw arrowhead
  uint16_t arrowheadLength = length / 1.5; // Length of the arrowhead
  float arrowAngle1 = angleRadians + M_PI / 6; // 30 degrees offset
  float arrowAngle2 = angleRadians - M_PI / 6; // 30 degrees offset

  uint16_t endXArrow = centerX + (length / 1.7) * cos(angleRadians);
  uint16_t endYArrow = centerY - (length / 1.7) * sin(angleRadians); // Y-axis is inverted in most displays

  uint16_t arrowX1 = endXArrow - arrowheadLength * cos(arrowAngle1);
  uint16_t arrowY1 = endYArrow + arrowheadLength * sin(arrowAngle1);
  uint16_t arrowX2 = endXArrow - arrowheadLength * cos(arrowAngle2);
  uint16_t arrowY2 = endYArrow + arrowheadLength * sin(arrowAngle2);

  // Draw thicker arrowhead lines
  /*for (int8_t offset = -3; offset <= 3; offset++) { // Adjust thickness by changing range
    display->drawLine(endX + offset, endY, arrowX1 + offset, arrowY1, GxEPD_BLACK);
    display->drawLine(endX + offset, endY, arrowX2 + offset, arrowY2, GxEPD_BLACK);
  }*/

  // Draw the triangle for the arrowhead
  display->fillTriangle(endXArrow, endYArrow, arrowX1, arrowY1, arrowX2, arrowY2, GxEPD_BLACK);
}


#endif /* USE_EPAPER */
