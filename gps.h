/**************************************************************
 * @file gps.h
 * @brief Header file for GPS module handling NMEA data parsing
 *
 * **********************************************************/

#ifndef GPS_H
#define GPS_H

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* Logging Macros */
#define GPS_INFO(fmt, ...)                                                     \
  syslog(LOG_INFO, "[%s] INFO: " fmt, LOG_TAG, ##__VA_ARGS__)
#define GPS_DEBUG(fmt, ...)                                                    \
  syslog(LOG_DEBUG, "[%s] DEBUG: " fmt, LOG_TAG, ##__VA_ARGS__)
#define GPS_ERROR(fmt, ...)                                                    \
  syslog(LOG_ERR, "[%s] ERROR: " fmt, LOG_TAG, ##__VA_ARGS__)

/* Constants */
#define DEF_DEV "/dev/ttyS2"
#define LOG_TAG "GPS"
#define MAX_NMEA_FIELDS 20           /* Max fields in an NMEA sentence */
#define NMEA_SENTENCE_MAX_LENGTH 256 /* Max length of an NMEA sentence */
#define GPS_BAUD_RATE B9600          /* Baud rate */

/*
 * @brief Structure to store parsed GPS data
 */
typedef struct {
  double latitude;    /* Latitude in decimal degrees */
  double longitude;   /* Longitude in decimal degress */
  double speed_kmh;   /* Speed in km/h */
  double altitude;    /* Altitude in meters */
  char timestamp[20]; /* Timestamp in HHMMSS format */
} GPS_Data;

/* Function Prototypes */

// Opening a GPS UART Port
int OpenGPSPort(const char *devname);

// Validating NMEA checksums
bool ValidateNMEAChecksum(const char *sentence);

// Parsing comma-sparated NMEA sentences
int ParseCommaDelimitedStr(char *string, char **fields, int max_fields);

// Converting NMEA coordinate to decimal format
double ConvertNMEAToDecimal(const char *nmea);

// Converting speed from knots to km/h
double ConvertKnotsToKmh(double knots);

// Settting system time from GPS timestamps
int SetSystemTime(const char *date, const char *time);

#endif
