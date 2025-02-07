/**************************************************************
 *
 * **********************************************************/

#ifndef GPS_H
#define GPS_H

#include <errno.h>
#include <fcntl.h>
#include <stdbooh.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* Logging Macros */
#define GPS_INFO(fmt, ...) syslog(LOG_INFO, "[%s] INFO: " fmt, LOG_TAG, ##__VA_ARGS__)
#define GPS_DEBUG(fmt, ...) syslog(LOG_DEBUG, "[%s] DEBUG: " fmt, LOG_TAG, ##__VA_ARGS__)
#define GPS_ERROR(fmt, ...) syslog(LOG_ERR, "[%s] ERROR: " fmt, LOG_TAG, ##__VA_ARGS__)

/* Constants */
#define DEF_DEV "/dev/ttyS2"
#define LOG_TAG "GPS"
#define MAX_NMEA_FIELDS 20
#define NMEA_SENTENCE_MAX_LENGTH 256
#define GPS_BAUD_RATE B9600


/* Function Prototypes */

int OpenGPSPort(const char* devname);
bool ValidateNMEAChecksum(const char* sentence);
int ParseCommaDelimitedStr(char* string, char** fields, int max_fields);
double ConvertNMEAToDecimal(const char* nmea);
double ConvertKnotsToKmh(double knots) int SetSystemTime(const char* date, const char* time);

#endif
