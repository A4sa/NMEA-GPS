/**************************************************************************************************************
 * Filename: gps.c
 * Author: Abdul Sattar <abdul.linuxdev@gmail.com>
 *
 * Description:
 * This program reads GPS data from the MAX-M10S module using UART
 * communication. It extracts latitude, longitude, speed, and other relevant
 * information for vehicle tracking in the ADAS project.
 *
 * Hardware: Ambarella SDK -ARM Cortex-A0
 *
 * Reference:
 * MAX-M10S Datasheet:
 * https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf
 * NMEA Protocol Documentation:
 * https://cdn.sparkfun.com/assets/a/3/2/f/a/NMEA_Reference_Manual-Rev2.1-Dec07.pdf
 *
 ***************************************************************************************************************/
#include "gps.h"

int OpenGPSPort(const char *devname) {

  int fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    GPS_ERROR("Error opening GPS: %s", strerror(errno));
    return -1;
  }

  struct termios options;
  tcgetattr(fd, &options);
  cfmakeraw(&options);

  cfsetispeed(&options, GPS_BAUD_RATE);
  cfsetospeed(&options, GPS_BAUD_RATE);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_lflag = 0;
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;

  tcsetattr(fd, TCSANOW, &options);
  GPS_INFO("GPS port %s opened at 115200 baud", devname);
  return fd;
}

bool ValidateNMEAChecksum(const char *sentence) {
  char *checksum_str = strchr(sentence, '*');
  if (!checksum_str) {
    GPS_ERROR("Checksum missing in NMEA sentence: %s", sentence);
    return false;
  }

  unsigned char checksum = 0;
  for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
    checksum ^= sentence[i];
  }

  return (checksum == strtol(checksum_str + 1, NULL, 16));
}

int ParseCommaDelimitedStr(char *string, char **fields, int max_fields) {
  int i = 0;
  if (!string)
    return 0;

  fields[i++] = string;
  while ((i < max_fields) && (string = strchr(string, ','))) {
    *string = '\0';
    fields[i++] = ++string;
  }

  fields[i] = NULL;
  return i - 1;
}

double ConvertNMEAToDecimal(const char *nmea) {
  if (!nmea || strlen(nmea) < 6) {
    GPS_ERROR("Invalid NMEA coordinate: %s", nmea);
    return 0.0;
  }

  double degrees = atof(nmea) / 100;
  int intDeg = (int)degrees;
  double minutes = (degrees - intDeg) * 100;

  return intDeg + (minutes / 60);
}

double ConvertKnotsToKmh(double knots) { return knots * 1.852; }

int SetSystemTime(const char *date, const char *time) {
  struct timespec ts;
  struct tm gpstime;
  char tempbuf[3];

  if ((strlen(date) != 6) || (strlen(time) != 9)) {
    GPS_ERROR("Invalid date or time format: Date=%s, Time=%s", date, time);
    return -1;
  }

  strncpy(tempbuf, date, 2);
  tempbuf[2] = '\0';
  gpstime.tm_mday = atoi(tempbuf);

  strncpy(tempbuf, date + 2, 2);
  tempbuf[2] = '\0';
  gpstime.tm_mon = atoi(tempbuf) - 1;

  strncpy(tempbuf, date + 4, 2);
  tempbuf[2] = '\0';
  gpstime.tm_year = atoi(tempbuf) + 100;

  strncpy(tempbuf, time, 2);
  tempbuf[2] = '\0';
  gpstime.tm_hour = atoi(tempbuf);

  strncpy(tempbuf, time + 2, 2);
  tempbuf[2] = '\0';
  gpstime.tm_min = atoi(tempbuf);

  strncpy(tempbuf, time + 4, 2);
  tempbuf[2] = '\0';
  gpstime.tm_sec = atoi(tempbuf);

  ts.tv_sec = mktime(&gpstime);
  ts.tv_nsec = 0;

  if (clock_settime(CLOCK_REALTIME, &ts) < 0) {
    GPS_ERROR("Failed to set system time: %s", strerror(errno));
    return -1;
  }

  GPS_INFO("System time set successfully: Date=%s, Time=%s", date, time);
  return 0;
}

ssize_t ReadCompleteSentence(int fd, char *buffer, size_t bufsize) {
  size_t index = 0;
  char c;
  struct timeval timeout = {2, 0}; // 2 sec timeout
  fd_set set;

  FD_ZERO(&set);
  FD_SET(fd, &set);

  if (select(fd + 1, &set, NULL, NULL, &timeout) <= 0) {
    GPS_ERROR("GPS Read timeout");
    return -1;
  }
  while (index < bufsize - 1) {
    if (read(fd, &c, 1) <= 0)
      return -1;
    buffer[index++] = c;
    if (c == '\n')
      break;
  }

  buffer[index] = '\0';
  return index;
}

int main(int argc, char **argv) {
  int fd;
  char buffer[NMEA_SENTENCE_MAX_LENGTH];
  const char *dev = (argc > 1) ? argv[1] : DEF_DEV;

  openlog(LOG_TAG, LOG_PID | LOG_CONS, LOG_USER);

  fd = OpenGPSPort(dev);
  if (fd < 0) {
    closelog();
    return EXIT_FAILURE;
  }

  while (1) {
    int nbytes = ReadCompleteSentence(fd, buffer, sizeof(buffer));
    if (nbytes < 0) {
      GPS_ERROR("Read error: %s", strerror(errno));
      break;
    } else if (nbytes == 0) {
      GPS_DEBUG("No data received from GPS module");
      sleep(1);
    } else {
      if (ValidateNMEAChecksum(buffer)) {
        char *fields[MAX_NMEA_FIELDS];
        int num_fields =
            ParseCommaDelimitedStr(buffer, fields, MAX_NMEA_FIELDS);

        if (strncmp(fields[0], "$GPGGA", 6) == 0 && num_fields >= 6) {
          double latitude = ConvertNMEAToDecimal(fields[2]);
          double longitude = ConvertNMEAToDecimal(fields[4]);
          GPS_INFO("Latitude: %.6f, Longitude: %.6f", latitude, longitude);
        } else if (strncmp(fields[0], "$GPRMC", 6) == 0 && num_fields >= 7) {
          double speedKmh = ConvertKnotsToKmh(atof(fields[7]));
          GPS_INFO("Speed: %.2f km/h", speedKmh);
        }
      }
    }
  }

  close(fd);
  closelog();
  return EXIT_SUCCESS;
}
