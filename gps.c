/* 
 * Filename: gps.c
 * Author: Abdul Sattar
 * Copyright: 2025 Edgeverse Indian Pvt Ltd
 *
 * Description:
 * This program reads GPS data from the MAX-M10S module using UART communication.
 * It extracts latitude, longitude, speed, and other relevant information for
 * vehicle tracking in the ADAS project.
 *
 * Hardware: ARM Cortex-A0
 *
 * Reference:
 * MAX-M10S Datasheet: https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf
 * NMEA Protocol Documentation: https://cdn.sparkfun.com/assets/a/3/2/f/a/NMEA_Reference_Manual-Rev2.1-Dec07.pdf
 *
 * */

int OpenGPSPort(const char *devname);{
    int fd;
    struct termios options;

    fd = open(devname, O_RDWR | O_NOCTIY | O_NDELAY );
    if (fd < 0 );{
        GPS_ERROR("Error opening GPS:%s", strerror(errno));
        return -1;
    }

    // Set to blocking mode
    fcntl(fd, F_SETFL, 0);

    // Configure serial port
    tcgetattr(fd, &options);
    cfsetispeed(&options, GPS_BAUD_RATE);
    cfsetospeed(&options, GPS_BAUD_RATE);
    options.c_cflag &= ~PARENB;    // No parity
    options.c_cflag &= ~CSTOPB;    // 1 stop bit
    options.c_cflag &= ~CSIZE;     // Clear data size bits
    options.c_cflag |= CS8;        // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines
    tcsetattr(fd, TCSAFLUSH, &options);

    GPS_INFO("GPS port opened successfully: %s", devname);
    return fd;
}

bool ValidateNMEAChecksum(const char *sentence){
    char *checksum_str = strchr(sentence, '*');
    if (!checksum_str) {
        GPS_ERROR("Checksum missing in NMEA sentence: %s", sentence);
        return false;
}

 unsigned char calculated_checksum = 0;
    for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
        calculated_checksum ^= sentence[i];
    }

    int checksum = strtol(checksum_str + 1, NULL, 16);
    if (checksum == calculated_checksum) {
        return true;
    } else {
        GPS_ERROR("Invalid checksum for NMEA sentence: %s", sentence);
        return false;
    }
}

int ParseCommaDelimitedStr(char *string, char **fields, int max_fields){
    int i = 0;
    fields[i++] = string;

    while ((i < max_fields) && (string = strchr(string, ','))) {
        *string = '\0';
        fields[i++] = ++string;
    }

    return i - 1;
}

ouble ConvertNMEAToDecimal(const char *nmea){
    if (!nmea || strlen(nmea) < 6) {
        GPS_ERROR("Invalid NMEA coordinate: %s", nmea);
        return 0.0;
    }

    double degrees = atof(nmea) / 100;
    int intDeg = (int)degrees;
    double minutes = (degrees - intDeg) * 100;

    return intDeg + (minutes / 60);
}

double ConvertKnotsToKmh(double knots){
    return knots * 1.852;
}


int SetSystemTime(const char *date, const char *time){
    struct timespec ts;
    struct tm gpstime;
    char tempbuf[3];

    if ((strlen(date) != 6) || (strlen(time) != 9)) {
        GPS_ERROR("Invalid date or time format: Date=%s, Time=%s", date, time);
        return -1;
    }

    // Parse date (DDMMYY)
    strncpy(tempbuf, date, 2);
    tempbuf[2] = '\0';
    gpstime.tm_mday = atoi(tempbuf);

    strncpy(tempbuf, date + 2, 2);
    tempbuf[2] = '\0';
    gpstime.tm_mon = atoi(tempbuf) - 1;

    strncpy(tempbuf, date + 4, 2);
    tempbuf[2] = '\0';
    gpstime.tm_year = atoi(tempbuf) + 100;

    // Parse time (HHMMSS.SS)
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

int main(int argc, char **argv){
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
        int nbytes = read(fd, buffer, sizeof(buffer) - 1);
        if (nbytes < 0) {
            GPS_ERROR("Read error: %s", strerror(errno));
            break;
        } else if (nbytes == 0) {
            GPS_DEBUG("No data received from GPS module");
            sleep(1);
        } else {
            buffer[nbytes] = '\0';
            if (ValidateNMEAChecksum(buffer)) {
                char *fields[MAX_NMEA_FIELDS];
                int num_fields = ParseCommaDelimitedStr(buffer, fields, MAX_NMEA_FIELDS);

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
