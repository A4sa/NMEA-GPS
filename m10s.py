## Install pynmea2 package 
import serial
import pynmea2 
import datetime


serial_port = '/dev/ttyUSB0'  # Change this to match your serial port
baud_rate = 9600  # Match this to the baud rate of your MAX-M10S module

# Initialize the serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

def read_nmea_sentence():
    """
    Read and parse NMEA sentence from the MAX-M10S module.
    """
    try:
        sentence = ser.readline().decode('utf-8', errors='replace').strip()
        parsed_sentence = pynmea2.parse(sentence)

        return parsed_sentence
    except pynmea2.ParseError:
        return None
    except UnicodeDecodeError:
        return None


def main():
    while True:
        # Read and parse NMEA sentence
        nmea_sentence = read_nmea_sentence()
        if nmea_sentence and isinstance(nmea_sentence, pynmea2.RMC):
            # Extract parameters
            gps_data = str(nmea_sentence)
            latitude = nmea_sentence.latitude
            longitude = nmea_sentence.longitude
            
            # Check if speed is available
            if nmea_sentence.spd_over_grnd is not None:
                speed = nmea_sentence.spd_over_grnd * 1.852  # Convert speed from knots to km/h
            else:
                speed = "Speed not available"
                
            # Check if timestamp is available
            if nmea_sentence.timestamp is not None:
                time_utc = datetime.datetime.combine(datetime.date.today(), nmea_sentence.timestamp)
                time_ist = time_utc + datetime.timedelta(hours=5, minutes=30)  # Assuming IST is 5 hours 30 minutes ahead of UTC
            else:
                time_utc = "Timestamp not available"
                time_ist = "Timestamp not available"
            
            # Print parameters
            print("NMEA Sentence: {}".format(gps_data.strip()))
            print("Latitude: {}, Longitude: {}".format(latitude, longitude))
            print("UTC Time: {}".format(time_utc))
            print("IST Time: {}".format(time_ist))
            print(f"Speed: {speed} km/h")
            print("= = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = ")
if __name__ == "__main__":
    main()
