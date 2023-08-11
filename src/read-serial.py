import os
import time
import csv
from datetime import datetime
import serial

arduino_port = 'com8'
arduino_rate = 9600

def download_data():
    arduino = serial.Serial(arduino_port, arduino_rate)
    time.sleep(1)
    now = datetime.now()
    file_name = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{now.strftime('%Y-%m-%d_%H-%M-%S')}.csv")

    with open(file_name, 'w', newline='', encoding='utf-8') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['Timestamp', 'Data'])

        try:
            while True:
                data = arduino.readline()
                data = data.decode('utf-8', errors='ignore').strip()
                if data:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                    print(f"Timestamp: {timestamp}, Data: {data}")
                    csv_writer.writerow([timestamp, data])
        except KeyboardInterrupt:
            print("Download stopped by user.")
        finally:
            arduino.close()

if __name__ == '__main__':
    download_data()
