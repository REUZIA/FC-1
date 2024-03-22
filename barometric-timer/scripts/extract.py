import struct
import serial
import re
import time
import pandas as pd
import matplotlib.pyplot as plt


class FileSystem:
    def __init__(self):
        self._files = []

    def init_from_serial(self, s: serial.Serial) -> bool:
        s.write(b'ls /')
        for _ in range(4):
            s.readline() # Drop lines

        while True:
            line = s.readline().decode('ascii')
            if line == "": # End of the list
                break
            [(size, addr)] = re.findall(r'dir\((\d+)B\):([\/a-zA-Z_0-9]+)', line)
            self._files.append((int(size), addr))
        return True

    def get_file(self, index: int):
        if index >= len(self._files) or index < 0:
            return None
        return self._files[index]

    def read_file(self, s: serial.Serial, file: str) -> bytes|None:
        s.write((f"cat {file}/log").encode())
        time.sleep(0.01)
        s.readline()

        lines: list[bytes] = []
        first_line = False
        head = 0
        while True:
            line = s.readline()
            if line == b'' or line == b'end\r\n':
                break
            if b'read:' in line and not first_line:
                line = line.replace(b'read:', b'')
                first_line = True
            lines.append(bytes(line))
            head += len(line)
            time.sleep(0.01)
        lines[-1] = lines[-1][:-2] # Remove the \r\n of last line
        
        return b''.join(lines)

    @staticmethod
    def file_to_csv(file: bytes, file_path: str):
        values: list[int] = []
        for i in range(0, len(file), 4):
            values.append(int.from_bytes(file[i:i+4], byteorder='big', signed=True))
        df = pd.DataFrame(values)
        df = df / 256
        df.to_csv(file_path, index=False, header=False)

    
    @property
    def files(self):
        return self._files

if __name__ == '__main__':
    serial_connection = serial.Serial('COM7', 115200, timeout=1)
    file_system = FileSystem()
    file_system.init_from_serial(serial_connection)
    for (_, run_folder) in file_system.files:
        print(f"Reading: {run_folder}")
        file_bytes = file_system.read_file(serial_connection, run_folder)
        if file_bytes is None:
            print(f"Error reading {run_folder}")
            continue

        logs: list[tuple] = []
        pressures: list[tuple] = []
        speeds: list[tuple] = []
        # print(file_bytes)
        parser_head = 0
        while parser_head < len(file_bytes):
            match file_bytes[parser_head]:
                case 0x01:
                    assert file_bytes[parser_head+1] == ord('|')
                    parser_head += 2
                    timestamp = int.from_bytes(file_bytes[parser_head:parser_head+8], byteorder='little', signed=False)
                    parser_head += 8
                    assert file_bytes[parser_head] == ord('|')
                    log: str = ""
                    parser_head += 1
                    while file_bytes[parser_head] != 0x00:
                        log += file_bytes[parser_head:parser_head+1].decode()
                        parser_head += 1
                    parser_head += 1

                    logs.append((timestamp, log))
                    print(f"Timestamp: {timestamp} Log: {log}")
                case 0x02:
                    assert file_bytes[parser_head+1] == ord('|')
                    parser_head += 2
                    timestamp = int.from_bytes(file_bytes[parser_head:parser_head+8], byteorder='little', signed=False)
                    parser_head += 8
                    assert file_bytes[parser_head] == ord('|')
                    parser_head += 1

                    (temp,) = struct.unpack('<f', file_bytes[parser_head:parser_head+4])
                    parser_head += 4
                    (pressure,) = struct.unpack('<f', file_bytes[parser_head:parser_head+4])
                    parser_head += 4
                    (altitude,) = struct.unpack('<f', file_bytes[parser_head:parser_head+4])
                    parser_head += 4
                    assert file_bytes[parser_head] == 0x00
                    parser_head += 1

                    pressures.append((timestamp, temp, pressure, altitude))
                    print(f"Timestamp: {timestamp} Temp: {temp} Pressure: {pressure} Altitude: {altitude}")
                case 0x03:
                    assert file_bytes[parser_head+1] == ord('|')
                    parser_head += 2
                    timestamp = int.from_bytes(file_bytes[parser_head:parser_head+8], byteorder='little', signed=False)
                    parser_head += 8
                    assert file_bytes[parser_head] == ord('|')
                    parser_head += 1

                    (speed,) = struct.unpack('<f', file_bytes[parser_head:parser_head+4])
                    parser_head += 4
                    assert file_bytes[parser_head] == 0x00
                    parser_head += 1

                    speeds.append((timestamp, speed))
                    print(f"Timestamp: {timestamp} Speed: {speed}")
                case other:
                    print(f"Unknown type: {file_bytes[parser_head]}")
                    break

        plt.figure()
        plt.title("BMP temp (C)")
        press_time = [x[0] for x in pressures]
        plt.plot(press_time, [x[1] for x in pressures])
        plt.show()

        plt.figure()
        plt.title("BMP pressure (Pa)")
        press_time = [x[0] for x in pressures]
        plt.plot(press_time, [x[2] for x in pressures])
        plt.show()

        plt.figure()
        plt.title("Altitude (m)")
        press_time = [x[0] for x in pressures]
        plt.plot(press_time, [x[3] for x in pressures])
        plt.show()

        plt.figure()
        plt.title("Speed (m/s)")
        plt.plot([x[0] for x in speeds], [x[1] for x in speeds])
        plt.show()

