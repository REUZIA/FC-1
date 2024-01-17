import serial
import re
import time


class FileSystem:
    def __init__(self):
        self._files = []

    def init_from_serial(self, s: serial.Serial) -> bool:
        s.write(b'i')
        feedback = s.readline()
        if feedback != b'> i\n':
            return False

        s.readline() # Drop the first line
        while True:
            line = s.readline().decode('ascii')
            if 'b' in line: # End of the list
                break
            [index, addr, size] = re.findall(r'\d+', line)
            self._files.append((int(index), int(addr), int(size)))
        return True

    def get_file(self, index: int):
        if index >= len(self._files) or index < 0:
            return None
        return self._files[index]

    def read_file(self, s: serial.Serial, index: int):
        file = self.get_file(index)
        if file is None:
            return None

        s.write(bytes([114, index]))
        time.sleep(0.01)
        s.readline()
        s.readline()
        lines: list(bytearray) = []
        while True:
            line = s.readline()
            if b'read' in line or b'Error' in line:
                break
            lines.append(bytearray(line))
        
        lines = list(map(lambda x: x[0:len(x)-2], lines))
        return b''.join(lines)

    
    @property
    def files(self):
        return self._files

serial_connection = serial.Serial('COM4', 9600)
file_system = FileSystem()
file_system.init_from_serial(serial_connection)
print(file_system.files)
print(file_system.read_file(serial_connection, 0))

# s.write([114])
# time.sleep(0.1)
# s.readline()
# s.readline()
# while True:
#     print(s.readline())
#     if s.in_waiting <= 0:
#         break