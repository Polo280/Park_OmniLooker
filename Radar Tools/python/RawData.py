import time 
import serial 
import numpy as np
import struct


class RadarHandler:
    def __init__(self, port:str, baud_rate=115200, timeout=2):
        # Serial parameters 
        self.port = port 
        self.baud_rate = baud_rate
        self.serial_timeout = timeout
        self.serial_handler = None
        # Radar parameters
        self.configured = False 
        self.frame_data = None 

    ############### SERIAL HANDLER ###############
    def openPort(self) -> None:
        if(self.port is None):
            print("Please specify a Serial Port")
            return
        # Attempt to open serial port 
        try:
            self.serial_handler = serial.Serial(self.port, self.baud_rate, timeout=self.serial_timeout)
            print("Success openning port")
        except serial.SerialException as ex: 
            print(f"Failed to open serial port: {ex}")


    def readBytes(self, bytes:int) -> str:
        try:
            data = self.serial_handler.read(bytes)
            return data
        except serial.SerialException as ex:
            print(f"ERROR Reading: {ex}")


    def closePort(self) -> None:
        if self.serial_handler and self.serial_handler.is_open:
            self.serial_handler.close()
            print("Port closed")


    ########### RADAR FUNCTIONS & PARSE ###########
    def sendConfig(self, config_file_path):
        if not self.configured:
            with open(config_file_path, 'r') as cfg_file:
                for line in cfg_file:
                    # Skip empty lines or comments
                    if line.strip() == '' or line.startswith('%'):
                        continue
                    
                    # Send each line to the radar sensor
                    # print(f"Sending command: {line.strip()}")
                    self.serial_handler.write((line.strip() + '\n').encode())
                    time.sleep(0.1)  # Short delay for the sensor to process the command

            print("Succesful configuration\n")
            self.configured = True
        else:
            print("Radar was already configured, please reset")
    

    # Use this function to get the specified number of bytes from serial port in a binary format 
    def readAndParseBytes(self, bytes:int):
        data = self.readBytes(bytes)
        byteVec = np.frombuffer(data, dtype='uint8')
        return byteVec
    

    # Use this function at the start of data reception to start receiving a frame 
    def validateMagicWord(self) -> bool:
        if self.serial_handler.is_open:
            UART_MAGIC_WORD = bytes(b'\x02\x01\x04\x03\x06\x05\x08\x07')  # This sequence defines the start of a data packet 
            magic_byte = self.readAndParseBytes(1)
            index = 0
            self.frame_data = bytearray(b'')

            while(1):
                # Condition that meets if device is not transmiting any data
                # print(magic_byte) # Debugging 
                if len(magic_byte) < 1:
                    print("No data is being received, check configuration and restart sensor")
                    magic_byte = self.readAndParseBytes(1)
                # When byte matches store and increase index 
                elif magic_byte[0] == UART_MAGIC_WORD[index]:
                    index += 1
                    self.frame_data.append(magic_byte[0])
                    if index >= 8:
                        print(self.frame_data)
                        print("Magic verification passed succesfully\n")
                        break
                    magic_byte = self.readAndParseBytes(1)   # Read a byte from the serial port 
                # Otherwise
                else:
                    if index == 0:
                        magic_byte = self.readAndParseBytes(1)
                    index = 0
                    self.frame_data = bytearray(b'')     
        else:
            print("Radar is not sending info yet, configure and connect")
            return False 


    # Call this function immediatly after validating the magic word, as a frame will be transmitted 
    def getSensorFrame(self):
        # First 4 bytes tell us the radar version
        version = self.readAndParseBytes(4)
        self.frame_data += bytearray(version)
        # Next 4 bytes are length from header 
        length_bytes = self.readAndParseBytes(4)
        self.frame_data += bytearray(length_bytes)
        frame_length = int.from_bytes(length_bytes, byteorder="little")  # Get the length of a frame in bytes 
        frame_length -= 16  # Useful frame bytes (total - version(4) - length(4) - magic_word(8))

        # Read the remaining bytes of the frame 
        self.frame_data += bytearray(self.readAndParseBytes(frame_length))


    def parseSensorFrame(self):
        # Constants to parse the data (which is in TLV [Type, length, value] format)
        header_struct = '-                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  '  # Q = 8-byte int, 8I = 8 consecutive 4-byte int
        frame_header_length = struct.calcsize(header_struct)
        tlv_header_length = 8

        # Checksum to check message integrity
        total_len_check = 0

        # Read in frame Header
        try:
            magic, version, totalPacketLen, platform, frameNum, timeCPUCycles, numDetectedObj, numTLVs, subFrameNum = struct.unpack(header_struct, self.frame_data[:frame_header_length])
            print(f"Magic: {hex(magic)}")
            print(f"Version: {version}")
            print(f"Packet length: {totalPacketLen}")
            print(f"Platform: {platform}")
            print(f"Frame number: {frameNum}")
            print(f"Time CPU cycles: {timeCPUCycles}")
            print(f"Detected Objects: {numDetectedObj}")
            print(f"Number TLVs: {numTLVs}")
            print(f"Sub-frame number: {subFrameNum}")
        except Exception as ex:
            print(f'Error: Could not read frame header -> {ex}')

        # Check message integrity 
        self.frame_data = self.frame_data[frame_header_length:]
        total_len_check += frame_header_length
        # Each point has the following: X, Y, Z, Doppler, SNR, Noise, Track index
        # point_cloud = np.zeros((numDetectedObj, 7), np.float64)  # One matrix for each object detected 
        # point_cloud[:, 6] = 255

        # Find and parse all TLV's received 

        # Decode TLV Header
        def tlvHeaderDecode(data):
            tlvType, tlvLength = struct.unpack('2I', data)   # 2 consecutive 4-byte int
            return tlvType, tlvLength



def main():
    try:
        radar = RadarHandler(port="COM10")
        radar.openPort()
        # Configure radar
        radar.sendConfig("C:/Users/jorgl/OneDrive/Escritorio/Park_Omnilooker/Radar Tools/python/Configs/PresenceDetect.cfg")
        # Start receiving 
        radar.validateMagicWord()
        radar.getSensorFrame()
        radar.parseSensorFrame()

        radar.closePort()

    except Exception as ex:
        print("Error: " + str(ex))
        radar.closePort()


if __name__ == "__main__":
    main()