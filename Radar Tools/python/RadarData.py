import time 
import serial 
import numpy as np
import struct

from Parsing_Libs.tlv_defines import *
from Parsing_Libs.parseTLVs import *
from Parsing_Libs.common import *

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
        # TLV decoding functions 
        self.parserFunctions = {
            MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:                     parsePointCloudTLV,
            MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:                       parseRangeProfileTLV,
            MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR:             parseRangeProfileTLV,
            MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR:             parseRangeProfileTLV,
            MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:           parseSideInfoTLV,
            MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS:                    parseSphericalPointCloudTLV,
            MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:          parseTrackTLV,
            MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST:                     parseTrackTLV,
            MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT:           parseTrackHeightTLV,
            MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:            parseTargetIndexTLV,
            MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX:                    parseTargetIndexTLV,
            MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:                   parseCompressedSphericalPointCloudTLV,
            MMWDEMO_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE:             parseOccStateMachTLV,
            MMWDEMO_OUTPUT_MSG_VITALSIGNS:                          parseVitalSignsTLV,
            MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS:                 parsePointCloudExtTLV,
            MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6843:               parseGestureFeaturesTLV,
            MMWDEMO_OUTPUT_MSG_GESTURE_OUTPUT_PROB_6843:            parseGestureProbTLV6843,
            MMWDEMO_OUTPUT_MSG_GESTURE_CLASSIFIER_6432:             parseGestureClassifierTLV6432,
            MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION:    parseEnhancedPresenceInfoTLV,
            MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO:                 parseClassifierTLV,
            MMWDEMO_OUTPUT_MSG_SURFACE_CLASSIFICATION:              parseSurfaceClassificationTLV,
            MMWDEMO_OUTPUT_EXT_MSG_VELOCITY:                        parseVelocityTLV,
            MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO:       parseRXChanCompTLV,
            MMWDEMO_OUTPUT_MSG_EXT_STATS:                           parseExtStatsTLV,
            MMWDEMO_OUTPUT_MSG_GESTURE_FEATURES_6432:               parseGestureFeaturesTLV6432,
            MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_x432:               parseGesturePresenceTLV6432,
            MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_THRESH_x432:        parsePresenceThreshold,
            MMWDEMO_OUTPUT_EXT_MSG_STATS_BSD:                       parseExtStatsTLVBSD,
            MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST_2D_BSD:              parseTrackTLV2D,
            MMWDEMO_OUTPUT_EXT_MSG_CAM_TRIGGERS:                    parseCamTLV,
            MMWDEMO_OUTPUT_EXT_MSG_POINT_CLOUD_ANTENNA_SYMBOLS:     parseAntSymbols,
            MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES:                     parseADCSamples,
            MMWDEMO_OUTPUT_EXT_MSG_MODE_SWITCH_INFO:                parseModeSwitchTLV
        }
        # Some unsupported but valid TLVs 
        self.unusedTLVs = [
            MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,
            MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,
            MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,
            MMWDEMO_OUTPUT_MSG_STATS,
            MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,
            MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,
            MMWDEMO_OUTPUT_MSG_PRESCENCE_INDICATION,
            MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_x432,
            MMWDEMO_OUTPUT_MSG_GESTURE_PRESENCE_THRESH_x432,
            MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA,
            MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES,
            MMWDEMO_OUTPUT_EXT_MSG_QUICK_EVAL_INFO
        ]
        # Dictionary to store output data
        self.output_dict = {}

    ############### SERIAL HANDLER ###############
    def openPort(self) -> None:
        if(self.port is None):
            print("Please specify a Serial Port")
            return
        # Attempt to open serial port 
        try:
            self.serial_handler = serial.Serial(self.port, self.baud_rate, timeout=self.serial_timeout)
            # print("Success openning port")
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
                    print(f"Sending command: {line.strip()}")
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
                        print("Magic verification passed succesfully\n")
                        return True
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
        header_struct = 'Q8I'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
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
            print(f"Sub-frame number: {subFrameNum}\n")
        except Exception as ex:
            print(f'Error: Could not read frame header -> {ex}')

        # Dictionary to store result of processing 
        output_dict = {}
        output_dict['frameNum'] = frameNum
        # Check message integrity 
        self.frame_data = self.frame_data[frame_header_length:]
        total_len_check += frame_header_length
        # Each point has the following: X, Y, Z, Doppler, SNR, Noise, Track index
        output_dict['pointCloud']= np.zeros((numDetectedObj, 7), np.float64)  # One matrix for each object detected 
        output_dict['pointCloud'][:, 6] = 255

        # Find and parse all TLV's received 
        for i in range(1):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(self.frame_data[:tlv_header_length])
                print(f"type: {tlvType}")
                self.frame_data = self.frame_data[tlv_header_length:]   # Discard current TLV to not consider it on next iter
                total_len_check += tlv_header_length
            except Exception as ex:
                print(f'Error: Parsing of TLVs went wrong -> {ex}')

        # If the function is defined in our code, then go to execute it 
        if tlvType in self.parserFunctions:
            self.parserFunctions[tlvType](self.frame_data[:tlvLength], tlvLength, output_dict)
        elif tlvType in self.unusedTLVs:
            print("No function to parse TLV type: %d" % (tlvType))
        else:
            print("Invalid TLV type: %d" % (tlvType))
        
        # Go to next TLV 
        self.frame_data = self.frame_data[tlvLength:]
        total_len_check += tlvLength

        # Pad totalLenCheck to the next largest multiple of 32
        # since the device does this to the totalPacketLen for transmission uniformity
        total_len_check= 32 * math.ceil(total_len_check / 32)

        # if (total_len_check != totalPacketLen):
        #     print('Frame packet length read is not equal to totalPacketLen in frame header. Subsequent frames may be dropped.')
        #     raise ConnectionAbortedError

        return output_dict 


    # Decode TLV Header
    def tlvHeaderDecode(self, data):
        tlvType, tlvLength = struct.unpack('2I', data)   # 2 consecutive 4-byte int
        return tlvType, tlvLength


    # Radar normal workflow in a function
    def getDataBlock(self, config_path:str, ):
        self.openPort()
        self.sendConfig(config_path)
        valid_word = self.validateMagicWord()
        if valid_word:
            self.getSensorFrame()
            self.output_dict = self.parseSensorFrame()


def main():
    try:
        radar = RadarHandler(port="COM10")
        radar.getDataBlock(config_path="./Configuration_Files/PresenceDetect.cfg")
        # Write point cloud into file 
        with open("./RadarOutput.txt", 'w') as file: 
            file.write(str(radar.output_dict))
            print("Data succesfully written to output.txt")
        radar.closePort()

    except Exception as ex:
        print("Error: " + str(ex))
        radar.closePort()


if __name__ == "__main__":
    main()