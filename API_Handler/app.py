# Add folder to path (to avoid trouble when importing modules)
import sys
sys.path.append('/home/squirreldj/Park_OmniLooker')

import threading
import numpy as np
from flask import Flask, jsonify
from Radar_Tools.RadarData import RadarHandler
from Environment_Sensor.EnvironmentSensor import Environment_Sensor

app = Flask(__name__)

display_stop = threading.Event()  # Store the state of environment sensor display 


''' Convert dictionaries to serializable items to be able to jsonify '''
def convert_to_serializable(data):
    """Convert NumPy arrays to lists recursively."""
    if isinstance(data, dict):
        return {key: convert_to_serializable(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [convert_to_serializable(item) for item in data]
    elif isinstance(data, np.ndarray):
        return data.tolist()  # Convert ndarray to list
    else:
        return data  # Return the original data if it's already serializable


''' Get the point cloud data from the IWRL6432BOOST Radar '''
@app.route('/radar', methods=['GET'])
def getPointCloud():
    try:
        config_path = "../Radar_Tools/Configuration_Files/PresenceDetect.cfg"
        radar = RadarHandler(port="/dev/ttyACM0")
        radar.openPort()
        radar.sendConfig(config_file_path=config_path)
        valid_word = radar.validateMagicWord()
        
        if valid_word:
            radar.getSensorFrame()
            radar.output_dict = radar.parseSensorFrame()
            
        # Convert radar.output_dict to a serializable format
        serializable_output = convert_to_serializable(radar.output_dict)
        return jsonify(serializable_output), 200
    
    except Exception as ex:
        return jsonify({"error": str(ex)}), 500 


''' Environment sensor data gather '''
@app.route('/env_sensor', methods=['GET'])
def getEnvironmentData():
    try:
        env_sensor = Environment_Sensor()
        env_sensor.startSensors()
        env_sensor.getEnvironmentData()
        env_sensor.getIMU_Data()
        
        output = {
            # Environment variables 
            "Temperature C" : env_sensor.env_data[0],
            "Humidity %"     : env_sensor.env_data[1],
            "Pressure hPa"   : env_sensor.env_data[2],
            "Lux"            : env_sensor.env_data[3],
            "UV uW/cm2"      : env_sensor.env_data[4],
            "VOC ppm"        : env_sensor.env_data[5],
            # IMU values
            "Acceleration" : env_sensor.accel_data,
            "Orientation"  : env_sensor.gyro_data,
            "Magnetic field" : env_sensor.magnetometer_data
        }
        formatted_output = convert_to_serializable(output)
        return jsonify(formatted_output), 200
        
    except Exception as ex:
        return jsonify({"error": str(ex)}), 500 


# /////////  OLED DISPLAY HANDLER  /////////
def updateDisplay():
    if not display_stop:
        pass 

@app.route('/start_oled', methods=['GET'])
def startOLED():
    global display_thread 
    if display_thread is None or not display_thread.is_alive():
        display_stop.clear()  # Reset the event
        display_thread = threading.Thread(target=updateDisplay)
        display_thread.daemon = True  # Allow thread to exit when the main program exits
        display_thread.start()
        return jsonify({"status": "Display started"}), 200
    else:
        return jsonify({"status": "Display update already running"}), 400
    

# TEST
if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True) 
