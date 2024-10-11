# Add folder to path (to avoid trouble when importing modules)
import sys
sys.path.append('/home/squirreldj/Park_OmniLooker')

import numpy as np
from flask import Flask, jsonify
from Radar_Tools.RadarData import RadarHandler
from Environment_Sensor.EnvironmentSensor import Environment_Sensor

app = Flask(__name__)


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
        env_sensor.getAccelerations()
        env_sensor.getOrientation()      
        
        output = convert_to_serializable([env_sensor.env_data, env_sensor.gyro_data, env_sensor.accel_data])
        return jsonify(output), 200
        
    except Exception as ex:
        return jsonify({"error": str(ex)}), 500 


# TEST
if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True) 
