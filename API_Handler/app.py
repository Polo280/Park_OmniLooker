# Add folder to path (to avoid trouble when importing modules)
import sys
sys.path.append('/home/squirreldj/Park_OmniLooker')

from flask import Flask, jsonify
from Radar_Tools.RadarData import RadarHandler

app = Flask(__name__)

@app.route('/api', methods=['GET'])
def getPointCloud():
    try:
        config_path = "../Radar_Tools/Configuration_Files/PresenceDetect.cfg"
        radar = RadarHandler(port="/dev/ttyACM0")
        # radar.getDataBlock(config_path="../Radar_Tools/Configuration_Files/PresenceDetect.cfg")
        radar.openPort()
        radar.sendConfig(config_file_path=config_path)
        return jsonify({"message": "Data block retrieved successfully!"}), 200
    except Exception as ex:
        return jsonify({"error": str(ex)}), 500 

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True) 
