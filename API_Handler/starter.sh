#!/bin/bash

# NOTE: Use this script to initialize the required modules and start the API running 

# Activate python venv 
source ../myenv/bin/activate

# Configure group permissions to use GPIO's (env sensor)
sudo chmod 666 /dev/gpiochip0
sudo chmod 666 /dev/gpiochip1

# Run API
python app.py