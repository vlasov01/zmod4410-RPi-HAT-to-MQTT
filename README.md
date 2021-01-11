# zmod4410-RPi-HAT-to-MQTT
ZMOD4410 Indoor Air Quality Raspberry Pi HAT HAL and MQTT client

The project was developed as part of my roadtest for element14 https://www.element14.com/community/roadTestReviews/3513/l/idt-zmod4410-indoor-air-quality-raspberry-pi-hat-review

Build requires Renesas firmware for ZMOD4410. I've used the firmware from October 19, 2020. It can be downloaded from https://www.renesas.com/us/en/products/sensor-products/gas-sensors/zmod4410-indoor-air-quality-sensor-platform
lib_iaq_2nd_gen.a and zmod4xxx.c should be copied to the this folder. Renesas header files from the firmware are expected to be located in /home/pi/zmod/dev/

MQTT development library (libmosquitto-dev) can be installed by running
sudo apt install libmosquitto-dev

Build:
g++ -I/home/pi/zmod/dev/ -o zmod2mqtt zmod2mqtt.cpp zmodhat.cpp zmod4xxx.c lib_iaq_2nd_gen.a -lmosquitto  

Run:
zmod2mqtt by default connect to MQTT boker, which should be resolved by DNS name mqtt on default port 1883.

It will publish sensor data every 10 seconds.

Bulding docker container:
docker build -t zmod2mqtt . 

Run docker container
docker run -it --rm --name my-zmod2mqtt zmod2mqtt:latest

Integration with Home Assistant 
Refer to https://www.element14.com/community/roadTestReviews/3513/l/idt-zmod4410-indoor-air-quality-raspberry-pi-hat-review
