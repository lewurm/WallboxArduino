PROJECT:=WallboxArduino
FQBN:=arduino:renesas_uno:unor4wifi
TTYDEV:=/dev/ttyACM0

.PHONY: compile upload serial

all: compile upload serial

compile:
	arduino-cli --log compile --fqbn $(FQBN) $(PWD)/$(PROJECT).ino

upload:
	arduino-cli --log upload -p $(TTYDEV) --fqbn $(FQBN) $(PWD)/$(PROJECT).ino

serial:
	arduino-cli monitor -p $(TTYDEV) -c baudrate=115200
