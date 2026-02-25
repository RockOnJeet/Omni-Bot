import RPi.GPIO as GPIO
import time
import sys

# Define the pin where the buzzer is connected
BUZZER_PIN = 18  # Use GPIO 18, you can change this if needed

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)


def beep(duration):
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, GPIO.LOW)


def play_sequence(durations, pauses):
    for i in range(len(durations)):
        beep(durations[i])
        time.sleep(pauses[i])


def startup_chime():
    # 4-beep arming sound
    durations = [0.15, 0.15, 0.15, 0.5]
    pauses = [0.08, 0.08, 0.75, 0.1]
    play_sequence(durations, pauses)


def shutdown_chime():
    # 5-beep disarming sound
    durations = [0.5, 0.5, 0.5, 0.125, 0.125]
    pauses = [0.45, 0.45, 0.45, 0.075, 0.075]
    play_sequence(durations, pauses)


def error_chime():
    # SOS in Morse code
    durations = [0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 0.1, 0.1, 0.1]
    pauses = [0.1, 0.1, 0.5, 0.1, 0.1, 0.5, 0.1, 0.1, 0.5]
    play_sequence(durations, pauses)


def single_beep():
    # Single beep
    beep(0.15)


try:
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

        if mode == "startup":
            startup_chime()
        elif mode == "shutdown":
            shutdown_chime()
        elif mode == "error":
            error_chime()
        elif mode == "beep":
            single_beep()
        else:
            print(f"Unknown mode: {mode}")
            print("Usage: python3 buzzer.py [startup|shutdown|error|beep]")
    else:
        print("Usage: python3 buzzer.py [startup|shutdown|error|beep]")
except Exception as e:
    print(f"An error occurred: {e}")

finally:
    GPIO.cleanup()
