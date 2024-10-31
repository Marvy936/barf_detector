import RPi.GPIO as GPIO
import time
from pushbullet import Pushbullet

# Pushbullet
ACCESS_TOKEN = 'PUSHBULLET TOKEN'
pb = Pushbullet(ACCESS_TOKEN)

# GPIO pins
HALL_SENSOR_PIN = 17
SOUND_SENSOR_PIN = 4

# GPIO mode
GPIO.setmode(GPIO.BCM)

# Set pins
GPIO.setup(HALL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)    
GPIO.setup(SOUND_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

def send_pushbullet_notification(title, body):
    """Pushbullet notification fuction."""
    try:
        pb.push_note(title, body)
        print("Notification sent.")
    except Exception as e:
        print("Error while sending notification:", e)

try:
    previous_hall_state = GPIO.input(HALL_SENSOR_PIN)
    previous_sound_state = GPIO.input(SOUND_SENSOR_PIN)
    while True:
        # Read HALL sensor
        hall_state = GPIO.input(HALL_SENSOR_PIN)
        if hall_state == GPIO.HIGH:
            # Magnet detected
            print("Magnet detected. Sound sensor is active.")
            # Read SOUND sensor
            sound_state = GPIO.input(SOUND_SENSOR_PIN)
            if sound_state == GPIO.HIGH:
                print("Sound detected!")
                send_pushbullet_notification("Notification!", "Dog is barking!")
            previous_sound_state = sound_state
        else:
            # Magnet not detected
            print("Magnet is not detected. Sound sensor is deactivated.")
            previous_sound_state = GPIO.LOW  # Reset SOUND sensor state

        time.sleep(0.1)
except KeyboardInterrupt:
    print("Script ended by user.")
finally:
    GPIO.cleanup()
