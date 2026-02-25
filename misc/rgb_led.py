import sys
import time
import board
import neopixel
import os
import signal
import subprocess
import glob


# Logging
LOG_FILE = "/tmp/rgb_led.log"
LOG_ROTATE_LIMIT = 3


def rotate_logs():
    # Get existing rotated logs, sorted oldest first
    logs = sorted(glob.glob("/tmp/rgb_led.log.*"))

    # Remove oldest logs beyond the limit
    while len(logs) >= LOG_ROTATE_LIMIT:
        os.remove(logs[0])
        logs.pop(0)

    # Rename current log to next rotation
    if os.path.exists(LOG_FILE):
        i = 1
        while os.path.exists(f"{LOG_FILE}.{i}"):
            i += 1
        os.rename(LOG_FILE, f"{LOG_FILE}.{i}")

    # Cleanup again after renaming
    logs = sorted(glob.glob("/tmp/rgb_led.log.*"))
    while len(logs) > LOG_ROTATE_LIMIT:
        os.remove(logs[0])
        logs.pop(0)


def log(msg):
    timestamp = time.strftime("[%Y-%m-%d %H:%M:%S]")
    try:
        if os.path.exists(LOG_FILE) and os.path.getsize(LOG_FILE) > 50 * 1024:
            rotate_logs()
        with open(LOG_FILE, "a") as f:
            f.write(f"{timestamp} {msg}\n")
    except Exception:
        pass


# PID management (to ensure only one instance runs)
PID_FILE = "/tmp/rgb_led.pid"


def is_rgb_led_process(pid):
    try:
        # Check if the process with `pid` is actually running rgb_led.py
        out = subprocess.check_output(["ps", "-p", str(pid), "-o", "cmd="])
        return "rgb_led.py" in out.decode()
    except subprocess.CalledProcessError:
        return False


def kill_existing_instances():
    # Try to kill existing process from PID file (if valid)
    if os.path.exists(PID_FILE):
        try:
            with open(PID_FILE, "r") as f:
                old_pid = int(f.read())
            if is_rgb_led_process(old_pid):
                os.kill(old_pid, signal.SIGTERM)
                log(f"Killed previous rgb_led.py instance (PID {old_pid})")
        except Exception as e:
            log(f"Error killing PID from file: {e}")
        os.remove(PID_FILE)

    # Additionally, kill any leftover processes just in case
    # try:
    #     subprocess.call(["pkill", "-f", "rgb_led.py"])
    # except Exception as e:
    #     log(f"Error using pkill fallback: {e}")


# Ensure only one instance is running
kill_existing_instances()

# Double-check PID file was removed
if os.path.exists(PID_FILE):
    print("Another instance might be starting. Exiting.")
    sys.exit(1)

# Register this instance
with open(PID_FILE, "w") as f:
    f.write(str(os.getpid()))

# Original code
pixel_pin = board.D12       # GPIO12
num_pixels = 10             # CHANGE to match your strip
ORDER = neopixel.GRB        # Most common strip order (can also try RGB)

# Initialize NeoPixel
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=1.0, auto_write=False, pixel_order=ORDER
)


def wheel(pos):
    # Generate rainbow colors across 0–255
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)


def rainbow_cycle(wait=0.01):
    # Animate rainbow cycle across all pixels
    while True:
        for j in range(255):
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels // 2) + j
                pixels[i] = wheel(pixel_index & 255)
            pixels.show()
            time.sleep(wait)


def test_rgb(wait=1.0):
    # Test RGB colors
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    for color in colors:
        pixels.fill(color)
        pixels.show()
        time.sleep(wait)


def stop_signal(wait=0.025):
    # Breathing effect for stop signal
    while True:
        for i in range(0, 256, 5):
            pixels.fill((255, 0, 0))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)
        for i in range(255, -1, -5):
            pixels.fill((255, 0, 0))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)


def shutdown_signal(wait=0.001):
    # Dimming effect to middle for shutdown
    pixels.fill((255, 0, 0))
    pixels.show()
    for i in range(num_pixels // 2):
        for j in range(255, -1, -1):
            pixels[i] = (j, 0, 0)
            pixels[num_pixels - i - 1] = (j, 0, 0)
            pixels.show()
            time.sleep(wait)


def automate_signal(wait=0.025):
    # Breathing effect for automate signal
    while True:
        for i in range(32, 256, 5):
            pixels.fill((0, 255, 0))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)
        for i in range(255, 32, -5):
            pixels.fill((0, 255, 0))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)


def manual_signal(wait=0.025):
    # Breathing effect for manual signal
    while True:
        for i in range(32, 256, 5):
            pixels.fill((0, 0, 255))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)
        for i in range(255, 32, -5):
            pixels.fill((0, 0, 255))
            pixels.brightness = i / 255.0
            pixels.show()
            time.sleep(wait)


def complete_signal(wait=0.5):
    # 5 blinks for complete signal
    for _ in range(5):
        pixels.fill((0, 255, 0))
        pixels.show()
        time.sleep(wait)
        pixels.fill((0, 0, 0))
        pixels.show()
        time.sleep(wait)


# Run stuff
try:
    if len(sys.argv) > 1:
        mode = sys.argv[1].lower()

        if mode == "startup":
            rainbow_cycle()
        elif mode == "shutdown":
            shutdown_signal()
        elif mode == "stop":
            stop_signal()
        elif mode == "auto":
            automate_signal()
        elif mode == "manual":
            manual_signal()
        elif mode == "complete":
            complete_signal()
        elif mode == "end":
            pass    # No action needed, just exit
        else:
            log(f"Unknown mode: {mode}, defaulting test")
            test_rgb()
            log(
                "Usage: sudo python3 rgb_led.py [startup|shutdown|stop|auto|manual|complete|end]")
    else:
        log(
            "Usage: sudo python3 rgb_led.py [startup|shutdown|stop|auto|manual|complete|end]")
except Exception as e:
    log(f"An error occurred: {e}")
finally:
    pixels.fill((0, 0, 0))
    pixels.show()
    try:
        if os.path.exists(PID_FILE):
            with open(PID_FILE, "r") as f:
                if f.read().strip() == str(os.getpid()):
                    os.remove(PID_FILE)
    except Exception as e:
        log(f"Error during PID cleanup: {e}")
