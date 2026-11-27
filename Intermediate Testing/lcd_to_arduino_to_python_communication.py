import serial
import time

# ================= CONFIGURATION =================
# Windows example: "COM3" | Linux/Mac example: "/dev/ttyUSB0" or "/dev/ttyACM0"
COM_PORT = "COM3"
BAUD_RATE = 115200

# Optional mapping of DWIN button return values to grocery items
BUTTON_TO_ITEM = {
    1: "milk",
    2: "bread",
    3: "apples",
    4: "coffee",
}

def main():
    print(f"Connecting to Arduino on {COM_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino auto-reset upon serial connection
        print("Connected! Tap any button on the DWIN LCD screen...\n")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {COM_PORT}: {e}")
        return

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                print(f"[RAW FROM ARDUINO] {line}")

                # Parse structured DWIN event: "DWIN_EVENT|VP:0x1000|VAL:1"
                if line.startswith("DWIN_EVENT"):
                    parts = line.split("|")
                    vp_str = parts[1].split(":")[1]
                    val = int(parts[2].split(":")[1])

                    item = BUTTON_TO_ITEM.get(val, f"Unknown Button ({val})")
                    print(f"  --> Verified Touch! VP Address: {vp_str} | Value: {val} | Mapped Item: {item}\n")

    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
