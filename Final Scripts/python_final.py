import time
from collections import deque
import cv2
import requests
import serial

# ================= CONFIGURATION =================
MEGA_COM_PORT = "COM3"       # Update to your Arduino Mega USB port (e.g., "COM4" or "/dev/ttyACM0")
MEGA_BAUD_RATE = 115200

ESP32_IP = "192.168.4.1"     # Fixed IP of ESP32-CAM SoftAP
CMD_URL = f"http://{ESP32_IP}/cmd?go="
STREAM_URL = f"http://{ESP32_IP}:81/stream"

# 0 = Open walkable aisle, 1 = Blocked shelf/obstacle
AISLE_MAP = [
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
]

# Grocery Database:
# - stop_pos: Open (0) cell where the robot parks
# - shelf_pos: Adjacent blocked (1) cell containing the item & ArUco tag
# - aruco_id: ID from DICT_4X4_50
GROCERY_ITEMS = {
    "milk":   {"stop_pos": (1, 0), "shelf_pos": (1, 1), "aruco_id": 10},
    "bread":  {"stop_pos": (2, 3), "shelf_pos": (2, 2), "aruco_id": 20},
    "apples": {"stop_pos": (4, 3), "shelf_pos": (4, 4), "aruco_id": 30},
    "coffee": {"stop_pos": (4, 5), "shelf_pos": (4, 4), "aruco_id": 40},
}

STATION_POS = (0, 0)
STATION_HEADING = 1  # 0=North (-row), 1=East (+col), 2=South (+row), 3=West (-col)

current_pos = STATION_POS
current_heading = STATION_HEADING


# ================= PATHFINDING (BFS) =================
def get_shortest_safe_path(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
        return None

    queue = deque([(start, [start])])
    visited = {start}

    while queue:
        (r, c), path = queue.popleft()
        if (r, c) == goal:
            return path

        for dr, dc in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] == 0 and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append(((nr, nc), path + [(nr, nc)]))
    return None


# ================= ROBOT MOTION & ARM COMMANDS =================
def send_robot_cmd(cmd):
    """Sends command to ESP32-CAM -> Mega and blocks until Mega confirms 'DONE'."""
    print(f"    [EXEC] Command '{cmd}' -> Waiting for Mega completion...")
    try:
        resp = requests.get(f"{CMD_URL}{cmd}", timeout=20.0)
        print(f"    [ACK]  Mega replied: {resp.text.strip()}")
    except requests.RequestException as e:
        print(f"    [ERROR] Communication failed: {e}")
    time.sleep(0.15)


def calculate_heading(from_cell, to_cell):
    dr = to_cell[0] - from_cell[0]
    dc = to_cell[1] - from_cell[1]
    if dr == -1: return 0  # North
    if dc == 1:  return 1  # East
    if dr == 1:  return 2  # South
    if dc == -1: return 3  # West
    return current_heading


def rotate_to(target_heading):
    global current_heading
    diff = (target_heading - current_heading) % 4
    if diff == 1:
        send_robot_cmd("R")
    elif diff == 2:
        send_robot_cmd("R")
        send_robot_cmd("R")
    elif diff == 3:
        send_robot_cmd("L")
    current_heading = target_heading


def execute_path(path):
    global current_pos
    for next_cell in path[1:]:
        desired_heading = calculate_heading(current_pos, next_cell)
        rotate_to(desired_heading)
        send_robot_cmd("F")
        current_pos = next_cell
        print(f"  Arrived at grid cell {current_pos}")


# ================= ARUCO CAMERA SCANNER =================
def scan_shelf_aruco(expected_id, timeout_sec=15):
    print(f"  [CAM] Scanning live stream for ArUco ID #{expected_id}...")
    cap = cv2.VideoCapture(STREAM_URL)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    start_time = time.time()
    detected = False

    while (time.time() - start_time) < timeout_sec:
        ret, frame = cap.read()
        if not ret:
            continue

        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if expected_id in ids.flatten():
                print(f"  [FOUND] Verified ArUco Tag #{expected_id}!")
                detected = True
                cv2.imshow("ESP32-CAM ArUco Scanner", frame)
                cv2.waitKey(1000)
                break

        cv2.imshow("ESP32-CAM ArUco Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return detected


# ================= DWIN LCD LISTENER =================
def wait_for_lcd_order(ser):
    """Listens on Arduino Mega USB Serial for the DWIN touchscreen order."""
    print("Listening for order from DWIN 4.3\" HMI LCD (tap items, then tap START)...")
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("LCD_SELECTED:"):
                print(f"  [LCD] Item added to cart: {line.split(':')[1]}")
            elif line.startswith("ORDER:"):
                raw_items = line.split(":", 1)[1]
                items = [x.strip().lower() for x in raw_items.split(",") if x.strip()]
                print(f"\n[LCD ORDER RECEIVED] Shopping List: {items}")
                return items
        time.sleep(0.05)


# ================= MAIN WORKFLOW =================
def main():
    print("=== Autonomous Grocery Robot Master Controller ===")
    try:
        mega_serial = serial.Serial(MEGA_COM_PORT, MEGA_BAUD_RATE, timeout=1)
        time.sleep(2)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {MEGA_COM_PORT}: {e}")
        return

    try:
        while True:
            shopping_list = wait_for_lcd_order(mega_serial)

            for item in shopping_list:
                if item not in GROCERY_ITEMS:
                    print(f"[SKIP] '{item}' is not mapped in GROCERY_ITEMS.")
                    continue

                info = GROCERY_ITEMS[item]
                stop_pos = info["stop_pos"]
                shelf_pos = info["shelf_pos"]
                aruco_id = info["aruco_id"]

                print(f"\n=== Fetching '{item.upper()}' at {stop_pos} (ArUco #{aruco_id}) ===")
                path = get_shortest_safe_path(AISLE_MAP, current_pos, stop_pos)
                if not path:
                    print(f"[ERROR] No safe path to {item}.")
                    continue

                execute_path(path)

                # Face the shelf cell
                shelf_dir = calculate_heading(stop_pos, shelf_pos)
                rotate_to(shelf_dir)

                # Scan ArUco tag & trigger arm pick-and-stow sequence
                if scan_shelf_aruco(aruco_id):
                    print(f"  [ARM] Actuating robotic arm to pick '{item}' and stow in rear container...")
                    send_robot_cmd("P")
                    print(f"  [OK] '{item}' safely stowed in rear container!")
                else:
                    print(f"  [WARN] ArUco #{aruco_id} not detected. Skipping pickup.")

            # Return to home docking station (0, 0)
            if current_pos != STATION_POS:
                print(f"\n=== Order Complete. Returning to Station {STATION_POS} ===")
                home_path = get_shortest_safe_path(AISLE_MAP, current_pos, STATION_POS)
                if home_path:
                    execute_path(home_path)
                    rotate_to(STATION_HEADING)
                    print("=== Docked at Station (0, 0). Ready for next LCD order! ===\n")

    except KeyboardInterrupt:
        print("\nShutting down controller.")
    finally:
        mega_serial.close()


if __name__ == "__main__":
    main()
