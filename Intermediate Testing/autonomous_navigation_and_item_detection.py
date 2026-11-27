import time
from collections import deque
import cv2
import requests

# ================= CONFIGURATION =================
# Fixed IP when connected to ESP32-CAM's 'RC_Car_AP' hotspot
ESP32_IP = "192.168.4.1"
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

# Grocery Dictionary:
# - stop_pos: Open (0) cell where the robot stops
# - shelf_pos: Adjacent blocked (1) cell holding the item's ArUco tag
# - aruco_id: Tag ID from DICT_4X4_50
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


# ================= ROBOT MOVEMENT =================
def send_car_cmd(cmd, settle_delay=0.85):
    print(f"    [CMD] -> {cmd}")
    try:
        requests.get(f"{CMD_URL}{cmd}", timeout=2.0)
    except requests.RequestException as e:
        print(f"    [ERROR] Connection failed (Are you connected to RC_Car_AP?): {e}")
    time.sleep(settle_delay)


def calculate_heading(from_cell, to_cell):
    dr = to_cell[0] - from_cell[0]
    dc = to_cell[1] - from_cell[1]
    if dr == -1:
        return 0  # North
    if dc == 1:
        return 1  # East
    if dr == 1:
        return 2  # South
    if dc == -1:
        return 3  # West
    return current_heading


def rotate_to(target_heading):
    global current_heading
    diff = (target_heading - current_heading) % 4
    if diff == 1:
        send_car_cmd("R")
    elif diff == 2:
        send_car_cmd("R")
        send_car_cmd("R")
    elif diff == 3:
        send_car_cmd("L")
    current_heading = target_heading


def execute_path(path):
    global current_pos
    for next_cell in path[1:]:
        desired_heading = calculate_heading(current_pos, next_cell)
        rotate_to(desired_heading)
        send_car_cmd("F")
        current_pos = next_cell
        print(f"  Moved to cell {current_pos}")


# ================= ARUCO CAMERA VERIFICATION =================
def scan_shelf_aruco(expected_id, timeout_sec=15):
    print(f"  [CAM] Opening stream to scan for ArUco ID #{expected_id}...")
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
                print(f"  [FOUND] ArUco tag #{expected_id} verified!")
                detected = True
                cv2.imshow("ESP32-CAM ArUco Scanner", frame)
                cv2.waitKey(1500)
                break

        cv2.imshow("ESP32-CAM ArUco Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return detected


# ================= MAIN WORKFLOW =================
def main():
    print("=== Autonomous Grocery Car (Arduino + ESP32-CAM AP Mode) ===")
    print("Ensure your PC is connected to Wi-Fi: 'RC_Car_AP'\n")
    print("Available items:", ", ".join(GROCERY_ITEMS.keys()))
    user_entry = input("Enter grocery items (comma-separated): ")
    requested_items = [x.strip().lower() for x in user_entry.split(",") if x.strip()]

    if not requested_items:
        print("No items entered. Exiting.")
        return

    for item in requested_items:
        if item not in GROCERY_ITEMS:
            print(f"\n[SKIP] '{item}' not found in grocery dictionary.")
            continue

        info = GROCERY_ITEMS[item]
        stop_pos = info["stop_pos"]
        shelf_pos = info["shelf_pos"]
        aruco_id = info["aruco_id"]

        print(f"\n--- Task: Fetch '{item.upper()}' (Aisle {stop_pos}, Tag #{aruco_id}) ---")
        path = get_shortest_safe_path(AISLE_MAP, current_pos, stop_pos)

        if not path:
            print(f"[ERROR] No safe route to {item} at {stop_pos}.")
            continue

        print(f"  Planned safe path: {path}")
        execute_path(path)

        # Rotate front camera toward the blocked shelf cell
        shelf_direction = calculate_heading(stop_pos, shelf_pos)
        rotate_to(shelf_direction)

        if scan_shelf_aruco(aruco_id):
            print(f"  [OK] '{item}' verified. Moving to next item.")
        else:
            print(f"  [WARN] Timed out scanning for '{item}' (Tag #{aruco_id}).")

    # Return to docking station (0, 0)
    if current_pos != STATION_POS:
        print(f"\n--- Returning to Docking Station {STATION_POS} ---")
        return_path = get_shortest_safe_path(AISLE_MAP, current_pos, STATION_POS)
        if return_path:
            print(f"  Return path: {return_path}")
            execute_path(return_path)
            rotate_to(STATION_HEADING)
            print("=== Safely docked at station ===")
        else:
            print("[ERROR] Path back to station is blocked!")


if __name__ == "__main__":
    main()
