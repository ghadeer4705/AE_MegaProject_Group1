import argparse
import math
import time
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil

OBSTACLES = [
    {"lat": 38.3140964, "lon": -76.5425000, "radius": 5, "height": 100},
    {"lat": 38.3179307, "lon": -76.5473377, "radius": 5, "height": 100},
    {"lat": 38.315784,  "lon": -76.5485000, "radius": 5, "height": 100},
]

COPTER_MODE_IDS = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "CIRCLE": 7, "LAND": 9}

SAFETY_BUFFER_M = 5      # extra margin added to each obstacles radius
CHECK_PERIOD_S = 1.0     # how often to check distance to obstacles
EARTH_RADIUS_M = 6378137.0


def ground_distance_m(lat1, lon1, lat2, lon2): # turns two points to a distance in meters
    d_lat = math.radians(lat2 - lat1) # haversize formula
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS_M * c


def offset_position(lat, lon, bearing_deg_, distance_m):
    bearing = math.radians(bearing_deg_) # returns lat and lon
    ang_dist = distance_m / EARTH_RADIUS_M # given a starting point, a direction, and a distance
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)

    lat2 = math.asin(math.sin(lat1) * math.cos(ang_dist) + math.cos(lat1) * math.sin(ang_dist) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(ang_dist) * math.cos(lat1), math.cos(ang_dist) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def bearing_deg(lat1, lon1, lat2, lon2): # compass direction between two points
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)
    x = math.sin(d_lon) * math.cos(phi2)
    y = (math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def read_mission_file(path): # parses waypoints file
    items = []
    with open(path, "r") as f:
        lines = f.readlines()

    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError("Not a recognized QGC WPL waypoint file")

    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            continue
        seq = int(parts[0])
        frame = int(parts[2])
        command = int(parts[3])
        p1, p2, p3, p4 = (float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]))
        lat = float(parts[8])
        lon = float(parts[9])
        alt = float(parts[10])

        if seq == 0:
            continue 

        items.append({
            "frame": frame, "command": command,
            "p1": p1, "p2": p2, "p3": p3, "p4": p4,
            "lat": lat, "lon": lon, "alt": alt,
        })
    return items

def upload_mission(vehicle, mission_items): # uploads mission to the simulated flight controller
    cmds = vehicle.commands
    cmds.clear() # clear prev mission
    cmds.upload()
    time.sleep(1)

    for item in mission_items:
        cmds.add(Command(
            0, 0, 0,
            item["frame"], item["command"], 0, 0,
            item["p1"], item["p2"], item["p3"], item["p4"],
            item["lat"], item["lon"], item["alt"],
        ))
    cmds.upload()
    print(f"Uploaded {len(mission_items)} mission items.")

def set_mode(vehicle, name, timeout=5):
    vehicle.mode = VehicleMode(name)
    t0 = time.time()
    while vehicle.mode.name != name and time.time() - t0 < timeout:
        time.sleep(0.2)
    if vehicle.mode.name == name:
        return

    print(f"  normal mode change to {name} not accepted, using legacy SET_MODE")
    master = vehicle._master
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        COPTER_MODE_IDS[name])
    t0 = time.time()
    while vehicle.mode.name != name and time.time() - t0 < timeout:
        time.sleep(0.2)
    if vehicle.mode.name != name:
        raise RuntimeError(f"Could not switch to {name} (still in {vehicle.mode.name})")

def arm_and_takeoff(vehicle, target_altitude, max_attempts=5):
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(1) # waits for is armable

    print("Waiting for 3D GPS fix...")
    while vehicle.gps_0.fix_type < 3:
        time.sleep(1) # wait for 3D GPS fix

    print("Letting the autopilot finish initialising...")
    time.sleep(10)

    vehicle.parameters["ARMING_CHECK"] = 0 # disables arming check

    for attempt in range(1, max_attempts + 1):
        print(f"Arm + takeoff attempt {attempt}/{max_attempts}")
        set_mode(vehicle, "GUIDED") # sets mode to guided
        vehicle.armed = True # armed = true

        t0 = time.time()
        while not vehicle.armed and time.time() - t0 < 10:
            time.sleep(0.5)
        if not vehicle.armed:
            print("  did not arm, retrying")
            continue

        vehicle.simple_takeoff(target_altitude) 

        t0 = time.time()
        while vehicle.armed and time.time() - t0 < 60: # checks that drone reached 95% within 60s
            alt = vehicle.location.global_relative_frame.alt
            print(f"  altitude: {alt:.1f} m")
            if alt >= target_altitude * 0.95:
                print("Reached target altitude")
                return
            time.sleep(1)

        print("  disarmed or stalled before reaching altitude, retrying")

    raise RuntimeError("Could not take off after several attempts")

def closest_obstacle(vehicle):
    loc = vehicle.location.global_relative_frame
    best_obs, best_dist = None, None
    for obs in OBSTACLES:
        dist = ground_distance_m(loc.lat, loc.lon, obs["lat"], obs["lon"])
        if best_dist is None or dist < best_dist:
            best_obs, best_dist = obs, dist
    return best_obs, best_dist # returns closest obstacle

def divert_around(vehicle, obstacle):
    print(f"!! Too close to obstacle at ({obstacle['lat']}, {obstacle['lon']}) - diverting")

    resume_wp = vehicle.commands.next      
    resume_mode = vehicle.mode.name # saves mode to return to 
    set_mode(vehicle, "GUIDED") # switches to guided

    loc = vehicle.location.global_relative_frame
    hold_alt = loc.alt

    away_bearing = bearing_deg(obstacle["lat"], obstacle["lon"], loc.lat, loc.lon)
    clear_distance = obstacle["radius"] + SAFETY_BUFFER_M + 10  
    target_lat, target_lon = offset_position( # calculates bearing from centre of obstacle to drone and computes target point
        obstacle["lat"], obstacle["lon"], away_bearing, clear_distance)

    target = LocationGlobalRelative(target_lat, target_lon, hold_alt)
    vehicle.simple_goto(target)

    while True:
        obs, dist = closest_obstacle(vehicle) # waits till drone is clear of obstacle
        if dist is None or dist > obs["radius"] + SAFETY_BUFFER_M:
            break
        time.sleep(CHECK_PERIOD_S)

    print(f"Clear of obstacle, resuming ({resume_mode})")
    set_mode(vehicle, resume_mode) # switches back to og mode and wp
    if resume_mode == "AUTO":
        vehicle.commands.next = resume_wp

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="127.0.0.1:14551",
                        help="MAVLink connection string to the SITL vehicle")
    parser.add_argument("--mission", default="part1_waypoints_2.waypoints",
                        help="Path to the .waypoints file exported from Mission Planner")
    parser.add_argument("--takeoff-alt", type=float, default=50.0,
                        help="Altitude (m) to take off to before starting AUTO mode")
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}")
    vehicle = connect(args.connect, wait_ready=True) # connects vehicle to MAVlink

    mission_items = read_mission_file(args.mission)
    upload_mission(vehicle, mission_items)

    arm_and_takeoff(vehicle, args.takeoff_alt)

    print("Starting mission")
    vehicle.commands.next = 0
    set_mode(vehicle, "AUTO")

    total_wp = len(mission_items)
    last_status = None
    try:
        while vehicle.armed:
            mode = vehicle.mode.name

            if mode in ("AUTO", "RTL"):
                obs, dist = closest_obstacle(vehicle) # checks if obstacle near and diverts
                if obs is not None and dist <= obs["radius"] + SAFETY_BUFFER_M:
                    divert_around(vehicle, obs)

            status = (vehicle.mode.name, vehicle.commands.next)
            if status != last_status:
                alt = vehicle.location.global_relative_frame.alt
                print(f"  mode={status[0]}  mission item {status[1]}/{total_wp}  alt={alt:.1f}m")
                last_status = status
            time.sleep(CHECK_PERIOD_S)

        print("Landed and disarmed - mission complete")
    except KeyboardInterrupt:
        print("Interrupted by user")

    print("Closing vehicle connection")
    vehicle.close()

if __name__ == "__main__":
    main()