import socket
import time
import math
import re

# ── 39N airport reference ──────────────────────────────────────────────────
AIRPORT_LAT = 40.3990
AIRPORT_LON = -74.6543

# ── Burst center: x0=-3657m (west), y0=-305m (south) of airport ───────────
EARTH_R = 6_378_137.0
CENTER_LAT = AIRPORT_LAT + (-305  / EARTH_R) * (180 / math.pi)
CENTER_LON = AIRPORT_LON + (-3657 / EARTH_R) * (180 / math.pi) / math.cos(math.radians(AIRPORT_LAT))

print(f"Burst center: lat={CENTER_LAT:.5f}, lon={CENTER_LON:.5f}")

# ──────────────────────────────────────────────────────────────────────────

def latlon_to_xy(lat, lon):
    dlat = math.radians(lat - CENTER_LAT)
    dlon = math.radians(lon - CENTER_LON)
    x = EARTH_R * dlon * math.cos(math.radians(CENTER_LAT))
    y = EARTH_R * dlat
    return x, y

def microburst_wind(Xe, Ye, Ze):
    c1, c2 = -0.15, -3.2175
    z_m  = 394  / 3.281
    u_m  = 98.4 / 3.281
    r_p  = 3678.5 / 3.281
    alpha = 2

    z = max(-Ze, 0)
    r = math.sqrt(Xe**2 + Ye**2)

    if r > 2.5 * r_p or z > 2.5 * z_m:
        return 0.0, 0.0, 0.0

    lam = 2*u_m / (r_p * (math.exp(c1) - math.exp(c2)) * math.exp(1 / (2*alpha)))
    R   = (r**(2*alpha)) / (r_p**(2*alpha))
    E   = math.exp((2 - R) / (2*alpha))
    fz  = math.exp(c1*z/z_m) - math.exp(c2*z/z_m)

    u =  lam * (Xe/2) * fz * E
    v =  lam * (Ye/2) * fz * E
    w = -lam * ((z_m/c1)*(math.exp(c1*z/z_m) - 1)
              - (z_m/c2)*(math.exp(c2*z/z_m) - 1)) * (1 - R/2) * E

    return u, v, w

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 5501))
sock.settimeout(2.0)

time.sleep(0.3)
try:
    sock.recv(4096)
except:
    pass

def fg_get(prop):
    for _ in range(3):
        try:
            sock.sendall(f'get {prop}\r\n'.encode())
            time.sleep(0.05)
            raw = sock.recv(1024).decode()
            m = re.search(r"'([^']+)'", raw)
            if m:
                return float(m.group(1))
            m = re.search(r'[-+]?\d+\.?\d*', raw)
            if m:
                return float(m.group())
        except:
            time.sleep(0.5)
    return 0.0

def fg_set(prop, val):
    sock.sendall(f'set {prop} {val:.4f}\r\n'.encode())
    time.sleep(0.01)
    try:
        sock.recv(256)
    except:
        pass

print("Microburst active. Ctrl-C to stop.")
try:
    while True:
        lat    = fg_get('/position/latitude-deg')
        lon    = fg_get('/position/longitude-deg')
        alt_ft = fg_get('/position/altitude-ft')

        alt_m = alt_ft * 0.3048
        x_east, y_north = latlon_to_xy(lat, lon)
        Ze = -alt_m

        u, v, w = microburst_wind(x_east, y_north, Ze)

        r = math.sqrt(x_east**2 + y_north**2)
        print(f"r={r:.0f}m  alt={alt_m:.0f}m  u={u:.1f} v={v:.1f} w={w:.1f} m/s")

        fg_set('/environment/wind-from-north-fps', -v * 3.28084)
        fg_set('/environment/wind-from-east-fps',  -u * 3.28084)
        fg_set('/environment/wind-from-down-fps',   w * 3.28084)

        time.sleep(0.05)   # 20 Hz

except KeyboardInterrupt:
    print("Stopped.")
finally:
    sock.close()
