# Xe do line 5 mat + ESP32 + L298N  (MicroPython - nap bang Thonny)
# Chuc nang: do line (PID) + dung khi gap vat can + dung o tram + dieu khien tay (Wi-Fi)
#
# Cam bien: D1->32, D2->33, D3->25, D4->26, D5->27, VCC->3V3, GND->GND
# L298N:    ENA->22, IN1->16, IN2->17, IN3->18, IN4->19, ENB->21
# Den:      GPIO 2
# NUT BAM:  GPIO 14 -> GND (dung dien tro keo len ben trong)
# HC-SR04:  VCC->5V, GND->GND, TRIG->13, ECHO->23 (ECHO QUA CAU PHAN AP 1k+2k)
#
# Do line: gap vat can (<= AD cm) -> dung + bao app; het vat can (> AD+5 cm) -> chay tiep.
# Tram: khoi hanh truoc tram 1; dem vach 1 -> 2 -> 3 -> 1.
#   Den tram -> PHANH NHANH -> dung. Co 2 che do:
#     SM0 = tu di tiep sau TW giay (theo thoi gian)
#     SM1 = cho bam nut (GPIO 14 hoac lenh N) moi di tiep
#   Bam nut / lenh N -> doi GD ms (mac dinh 2000 = 2 giay) roi moi chay.
#   Roi tram -> DI THANG TS ms (bo qua line) de vuot vach tram, roi moi bam line.
import network
import socket
import json
import os
import time
from machine import Pin, PWM, time_pulse_us

# ---------- Cam bien ----------
sensor_pins = [Pin(p, Pin.IN) for p in (32, 33, 25, 26, 27)]
LINE_LEVEL = 1   # Doi thanh 0 neu module bi nguoc muc logic

# ---------- L298N ----------
IN1 = Pin(16, Pin.OUT)
IN2 = Pin(17, Pin.OUT)
IN3 = Pin(18, Pin.OUT)
IN4 = Pin(19, Pin.OUT)
PWM_L = PWM(Pin(22), freq=1000, duty=0)   # ENA
PWM_R = PWM(Pin(21), freq=1000, duty=0)   # ENB

INVERT_LEFT = True    # Dao chieu motor trai neu xe chay lui
INVERT_RIGHT = True   # Dao chieu motor phai

# Hieu chinh 2 motor chay thang (% cong suat, 50-100). Xe lech PHAI -> giam CL;
# xe lech TRAI -> giam CR. Doi bang lenh: CL92, CR95 ...
gain_left = 97
gain_right = 100

# ---------- Den ----------
led = Pin(2, Pin.OUT)
led.value(0)

# ---------- Nut bam di tiep tram ----------
btn = Pin(14, Pin.IN, Pin.PULL_UP)   # bam = noi GND = 0
btn_last = 1
btn_time = 0
go_flag = False       # True khi co yeu cau "di tiep" (nut hoac lenh N)
go_delay_ms = 1000    # tre sau khi bam nut roi moi chay (ms), lenh GD
go_at = None          # moc thoi gian se chay (None = chua hen)

# ---------- Sieu am HC-SR04 ----------
trig = Pin(13, Pin.OUT)
echo = Pin(23, Pin.IN)
trig.value(0)

# ---------- Nut chuyen che do ----------
AUTO_ON_CHAR = 'U'
AUTO_OFF_CHAR = 'u'

# ---------- Thong so PID ----------
Kp = 6.0
Ki = 0.0
Kd = 300.0

base_speed = 95
max_speed = 230
min_start = 80
search_speed = 110

# ---------- Phanh nhanh khi den tram ----------
brake_ms = 30        # thoi gian phanh nguoc (ms), 0 = chi phanh dien
brake_power = 200    # luc phanh nguoc (0-255)

# ---------- Toc do dieu khien tay ----------
speed_level = [100, 150, 200, 255]
manual_speed = 120
TURN_RATIO = 0.8  # Giam toc khi re; thu 0.5 - 0.7

# Trang web gui lap lenh khi giu nut; tha nut gui lenh dung.
# Khong nhan lenh moi thi dung; TO gioi han 200..2000 ms.
manual_timeout = 600
manual_deadline = None

# ---------- Che do ----------
MODE_STOP, MODE_AUTO, MODE_MANUAL = 0, 1, 2
mode = MODE_STOP

# ---------- Vat can khi do line ----------
ob_enable = True       # Bat/tat dung khi gap vat can (lenh OB1/OB0, ky tu W)
avoid_dist = 20        # cm (lenh AD)
road_state = "CLEAR"   # "CLEAR" hoac "WAIT" (dang dung cho vat can)
clear_hits = 0
ob_next_check = 0

# ---------- Tram ----------
station_wait_s = 5
station_btn_mode = False   # False = theo thoi gian, True = cho bam nut
station_waiting = False    # dang dung o tram
next_station = 1
location_state = "before_first"
last_station = 0
current_station = 0
station_visits = 0
station_deadline = None
station_armed = True
station_hits = 0
station_exit_hits = 0
station_gap_ms = 1000      # sau khi roi tram, it nhat bay nhieu ms moi nhan tram ke (lenh SG)
station_block_until = None

# ---------- Di thang sau khi roi tram ----------
straight_ms = 500    # sau khi roi tram di thang bay nhieu ms, bo qua line (lenh TS), 0 = tat
straight_until = None

mv_l = 0
mv_r = 0

last_error = 0.0
integral = 0.0
last_known_error = 0.0

last_distance = None
distance_at = None
telemetry_next = 0
motor_left = 0
motor_right = 0

# ---------- Nut bam (chong doi 50 ms) ----------
def poll_button():
    global btn_last, btn_time, go_flag
    v = btn.value()
    if v != btn_last:
        now = time.ticks_ms()
        if time.ticks_diff(now, btn_time) > 50:
            btn_time = now
            btn_last = v
            if v == 0:
                go_flag = True


# ---------- Nhat ky trang web ----------
events = []
def log_event(text):
    events.append(text)
    if len(events) > 12:
        events.pop(0)

# ---------- Motor ----------
left_braking = False
right_braking = False


def _pwm_duty(speed):
    return min(abs(speed), 255) * 1023 // 255


def set_motor_left(speed):
    global left_braking, motor_left
    motor_left = int(speed)
    speed = max(-255, min(255, int(speed)))
    if speed == 0:
        brake_left()
        return
    if left_braking:
        PWM_L.duty(0)  # Nha phanh truoc khi dat chieu quay
        left_braking = False
    speed = speed * gain_left // 100   # hieu chinh lech
    if INVERT_LEFT:
        speed = -speed
    if speed >= 0:
        IN1.value(1)
        IN2.value(0)
    else:
        IN1.value(0)
        IN2.value(1)
    PWM_L.duty(_pwm_duty(speed))


def set_motor_right(speed):
    global right_braking, motor_right
    motor_right = int(speed)
    speed = max(-255, min(255, int(speed)))
    if speed == 0:
        brake_right()
        return
    if right_braking:
        PWM_R.duty(0)  # Nha phanh truoc khi dat chieu quay
        right_braking = False
    speed = speed * gain_right // 100  # hieu chinh lech
    if INVERT_RIGHT:
        speed = -speed
    if speed >= 0:
        IN3.value(1)
        IN4.value(0)
    else:
        IN3.value(0)
        IN4.value(1)
    PWM_R.duty(_pwm_duty(speed))


def brake_left():
    global left_braking, motor_left
    motor_left = 0
    if not left_braking:
        PWM_L.duty(0)
        IN1.value(0)
        IN2.value(0)
        PWM_L.duty(1023)
        left_braking = True


def brake_right():
    global right_braking, motor_right
    motor_right = 0
    if not right_braking:
        PWM_R.duty(0)
        IN3.value(0)
        IN4.value(0)
        PWM_R.duty(1023)
        right_braking = True


def brake_motors():
    # L298N: EN=1, IN1=IN2 -> phanh dien, khong dao chieu motor.
    brake_left()
    brake_right()


def brake_fast():
    """Phanh nhanh: dao chieu 2 banh mot luc ngan (phanh nguoc) roi phanh dien giu cung."""
    if brake_ms > 0 and brake_power > 0:
        set_motor_left(-brake_power)
        set_motor_right(-brake_power)
        time.sleep_ms(brake_ms)
    brake_motors()


def stop_motors():
    # Giu phanh cho den khi co lenh chay moi, khong sleep chan Wi-Fi.
    brake_motors()


def apply_manual():
    speed = manual_speed
    if mv_l != mv_r:
        speed = int(manual_speed * TURN_RATIO)
    set_motor_left(speed * mv_l // 100)
    set_motor_right(speed * mv_r // 100)


def manual_move(l, r, use_timeout=True):
    global mode, mv_l, mv_r, manual_deadline
    global location_state, current_station, last_station
    location_state = "unknown"
    current_station = last_station = 0
    if mode == MODE_AUTO:
        led.value(0)   # thoat tu dong -> tat den
    mode = MODE_MANUAL
    mv_l = l
    mv_r = r
    if use_timeout and manual_timeout > 0:
        manual_deadline = time.ticks_add(time.ticks_ms(), manual_timeout)
    else:
        manual_deadline = None
    apply_manual()


# ---------- Cam bien ----------
def read_distance_cm():
    """Do khoang cach (cm) bang HC-SR04. Tra ve None neu khong co tin hieu."""
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    t = time_pulse_us(echo, 1, 25000)   # toi da ~4m
    if t < 0:
        return None
    d = t / 58.0
    if d < 2:        # nhieu / qua gan khong tin cay
        return None
    return d


def line_count():
    n = 0
    for p in sensor_pins:
        if p.value() == LINE_LEVEL:
            n += 1
    return n


def read_line():
    """Tra ve sai so (float) hoac None neu mat line."""
    weights = (-4, -2, 0, 2, 4)
    total = 0
    count = 0
    for i in range(5):
        if sensor_pins[i].value() == LINE_LEVEL:
            total += weights[i]
            count += 1
    if count == 0:
        return None
    return total / count


# ---------- Tram ----------
def station_detected():
    """Tram = vach ngang (ca 5 mat cung thay line), loc nhieu 2 lan lien tiep."""
    global station_armed, station_hits, station_exit_hits
    count = line_count()
    # Khoang cach toi thieu giua 2 tram: dang trong thoi gian khoa thi bo qua vach
    if station_block_until is not None:
        if time.ticks_diff(time.ticks_ms(), station_block_until) < 0:
            station_hits = 0
            return False
    if not station_armed:
        station_exit_hits = station_exit_hits + 1 if count < 5 else 0
        if station_exit_hits >= 3:
            station_armed = True
            station_exit_hits = 0
        return False
    station_hits = station_hits + 1 if count == 5 else 0
    if station_hits >= 2:
        station_armed = False
        station_hits = 0
        return True
    return False


def leave_station(reason):
    """Roi tram, di thang vuot vach tram roi chay tiep ve tram ke tiep."""
    global station_waiting, station_deadline, ob_next_check, integral, last_error, go_flag, go_at
    global station_block_until, straight_until
    global location_state, current_station
    location_state = "segment"
    current_station = 0
    station_waiting = False
    station_deadline = None
    go_flag = False
    go_at = None
    now = time.ticks_ms()
    # Di thang, bo qua line de khong re theo vach tram
    straight_until = time.ticks_add(now, straight_ms) if straight_ms > 0 else None
    # Khoa nhan tram it nhat bang thoi gian di thang
    station_block_until = time.ticks_add(now, max(station_gap_ms, straight_ms))
    ob_next_check = now
    integral = last_error = 0.0
    log_event("%s - TRAM KE TIEP %d" % (reason, next_station))


# ---------- Vat can: dung roi chay tiep ----------
def update_road(distance):
    global road_state, clear_hits, integral, last_error
    if distance is None:
        clear_hits = 0
        return
    if distance <= avoid_dist:
        clear_hits = 0
        if road_state != "WAIT":
            stop_motors()
            road_state = "WAIT"
            log_event("VAT CAN - DUNG")
        return
    if road_state == "WAIT":
        clear_hits = clear_hits + 1 if distance > avoid_dist + 5 else 0
        if clear_hits >= 3:
            road_state = "CLEAR"
            clear_hits = 0
            integral = last_error = 0.0
            log_event("DA THONG - CHAY TIEP")


def set_obstacle(enabled):
    global ob_enable, road_state, clear_hits
    ob_enable = enabled
    if not enabled:
        road_state = "CLEAR"
        clear_hits = 0


def set_station_mode(btn_mode):
    global station_btn_mode, station_deadline
    station_btn_mode = btn_mode
    if btn_mode:
        station_deadline = None
        log_event("TRAM: CHO BAM NUT (SM1)")
    else:
        log_event("TRAM: THEO THOI GIAN %d s (SM0)" % station_wait_s)


# ---------- Lenh Wi-Fi ----------
def handle_char(c):
    global mode, integral, last_error, mv_l, mv_r, manual_speed
    global ob_next_check, station_hits, road_state, clear_hits
    global go_flag, go_at, station_waiting, station_deadline, straight_until
    global location_state, current_station

    # ----- Chuyen che do tu dong -----
    if c == AUTO_ON_CHAR:
        if mode == MODE_AUTO and station_waiting:
            return
        if location_state == "station":
            location_state = "segment"
            current_station = 0
        mode = MODE_AUTO
        integral = 0.0
        straight_until = None
        last_error = 0.0
        led.value(1)   # Den sang = dang tu dong
        road_state = "CLEAR"
        clear_hits = 0
        ob_next_check = time.ticks_ms()
        station_hits = 0
        log_event("TU DONG DO LINE")
        return
    if c == AUTO_OFF_CHAR:
        mode = MODE_STOP
        station_waiting = False
        station_deadline = None
        go_at = None
        straight_until = None
        stop_motors()
        led.value(0)
        log_event("DUNG")
        return

    # ----- Dieu khien tay -----
    if c == 'F':
        manual_move(100, 100)
    elif c == 'B':
        manual_move(-100, -100)
    elif c == 'L':
        manual_move(-100, 100)
    elif c == 'R':
        manual_move(100, -100)
    elif c == 'G':
        manual_move(40, 100)
    elif c == 'I':
        manual_move(100, 40)
    elif c == 'H':
        manual_move(-40, -100)
    elif c == 'J':
        manual_move(-100, -40)
    elif c == 'S':
        if mode == MODE_MANUAL:
            mv_l = 0
            mv_r = 0
            stop_motors()
    elif c == 'D':
        if mode == MODE_AUTO:
            led.value(0)
        mode = MODE_STOP
        station_waiting = False
        station_deadline = None
        go_at = None
        straight_until = None
        stop_motors()
        log_event("DUNG HAN")
    elif c == 'W':
        set_obstacle(not ob_enable)
        log_event("Dung khi gap vat can: " + ("BAT" if ob_enable else "TAT"))
    elif c == 'M':
        set_station_mode(not station_btn_mode)
    elif c == 'N':
        if mode == MODE_AUTO and station_waiting:
            go_flag = True
        else:
            log_event("Khong o tram")
    elif c in "1234":
        manual_speed = speed_level[int(c) - 1]
        if mode == MODE_MANUAL:
            apply_manual()


def handle_param(cmd):
    """Lenh chinh thong so, ket thuc bang xuong dong. Vi du: KP20 KD14 AS130"""
    global Kp, Ki, Kd, integral, base_speed, max_speed, search_speed, manual_speed
    global station_wait_s, manual_timeout, avoid_dist, brake_ms, brake_power, go_delay_ms
    global gain_left, gain_right, station_gap_ms, straight_ms
    cmd = cmd.strip().upper()
    if len(cmd) < 3:
        log_event("Lenh sai")
        return

    key = cmd[:2]
    try:
        val = float(cmd[2:])
    except ValueError:
        log_event("Lenh sai")
        return

    def clamp(v):
        return max(0, min(255, int(v)))

    if key == "TW":
        if not (1 <= val <= 120) or val != int(val):
            log_event("TW: 1..120 giay")
            return
        station_wait_s = int(val)
        log_event("Cho moi tram: %d s" % station_wait_s)
    elif key == "SM":
        if val not in (0, 1):
            log_event("SM0 (thoi gian) / SM1 (nut)")
            return
        set_station_mode(val == 1)
    elif key == "GD":
        go_delay_ms = max(0, min(10000, int(val)))
        log_event("Tre khi bam nut = %d ms" % go_delay_ms)
    elif key == "SG":
        station_gap_ms = max(0, min(10000, int(val)))
        log_event("Khoang cach 2 tram >= %d ms" % station_gap_ms)
    elif key == "TS":
        straight_ms = max(0, min(10000, int(val)))
        log_event("Di thang sau tram = %d ms" % straight_ms)
    elif key == "CL":
        gain_left = max(50, min(100, int(val)))
        log_event("Can chinh trai = %d%%" % gain_left)
    elif key == "CR":
        gain_right = max(50, min(100, int(val)))
        log_event("Can chinh phai = %d%%" % gain_right)
    elif key == "BK":
        brake_ms = max(0, min(300, int(val)))
        log_event("Brake = %d ms" % brake_ms)
    elif key == "BP":
        brake_power = clamp(val)
        log_event("Brake power = %d" % brake_power)
    elif key == "KP":
        Kp = val
        log_event("Kp = %.2f" % Kp)
    elif key == "KI":
        Ki = val
        integral = 0.0
        log_event("Ki = %.2f" % Ki)
    elif key == "KD":
        Kd = val
        log_event("Kd = %.2f" % Kd)
    elif key == "AS":
        base_speed = clamp(val)
        log_event("Auto speed = %d" % base_speed)
    elif key == "AM":
        max_speed = clamp(val)
        log_event("Auto max = %d" % max_speed)
    elif key == "AF":
        search_speed = clamp(val)
        log_event("Search = %d" % search_speed)
    elif key == "MS":
        manual_speed = clamp(val)
        log_event("Manual speed = %d" % manual_speed)
    elif key == "TO":
        manual_timeout = max(200, min(2000, int(val)))
        log_event("Timeout = %d ms" % manual_timeout)
    elif key == "AD":
        avoid_dist = max(5, min(200, int(val)))
        log_event("Avoid dist = %d cm" % avoid_dist)
    elif key == "OB":
        if val not in (0, 1):
            log_event("OB0 / OB1")
            return
        set_obstacle(val == 1)
        log_event("OB=%d" % int(val))
    else:
        log_event("Lenh sai. Vi du: KP20 KD14 AS130 MS150")


# ---------- Wi-Fi AP va web server khong chan vong lap ----------
WIFI_SSID = 'XeDoLine-WiFi'
WIFI_PASSWORD = 'xedoline123'
CONFIG_FILE = 'xe_config.json'
server = None
ap = None
clients = []
PARAMS = {
    'KP': ('Kp', 0, 1000), 'KI': ('Ki', 0, 1000), 'KD': ('Kd', 0, 2000),
    'AS': ('base_speed', 0, 255), 'AM': ('max_speed', 0, 255),
    'AF': ('search_speed', 0, 255), 'MS': ('manual_speed', 0, 255),
    'TO': ('manual_timeout', 200, 2000), 'AD': ('avoid_dist', 5, 200),
    'OB': ('ob_enable', 0, 1), 'TW': ('station_wait_s', 1, 120),
    'SM': ('station_btn_mode', 0, 1), 'BK': ('brake_ms', 0, 300),
    'BP': ('brake_power', 0, 255), 'GD': ('go_delay_ms', 0, 10000),
    'SG': ('station_gap_ms', 0, 10000), 'TS': ('straight_ms', 0, 10000),
    'CL': ('gain_left', 50, 100), 'CR': ('gain_right', 50, 100)
}

def settings():
    return {k: globals()[v[0]] for k, v in PARAMS.items()}

def validate_settings(data):
    if not isinstance(data, dict) or not data:
        raise ValueError('Thong so rong')
    for key, value in data.items():
        if key not in PARAMS or not isinstance(value, (int, float)):
            raise ValueError('Thong so khong hop le')
        low, high = PARAMS[key][1:]
        if not low <= value <= high:
            raise ValueError(key + ': ngoai gioi han')
        if key not in ('KP', 'KI', 'KD') and value != int(value):
            raise ValueError(key + ': can so nguyen')
    merged = settings()
    merged.update(data)
    if merged['AS'] > merged['AM']:
        raise ValueError('AS phai <= AM')

def apply_settings(data):
    validate_settings(data)
    for key, value in data.items():
        handle_param(key + str(int(value) if key not in ('KP', 'KI', 'KD') else value))

def load_settings():
    try:
        with open(CONFIG_FILE) as f:
            data = json.load(f)
        apply_settings(data)
        log_event('Da nap thong so da luu')
    except (OSError, ValueError, TypeError):
        log_event('Su dung thong so mac dinh')

def start_wifi():
    global ap, server
    iface = getattr(network, 'AP_IF', None)
    if iface is None:
        iface = network.WLAN.IF_AP
    ap = network.WLAN(iface)
    ap.active(True)
    try:
        ap.config(essid=WIFI_SSID, password=WIFI_PASSWORD, authmode=3)
    except (ValueError, TypeError):
        ap.config(ssid=WIFI_SSID, key=WIFI_PASSWORD, security=3)
    ap.ifconfig(('192.168.4.1', '255.255.255.0', '192.168.4.1', '192.168.4.1'))
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', 80))
    server.listen(4)
    server.setblocking(False)
    print('Wi-Fi:', WIFI_SSID, '| Mat khau:', WIFI_PASSWORD)
    print('Mo http://192.168.4.1 tren trinh duyet')

def sample_telemetry():
    global telemetry_next, last_distance, distance_at
    now = time.ticks_ms()
    if time.ticks_diff(now, telemetry_next) >= 0:
        telemetry_next = time.ticks_add(now, 100)
        last_distance = read_distance_cm()
        distance_at = time.ticks_ms()

def status():
    return {'mode': ('DUNG', 'DO LINE', 'DIEU KHIEN TAY')[mode],
            'distance': last_distance, 'line': [p.value() == LINE_LEVEL for p in sensor_pins],
            'road': road_state, 'waiting': station_waiting, 'next_station': next_station,
            'motor': [motor_left, motor_right], 'events': events,
            'location': {'state': location_state, 'current': current_station,
                         'last': last_station, 'visits': station_visits},
            'params': settings()}

def route(method, path, body):
    if method == 'GET' and path == '/':
        return 200, 'text/html; charset=utf-8', HTML
    if method == 'GET' and path == '/api/status':
        return 200, 'application/json', json.dumps(status()).encode()
    if method != 'POST':
        return 404, 'text/plain', b'Not found'
    data = json.loads(body)
    if path == '/api/cmd':
        c = data.get('cmd', '')
        if len(c) != 1 or c not in 'UuFBLRGIHJSD1234WMN':
            raise ValueError('Lenh khong hop le')
        handle_char(c)
    elif path == '/api/params':
        if mode != MODE_STOP:
            raise ValueError('Dung xe truoc khi chinh thong so')
        apply_settings(data)
    elif path == '/api/save':
        if mode != MODE_STOP:
            raise ValueError('Dung xe truoc khi luu')
        with open(CONFIG_FILE + '.tmp', 'w') as f:
            json.dump(settings(), f)
        os.rename(CONFIG_FILE + '.tmp', CONFIG_FILE)
        log_event('Da luu thong so vao ESP32')
    else:
        return 404, 'text/plain', b'Not found'
    return 200, 'application/json', b'{"ok":true}'

def response(code, kind, body):
    return ('HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n' %
            (code, 'OK' if code == 200 else 'Error', kind, len(body))).encode() + body

def would_block(exc):
    return exc.args[0] in (11, 35, 10035)

def poll_http():
    # Moi client: socket, input, output, offset, accepted_at. Gioi han RAM/thoi gian.
    try:
        conn, addr = server.accept()
        conn.setblocking(False)
        if len(clients) >= 4:
            conn.close()
        else:
            clients.append([conn, b'', None, 0, time.ticks_ms()])
    except OSError as exc:
        if not would_block(exc):
            raise
    for item in clients[:]:
        conn = item[0]
        close = time.ticks_diff(time.ticks_ms(), item[4]) > 1500
        try:
            if not close and item[2] is None:
                try:
                    chunk = conn.recv(512)
                except OSError as exc:
                    if not would_block(exc):
                        raise
                    chunk = None
                if chunk == b'':
                    close = True
                elif chunk:
                    item[1] += chunk
                    if len(item[1]) > 4096:
                        close = True
                    elif b'\r\n\r\n' in item[1]:
                        head, body = item[1].split(b'\r\n\r\n', 1)
                        lines = head.decode().split('\r\n')
                        method, path, version = lines[0].split()
                        length = 0
                        for line in lines[1:]:
                            key, value = line.split(':', 1)
                            if key.lower() == 'content-length':
                                length = int(value.strip())
                            if key.lower() == 'transfer-encoding':
                                raise ValueError('Unsupported transfer encoding')
                        if length < 0 or length > 2048:
                            raise ValueError('Request too large')
                        if len(body) >= length:
                            try:
                                code, kind, payload = route(method, path, body[:length])
                            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                                code, kind, payload = 400, 'application/json', json.dumps({'error': str(exc)}).encode()
                            except OSError:
                                code, kind, payload = 500, 'application/json', b'{"error":"Khong luu duoc bo nho"}'
                            item[2] = response(code, kind, payload)
                            item[1] = b''
            if not close and item[2] is not None:
                try:
                    count = conn.send(item[2][item[3]:item[3] + 1024])
                    if count:
                        item[3] += count
                    close = not count or item[3] >= len(item[2])
                except OSError as exc:
                    if not would_block(exc):
                        raise
        except (OSError, ValueError, UnicodeError):
            close = True
        if close:
            conn.close()
            clients.remove(item)

HTML = b'<!doctype html><html lang="vi"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Xe d\xc3\xb2 line \xc2\xb7 Wi-Fi</title>\r\n<style>body{font:16px system-ui;background:#101827;color:#eef3ff;margin:0;padding:18px;max-width:960px;margin:auto}h1{font-size:26px}section{background:#1e293b;padding:18px;border-radius:16px;margin:16px 0}button,input,select{font:inherit;padding:12px;border:0;border-radius:9px}button{background:#38bdf8;color:#071827;cursor:pointer;margin:4px;touch-action:none}button:active{background:#fff}button.stop{background:#fb7185;font-weight:bold}button:disabled{opacity:.45}#pad{display:grid;grid-template-columns:repeat(3,1fr);max-width:390px;margin:auto;user-select:none}#pad button{min-height:58px}#fields{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:14px}label{font-size:14px;display:grid;gap:6px}input,select{width:100%;box-sizing:border-box;background:#0f172a;color:white}#stats{line-height:1.9}#lights span{display:inline-block;width:25px;height:25px;background:#475569;border-radius:50%;margin:6px}#lights .on{background:#34d399}pre{white-space:pre-wrap;font:14px system-ui}small{color:#cbd5e1}#notice{min-height:24px;color:#fbbf24}</style>\r\n<h1>Xe d\xc3\xb2 line <small>Wi-Fi</small></h1><div id="connection">\xc4\x90ang k\xe1\xba\xbft n\xe1\xbb\x91i\xe2\x80\xa6</div><p id="notice"></p>\r\n<section><h2>\xc4\x90i\xe1\xbb\x81u khi\xe1\xbb\x83n</h2><button id="auto">Ch\xe1\xba\xa1y d\xc3\xb2 line</button><button id="stop" class="stop">D\xe1\xbb\xaaNG XE</button><button id="next">\xc4\x90i ti\xe1\xba\xbfp t\xe1\xbb\xab tr\xe1\xba\xa1m</button><p>Gi\xe1\xbb\xaf n\xc3\xbat \xc4\x91\xe1\xbb\x83 ch\xe1\xba\xa1y tay, th\xe1\xba\xa3 \xc4\x91\xe1\xbb\x83 d\xe1\xbb\xabng.</p><div id="pad"><button data-c="G">\xe2\x86\x96</button><button data-c="F">\xe2\x86\x91 Ti\xe1\xba\xbfn</button><button data-c="I">\xe2\x86\x97</button><button data-c="L">\xe2\x86\x90 Tr\xc3\xa1i</button><button id="stop2" class="stop">D\xe1\xbb\xabng</button><button data-c="R">Ph\xe1\xba\xa3i \xe2\x86\x92</button><button data-c="H">\xe2\x86\x99</button><button data-c="B">\xe2\x86\x93 L\xc3\xb9i</button><button data-c="J">\xe2\x86\x98</button></div></section>\r\n<section><h2>V\xc3\xb2ng ch\xe1\xba\xa1y \xc2\xb7 3 tr\xe1\xba\xa1m</h2>\r\n<p id="positionText" aria-live="polite">\xc4\x90ang ch\xe1\xbb\x9d d\xe1\xbb\xaf li\xe1\xbb\x87u v\xe1\xbb\x8b tr\xc3\xad\xe2\x80\xa6</p>\r\n<svg id="trackMap" viewBox="0 0 600 350" role="img" aria-label="S\xc6\xa1 \xc4\x91\xe1\xbb\x93 v\xc3\xb2ng ch\xe1\xba\xa1y qua tr\xe1\xba\xa1m 1, 2, 3" style="width:100%;max-height:380px">\r\n<path d="M300 60 A220 110 0 0 1 490 225 A220 110 0 0 1 110 225 A220 110 0 0 1 300 60" fill="none" stroke="#475569" stroke-width="20"/>\r\n<g fill="none" stroke="#38bdf8" stroke-width="7" stroke-linecap="round">\r\n<path id="segment1" d="M300 60 A220 110 0 0 1 490 225" opacity="0.2"/>\r\n<path id="segment2" d="M490 225 A220 110 0 0 1 110 225" opacity="0.2"/>\r\n<path id="segment3" d="M110 225 A220 110 0 0 1 300 60" opacity="0.2"/></g>\r\n<g fill="#cbd5e1" font-size="16" text-anchor="middle"><text x="300" y="160">Chi\xe1\xbb\x81u ch\xe1\xba\xa1y: 1 \xe2\x86\x92 2 \xe2\x86\x92 3 \xe2\x86\x92 1</text><text id="mapMode" x="300" y="187">Ch\xe1\xbb\x9d k\xe1\xba\xbft n\xe1\xbb\x91i</text></g>\r\n<g fill="#1e293b" stroke="#94a3b8" stroke-width="4"><circle id="station1" cx="300" cy="60" r="23"/><circle id="station2" cx="490" cy="225" r="23"/><circle id="station3" cx="110" cy="225" r="23"/></g>\r\n<g fill="white" font-size="18" text-anchor="middle"><text x="300" y="24">Tr\xe1\xba\xa1m 1</text><text x="490" y="277">Tr\xe1\xba\xa1m 2</text><text x="110" y="277">Tr\xe1\xba\xa1m 3</text></g>\r\n<g id="mapCar" visibility="hidden"><rect x="-18" y="-12" width="36" height="24" rx="7" fill="#fbbf24" stroke="#101827" stroke-width="3"/><path d="M-6 -5 L7 0 L-6 5" fill="none" stroke="#101827" stroke-width="3"/></g>\r\n</svg><small>Bi\xe1\xbb\x83u t\xc6\xb0\xe1\xbb\xa3ng xe gi\xe1\xbb\xafa \xc4\x91o\xe1\xba\xa1n ch\xe1\xbb\x89 bi\xe1\xbb\x83u th\xe1\xbb\x8b \xc4\x91o\xe1\xba\xa1n \xc4\x91ang \xc4\x91i, kh\xc3\xb4ng ph\xe1\xba\xa3i t\xe1\xbb\x8da \xc4\x91\xe1\xbb\x99 th\xe1\xbb\xb1c. Nh\xe1\xba\xadn di\xe1\xbb\x87n tr\xe1\xba\xa1m theo th\xe1\xbb\xa9 t\xe1\xbb\xb1 \xc4\x91\xe1\xba\xbfm v\xe1\xba\xa1ch c\xe1\xbb\xa7a xe; kh\xe1\xbb\x9fi h\xc3\xa0nh tr\xc6\xb0\xe1\xbb\x9bc tr\xe1\xba\xa1m 1.</small></section>\r\n\r\n<section><h2>Gi\xc3\xa1m s\xc3\xa1t tr\xe1\xbb\xb1c ti\xe1\xba\xbfp</h2><div id="stats">Ch\xe1\xbb\x9d d\xe1\xbb\xaf li\xe1\xbb\x87u\xe2\x80\xa6</div><div id="lights"></div><small>5 m\xe1\xba\xaft d\xc3\xb2 line, t\xe1\xbb\xab tr\xc3\xa1i sang ph\xe1\xba\xa3i \xc2\xb7 s\xc3\xa1ng = th\xe1\xba\xa5y line</small><pre id="events"></pre></section>\r\n<section><h2>Th\xc3\xb4ng s\xe1\xbb\x91</h2><p>D\xe1\xbb\xabng xe tr\xc6\xb0\xe1\xbb\x9bc khi ch\xe1\xbb\x89nh. \xe2\x80\x9c\xc3\x81p d\xe1\xbb\xa5ng\xe2\x80\x9d c\xc3\xb3 hi\xe1\xbb\x87u l\xe1\xbb\xb1c ngay; \xe2\x80\x9cL\xc6\xb0u v\xc3\xa0o ESP32\xe2\x80\x9d gi\xe1\xbb\xaf th\xc3\xb4ng s\xe1\xbb\x91 sau khi t\xe1\xba\xaft ngu\xe1\xbb\x93n.</p><form id="form"><div id="fields"></div><p><button type="submit">\xc3\x81p d\xe1\xbb\xa5ng</button><button type="button" id="reload">\xc4\x90\xe1\xbb\x8dc l\xe1\xba\xa1i t\xe1\xbb\xab xe</button><button type="button" id="save">L\xc6\xb0u v\xc3\xa0o ESP32</button></p></form></section>\r\n<script>\r\nconst $=id=>document.getElementById(id);\r\nconst defs=[[\'KP\',\'Kp \xe2\x80\x94 t\xe1\xbb\x89 l\xe1\xbb\x87\',0,1000,.01],[\'KI\',\'Ki \xe2\x80\x94 t\xc3\xadch ph\xc3\xa2n\',0,1000,.01],[\'KD\',\'Kd \xe2\x80\x94 vi ph\xc3\xa2n\',0,2000,.01],[\'AS\',\'T\xe1\xbb\x91c \xc4\x91\xe1\xbb\x99 d\xc3\xb2 line\',0,255],[\'AM\',\'T\xe1\xbb\x91c \xc4\x91\xe1\xbb\x99 t\xe1\xbb\x91i \xc4\x91a d\xc3\xb2 line\',0,255],[\'AF\',\'T\xe1\xbb\x91c \xc4\x91\xe1\xbb\x99 t\xc3\xacm line\',0,255],[\'MS\',\'T\xe1\xbb\x91c \xc4\x91\xe1\xbb\x99 \xc4\x91i\xe1\xbb\x81u khi\xe1\xbb\x83n tay\',0,255],[\'TO\',\'T\xe1\xbb\xb1 d\xe1\xbb\xabng khi m\xe1\xba\xa5t l\xe1\xbb\x87nh (ms)\',200,2000],[\'AD\',\'Kho\xe1\xba\xa3ng c\xc3\xa1ch v\xe1\xba\xadt c\xe1\xba\xa3n (cm)\',5,200],[\'OB\',\'D\xe1\xbb\xabng khi g\xe1\xba\xb7p v\xe1\xba\xadt c\xe1\xba\xa3n\',0,1],[\'TW\',\'Ch\xe1\xbb\x9d t\xe1\xba\xa1i tr\xe1\xba\xa1m (gi\xc3\xa2y)\',1,120],[\'SM\',\'Ch\xe1\xba\xbf \xc4\x91\xe1\xbb\x99 d\xe1\xbb\xabng tr\xe1\xba\xa1m\',0,1],[\'BK\',\'Th\xe1\xbb\x9di gian phanh ng\xc6\xb0\xe1\xbb\xa3c (ms)\',0,300],[\'BP\',\'L\xe1\xbb\xb1c phanh ng\xc6\xb0\xe1\xbb\xa3c\',0,255],[\'GD\',\'Tr\xe1\xbb\x85 sau n\xc3\xbat \xc4\x91i ti\xe1\xba\xbfp (ms)\',0,10000],[\'SG\',\'Kh\xc3\xb3a nh\xe1\xba\xadn tr\xe1\xba\xa1m m\xe1\xbb\x9bi (ms)\',0,10000],[\'TS\',\'\xc4\x90i th\xe1\xba\xb3ng sau tr\xe1\xba\xa1m (ms)\',0,10000],[\'CL\',\'C\xc3\xb4ng su\xe1\xba\xa5t motor tr\xc3\xa1i (%)\',50,100],[\'CR\',\'C\xc3\xb4ng su\xe1\xba\xa5t motor ph\xe1\xba\xa3i (%)\',50,100]];\r\nfor(const [k,label,min,max,step] of defs){const l=document.createElement(\'label\');l.textContent=label+\' \xc2\xb7 \'+k;let x=document.createElement(k===\'SM\'||k===\'OB\'?\'select\':\'input\');x.id=k;if(x.tagName===\'SELECT\'){const names=k===\'SM\'?[\'Theo th\xe1\xbb\x9di gian\',\'Ch\xe1\xbb\x9d n\xc3\xbat b\xe1\xba\xa5m\']:[\'T\xe1\xba\xaft\',\'B\xe1\xba\xadt\'];names.forEach((name,i)=>{let o=document.createElement(\'option\');o.value=i;o.textContent=name;x.append(o)})}else{x.type=\'number\';x.min=min;x.max=max;x.step=step||1;x.required=true}l.append(x);$(\'fields\').append(l)}\r\nlet latest=null,initialized=false,held=null,moveBusy=false,chain=Promise.resolve(),online=false;\r\nfunction note(t){$(\'notice\').textContent=t}\r\nasync function api(path,data){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),1200);try{const r=await fetch(path,{method:data===undefined?\'GET\':\'POST\',headers:data===undefined?{}:{\'Content-Type\':\'application/json\'},body:data===undefined?undefined:JSON.stringify(data),signal:ctl.signal,cache:\'no-store\'});const j=await r.json();if(!r.ok)throw Error(j.error||\'Y\xc3\xaau c\xe1\xba\xa7u th\xe1\xba\xa5t b\xe1\xba\xa1i\');return j}finally{clearTimeout(timer)}}\r\nfunction command(c){chain=chain.catch(()=>{}).then(()=>api(\'/api/cmd\',{cmd:c}));return chain}\r\nfunction stop(){held=null;command(\'D\').catch(()=>note(\'M\xe1\xba\xa5t k\xe1\xba\xbft n\xe1\xbb\x91i; xe ch\xe1\xba\xa1y tay s\xe1\xba\xbd t\xe1\xbb\xb1 d\xe1\xbb\xabng theo th\xe1\xbb\x9di h\xe1\xba\xa1n TO.\'))}\r\nasync function pulse(){if(!held||moveBusy)return;moveBusy=true;try{await command(held)}catch(e){held=null;note(\'M\xe1\xba\xa5t k\xe1\xba\xbft n\xe1\xbb\x91i \xc4\x91i\xe1\xbb\x81u khi\xe1\xbb\x83n\')}finally{moveBusy=false}}\r\nfor(const b of document.querySelectorAll(\'[data-c]\')){b.onpointerdown=e=>{if(!online)return;e.preventDefault();held=b.dataset.c;b.setPointerCapture(e.pointerId);pulse()};b.onpointerup=b.onpointercancel=b.onlostpointercapture=()=>{if(held)stop()};b.oncontextmenu=e=>e.preventDefault()}\r\nsetInterval(pulse,80);\r\n$(\'stop\').onclick=$(\'stop2\').onclick=stop;\r\n$(\'auto\').onclick=()=>{held=null;command(\'U\').catch(e=>note(e.message))};$(\'next\').onclick=()=>command(\'N\').catch(e=>note(e.message));\r\nwindow.addEventListener(\'blur\',()=>{if(held)stop()});document.addEventListener(\'visibilitychange\',()=>{if(document.hidden&&held)stop()});window.addEventListener(\'pagehide\',()=>{if(held)navigator.sendBeacon(\'/api/cmd\',JSON.stringify({cmd:\'D\'}))});\r\nfunction fill(){if(latest){for(const [k]of defs)$(k).value=Number(latest.params[k]);initialized=true}}\r\n$(\'reload\').onclick=()=>{fill();note(\'\xc4\x90\xc3\xa3 \xc4\x91\xe1\xbb\x8dc l\xe1\xba\xa1i th\xc3\xb4ng s\xe1\xbb\x91 hi\xe1\xbb\x87n t\xe1\xba\xa1i\')};\r\n$(\'form\').onsubmit=async e=>{e.preventDefault();let data={};for(const [k]of defs)data[k]=Number($(k).value);try{await api(\'/api/params\',data);note(\'\xc4\x90\xc3\xa3 \xc3\xa1p d\xe1\xbb\xa5ng. B\xe1\xba\xa5m L\xc6\xb0u v\xc3\xa0o ESP32 \xc4\x91\xe1\xbb\x83 gi\xe1\xbb\xaf sau khi t\xe1\xba\xaft ngu\xe1\xbb\x93n.\')}catch(e){note(e.message)}};\r\n$(\'save\').onclick=async()=>{try{await api(\'/api/save\',{});note(\'\xc4\x90\xc3\xa3 l\xc6\xb0u th\xc3\xb4ng s\xe1\xbb\x91 \xc4\x91ang \xc3\xa1p d\xe1\xbb\xa5ng v\xc3\xa0o ESP32\')}catch(e){note(e.message)}};\r\nfunction updateMap(s){\r\n const p=s.location||{state:\'unknown\',current:0,last:0};\r\n const known=p.state===\'station\'||p.state===\'segment\';\r\n const car=$(\'mapCar\');car.setAttribute(\'visibility\',known?\'visible\':\'hidden\');\r\n $(\'trackMap\').style.opacity=\'1\';\r\n for(let i=1;i<=3;i++){\r\n  $(\'station\'+i).setAttribute(\'fill\',p.state===\'station\'&&p.current===i?\'#34d399\':\'#1e293b\');\r\n  $(\'segment\'+i).setAttribute(\'opacity\',p.state===\'segment\'&&p.last===i?\'1\':\'0.2\');\r\n }\r\n let text=\'Ch\xc6\xb0a x\xc3\xa1c \xc4\x91\xe1\xbb\x8bnh v\xe1\xbb\x8b tr\xc3\xad \xe2\x80\x94 ch\xe1\xbb\x9d xe nh\xe1\xba\xadn v\xe1\xba\xa1ch tr\xe1\xba\xa1m ti\xe1\xba\xbfp theo.\';\r\n if(p.state===\'before_first\')text=\'Kh\xe1\xbb\x9fi h\xc3\xa0nh tr\xc6\xb0\xe1\xbb\x9bc tr\xe1\xba\xa1m 1 \xe2\x80\x94 ch\xc6\xb0a nh\xe1\xba\xadn v\xe1\xba\xa1ch tr\xe1\xba\xa1m.\';\r\n if(p.state===\'station\'){\r\n  const xy={1:[300,60],2:[490,225],3:[110,225]}[p.current];\r\n  car.setAttribute(\'transform\',\'translate(\'+xy.join(\' \')+\')\');text=\'Xe t\xe1\xba\xa1i tr\xe1\xba\xa1m \'+p.current;\r\n }else if(p.state===\'segment\'){\r\n  const path=$(\'segment\'+p.last),point=path.getPointAtLength(path.getTotalLength()/2);\r\n  car.setAttribute(\'transform\',\'translate(\'+point.x+\' \'+point.y+\')\');\r\n  text=\'Xe tr\xc3\xaan \xc4\x91o\xe1\xba\xa1n tr\xe1\xba\xa1m \'+p.last+\' \xe2\x86\x92 tr\xe1\xba\xa1m \'+s.next_station+\' (v\xe1\xbb\x8b tr\xc3\xad trong \xc4\x91o\xe1\xba\xa1n ch\xc6\xb0a x\xc3\xa1c \xc4\x91\xe1\xbb\x8bnh)\';\r\n }\r\n $(\'mapMode\').textContent=s.mode===\'DO LINE\'&&s.road===\'WAIT\'?\'\xc4\x90ang ch\xe1\xbb\x9d v\xe1\xba\xadt c\xe1\xba\xa3n\':s.mode===\'DUNG\'?\'\xc4\x90\xc3\xa3 d\xe1\xbb\xabng\':s.mode===\'DIEU KHIEN TAY\'?\'\xc4\x90i\xe1\xbb\x81u khi\xe1\xbb\x83n tay\':s.waiting?\'\xc4\x90ang ch\xe1\xbb\x9d t\xe1\xba\xa1i tr\xe1\xba\xa1m\':\'\xc4\x90ang d\xc3\xb2 line\';\r\n $(\'positionText\').textContent=text;\r\n}\r\nfunction staleMap(){\r\n $(\'trackMap\').style.opacity=\'.4\';$(\'mapCar\').setAttribute(\'visibility\',\'hidden\');\r\n $(\'mapMode\').textContent=\'M\xe1\xba\xa5t k\xe1\xba\xbft n\xe1\xbb\x91i\';$(\'positionText\').textContent=\'M\xe1\xba\xa5t k\xe1\xba\xbft n\xe1\xbb\x91i \xe2\x80\x94 kh\xc3\xb4ng c\xc3\xb3 d\xe1\xbb\xaf li\xe1\xbb\x87u v\xe1\xbb\x8b tr\xc3\xad hi\xe1\xbb\x87n t\xe1\xba\xa1i.\';\r\n}\r\n\r\nasync function refresh(){try{latest=await api(\'/api/status\');online=true;updateMap(latest);$(\'connection\').textContent=\'\xe2\x97\x8f \xc4\x90\xc3\xa3 k\xe1\xba\xbft n\xe1\xbb\x91i \xc2\xb7 c\xe1\xba\xadp nh\xe1\xba\xadt m\xe1\xbb\x97i 0,5 gi\xc3\xa2y\';$(\'stats\').textContent=\'Ch\xe1\xba\xbf \xc4\x91\xe1\xbb\x99: \'+latest.mode+\' | Kho\xe1\xba\xa3ng c\xc3\xa1ch: \'+(latest.distance===null?\'Kh\xc3\xb4ng c\xc3\xb3 t\xc3\xadn hi\xe1\xbb\x87u\':latest.distance.toFixed(1)+\' cm\')+\' | V\xe1\xba\xadt c\xe1\xba\xa3n: \'+(latest.road===\'WAIT\'?\'\xc4\x90ang ch\xe1\xbb\x9d\':\'Th\xc3\xb4ng tho\xc3\xa1ng\')+\' | \'+(latest.waiting?\'\xc4\x90ang d\xe1\xbb\xabng t\xe1\xba\xa1i tr\xe1\xba\xa1m\':\'\xc4\x90ang ngo\xc3\xa0i tr\xe1\xba\xa1m\')+\' | Tr\xe1\xba\xa1m k\xe1\xba\xbf ti\xe1\xba\xbfp: \'+latest.next_station+\' | L\xe1\xbb\x87nh motor tr\xc3\xa1i/ph\xe1\xba\xa3i: \'+latest.motor.join(\' / \');$(\'lights\').innerHTML=latest.line.map(v=>\'<span class="\'+(v?\'on\':\'\')+\'"></span>\').join(\'\');$(\'events\').textContent=latest.events.join(\'\\n\');if(!initialized)fill()}catch(e){online=false;held=null;staleMap();$(\'connection\').textContent=\'\xe2\x97\x8f M\xe1\xba\xa5t k\xe1\xba\xbft n\xe1\xbb\x91i \xe2\x80\x94 ki\xe1\xbb\x83m tra Wi-Fi\';}finally{setTimeout(refresh,500)}}refresh();\r\n</script></html>\r\n'


# ---------- Chuong trinh chinh ----------
def main():
    global station_deadline, next_station, station_waiting, go_flag, go_at
    global location_state, last_station, current_station, station_visits
    global mode, integral, last_error, last_known_error
    global ob_next_check, mv_l, mv_r, manual_deadline, straight_until

    stop_motors()
    load_settings()
    start_wifi()

    while True:
        poll_http()
        sample_telemetry()
        poll_button()

        # ----- Dung han -----
        if mode == MODE_STOP:
            station_deadline = None
            station_waiting = False
            go_flag = False
            go_at = None
            straight_until = None
            stop_motors()
            time.sleep_ms(10)
            continue

        # ----- Dieu khien tay -----
        if mode == MODE_MANUAL:
            station_deadline = None
            station_waiting = False
            go_flag = False
            go_at = None
            straight_until = None
            # An toan: mat ket noi Wi-Fi thi dung xe
            if not ap.active():
                mode = MODE_STOP
                stop_motors()
            # Het han ma khong co lenh moi -> dung xe (app khong gui lenh luc tha nut)
            elif manual_deadline is not None and time.ticks_diff(time.ticks_ms(), manual_deadline) >= 0:
                mv_l = 0
                mv_r = 0
                manual_deadline = None
                stop_motors()
            time.sleep_ms(10)
            continue

        # ----- Dang dung cho o tram -----
        if station_waiting:
            now = time.ticks_ms()

            # Vua bam nut / lenh N -> hen gio chay sau go_delay_ms
            if go_flag and go_at is None:
                go_flag = False
                go_at = time.ticks_add(now, go_delay_ms)
                log_event("SE DI SAU %d ms" % go_delay_ms)

            if go_at is not None:
                # Dang dem tre: van dung yen
                if time.ticks_diff(now, go_at) >= 0:
                    leave_station("DI TIEP")
                else:
                    stop_motors()
                    time.sleep_ms(5)
                    continue
            elif station_btn_mode:
                # Che do nut bam: dung cho den khi bam
                station_deadline = None
                stop_motors()
                time.sleep_ms(5)
                continue
            else:
                # Che do thoi gian
                if station_deadline is None:
                    station_deadline = time.ticks_add(now, station_wait_s * 1000)
                if time.ticks_diff(now, station_deadline) < 0:
                    stop_motors()
                    time.sleep_ms(5)
                    continue
                leave_station("HET GIO CHO")

        # ----- Phat hien tram -----
        if station_detected():
            brake_fast()   # phanh nhanh de dung dung vach tram
            station_waiting = True
            location_state = "station"
            current_station = last_station = next_station
            station_visits += 1
            go_flag = False
            go_at = None
            if station_btn_mode:
                station_deadline = None
                log_event("DA DEN TRAM %d - BAM NUT DE DI TIEP" % next_station)
            else:
                station_deadline = time.ticks_add(time.ticks_ms(), station_wait_s * 1000)
                log_event("DA DEN TRAM %d - DUNG %d s" % (next_station, station_wait_s))
            next_station = next_station % 3 + 1
            continue

        # ----- Vat can: dung neu co vat can phia truoc -----
        if ob_enable:
            now = time.ticks_ms()
            if time.ticks_diff(now, ob_next_check) >= 0:
                ob_next_check = time.ticks_add(now, 60)
                update_road(last_distance)
            if road_state == "WAIT":
                stop_motors()
                time.sleep_ms(5)
                continue

        # ----- Moi roi tram: di thang, bo qua line -----
        if straight_until is not None:
            if time.ticks_diff(time.ticks_ms(), straight_until) < 0:
                set_motor_left(base_speed)
                set_motor_right(base_speed)
                time.sleep_ms(5)
                continue
            straight_until = None
            integral = last_error = 0.0

        # ----- Tu dong do line (PID) -----
        error = read_line()

        if error is not None:
            last_known_error = error

            integral += error
            integral = max(-50, min(50, integral))
            derivative = error - last_error
            correction = Kp * error + Ki * integral + Kd * derivative
            last_error = error

            left = max(-max_speed, min(max_speed, base_speed + int(correction)))
            right = max(-max_speed, min(max_speed, base_speed - int(correction)))

            if 0 < left < min_start:
                left = min(min_start, max_speed)
            if 0 < right < min_start:
                right = min(min_start, max_speed)

            set_motor_left(left)
            set_motor_right(right)
        else:
            # Mat line: quay ve phia line bien mat lan cuoi
            integral = 0.0
            if last_known_error < 0:
                set_motor_left(-search_speed)
                set_motor_right(search_speed)
            else:
                set_motor_left(search_speed)
                set_motor_right(-search_speed)

        time.sleep_ms(5)


try:
    main()
finally:
    stop_motors()
    led.value(0)
    for client in clients:
        client[0].close()
    if server is not None:
        server.close()
    print("Da dung")
