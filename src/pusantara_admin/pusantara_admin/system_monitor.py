import os
import socket
import subprocess
import time
import psutil

def get_hostname():
    try:
        return socket.gethostname()
    except Exception:
        return "Unknown"

def get_ip_address():
    try:
        # Standard way to get the primary IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Doesn't need to be reachable
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"

def get_cpu_temp():
    # Attempt to read from thermal zones (common in Jetson / Linux)
    thermal_dirs = ["/sys/class/thermal/thermal_zone0", "/sys/class/thermal/thermal_zone1"]
    for directory in thermal_dirs:
        try:
            type_path = os.path.join(directory, "type")
            temp_path = os.path.join(directory, "temp")
            if os.path.exists(type_path) and os.path.exists(temp_path):
                with open(type_path, "r") as f:
                    tz_type = f.read().strip().lower()
                # Jetson CPU temp is usually zone0 or has 'cpu' in type
                with open(temp_path, "r") as f:
                    temp_raw = float(f.read().strip())
                temp_c = temp_raw / 1000.0
                return temp_c
        except Exception:
            pass

    # Fallback to psutil temperatures if available
    try:
        temps = psutil.sensors_temperatures()
        if temps:
            for name, entries in temps.items():
                for entry in entries:
                    return entry.current
    except Exception:
        pass

    return 45.0  # Fallback dummy value if unavailable

def get_wifi_status():
    try:
        # Try iwgetid first
        ssid = subprocess.check_output(["iwgetid", "-r"], stderr=subprocess.DEVNULL).decode().strip()
        if ssid:
            return "Connected", ssid
    except Exception:
        pass

    try:
        # Try nmcli
        out = subprocess.check_output(
            "nmcli -t -f active,ssid dev wifi | grep '^yes:'",
            shell=True,
            stderr=subprocess.DEVNULL
        ).decode().strip()
        if out and out.startswith("yes:"):
            ssid = out.split(":", 1)[1]
            return "Connected", ssid
    except Exception:
        pass

    # Check route or interface to see if wifi interface is up
    try:
        for interface, addrs in psutil.net_if_addrs().items():
            if "wlan" in interface or "wlp" in interface:
                stats = psutil.net_if_stats().get(interface)
                if stats and stats.isup:
                    return "Connected (No SSID)", interface
    except Exception:
        pass

    return "Disconnected", "None"

def get_uptime_seconds():
    try:
        return time.time() - psutil.boot_time()
    except Exception:
        try:
            with open("/proc/uptime", "r") as f:
                return float(f.readline().split()[0])
        except Exception:
            return 0.0

def format_uptime(seconds):
    days = int(seconds // 86400)
    hours = int((seconds % 86400) // 3600)
    minutes = int((seconds % 3600) // 60)
    secs = int(seconds % 60)
    if days > 0:
        return f"{days}d {hours:02d}:{minutes:02d}:{secs:02d}"
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"

def get_system_stats():
    wifi_status, wifi_name = get_wifi_status()
    uptime_sec = get_uptime_seconds()
    
    return {
        "cpu_percent": psutil.cpu_percent(),
        "ram_percent": psutil.virtual_memory().percent,
        "disk_percent": psutil.disk_usage("/").percent,
        "cpu_temp": get_cpu_temp(),
        "hostname": get_hostname(),
        "ip_address": get_ip_address(),
        "wifi_status": wifi_status,
        "wifi_name": wifi_name,
        "uptime": format_uptime(uptime_sec),
        "uptime_raw": uptime_sec
    }
