#!/usr/bin/env python3
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import serial


@dataclass
class Sensor:
    sensor_id: int
    fx: int = 0
    fy: int = 0
    fz: int = 0
    abs: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    is_present: bool = True
    is_3d: bool = True


def cart2sph(x: int, y: int, z: int) -> Tuple[float, float, float]:
    hxy = np.hypot(x, y)
    radius = np.hypot(hxy, z)
    elevation = np.pi / 2 - np.arctan2(hxy, z)
    azimuth = np.arctan2(y, x)
    return float(radius), float(elevation), float(azimuth)


def _parse_data_into_sensors(
    data: List[str],
    sensors: List[Sensor],
    has_timestamp: bool,
) -> Optional[str]:
    if not data:
        return None
    if data[0] == '@':
        data_offset = 1
        if has_timestamp:
            timestamp = f"{data[1]},{data[2]}"
            data_offset = 3
        else:
            timestamp = None
            data_offset = 1
        for i, sensor in enumerate(sensors):
            fx_idx = data_offset + 0 + 3 * i
            fy_idx = data_offset + 1 + 3 * i
            fz_idx = data_offset + 2 + 3 * i

            fx_raw = data[fx_idx].strip()
            fy_raw = data[fy_idx].strip()
            fz_raw = data[fz_idx].strip()

            if fz_raw == '':
                sensor.sensor_id = i
                sensor.fx = 0
                sensor.fy = 0
                sensor.fz = 0
                sensor.is_present = False
                continue

            sensor.is_present = True

            if fx_raw == '' or fy_raw == '':
                sensor.sensor_id = i
                sensor.fx = 0
                sensor.fy = 0
                sensor.fz = int(fz_raw)
                sensor.is_3d = False
                continue

            try:
                fx = int(fx_raw)
                fy = int(fy_raw)
                fz = int(fz_raw)
            except ValueError:
                sensor.is_present = False
                continue

            sensor.sensor_id = i
            sensor.fx = fx
            sensor.fy = fy
            sensor.fz = fz
            sensor.is_3d = True
            radius, theta, phi = cart2sph(fx, fy, fz)
            sensor.abs = radius
            sensor.pitch = theta
            sensor.yaw = phi
        return timestamp
    return None


def read_sensor_frame(
    port: str,
    sensor_number: int = 5,
    baudrate: int = 1000000,
    timeout: float = 1.5,
) -> Tuple[Optional[str], List[Sensor]]:
    """Read one sensor frame from serial and return (timestamp, sensors)."""
    if sensor_number <= 0:
        raise ValueError("sensor_number must be > 0")

    sensors = [Sensor(i) for i in range(sensor_number)]

    with serial.Serial(port, baudrate, timeout=timeout, write_timeout=1) as ser:
        time.sleep(0.1)
        line = ser.readline()

    if not line:
        return None, sensors

    try:
        line_str = line.decode("utf-8").strip()
    except UnicodeDecodeError:
        return None, sensors

    data = line_str.split(',')

    #cut of null terminator
    data = data[:-1]

    has_timestamp = False
    if len(data) == sensor_number * 3 + 3:
        has_timestamp = True
    elif len(data) == sensor_number * 3 + 1:
        has_timestamp = False
    else:
        return None, None
    
    timestamp = _parse_data_into_sensors(data, sensors, has_timestamp)
    return timestamp, sensors
