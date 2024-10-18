from datetime import datetime
from zoneinfo import ZoneInfo
from scipy.signal import butter, filtfilt


def m_per_sec_to_kmh(m_per_sec):
    return m_per_sec * 3.6

def kmh_to_m_per_sec(kmh):
    return kmh / 3.6

def map_to_world(mx, my, origin, size, resolution):
    pgm_mx = int(mx + 0.5)
    pgm_my = size[1] - 1 - int(my + 0.5)
    wx = pgm_mx * resolution + origin[0]
    wy = pgm_my * resolution + origin[1]
    return wx, wy


def world_to_map(wx, wy, origin, size, resolution):
    mx = int((wx - origin[0]) / resolution + 0.5)
    my = int((size[1] - 1) - (wy - origin[1]) / resolution + 0.5)
    return mx, my


def to_jst(timestamp):
    jst_time = datetime.fromtimestamp(timestamp / 1e9, ZoneInfo("Asia/Tokyo"))
    return jst_time


def lowpass_filter(data, cutoff_freq, fs, order=5):
    nyquist_freq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist_freq
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    y = filtfilt(b, a, data)
    return y
