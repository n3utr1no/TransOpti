from datetime import datetime
from math import radians, sin, cos, sqrt, atan2

def calculate_time_difference(timestamp1, timestamp2):
    t1 = datetime.strptime(timestamp1, "%H:%M:%S")
    t2 = datetime.strptime(timestamp2, "%H:%M:%S")
    return (t2 - t1).total_seconds()


def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance