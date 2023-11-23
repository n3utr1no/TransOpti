from transport_network import TransportNetwork
from data_loader import load_csv

test_network = TransportNetwork()
# Load DMRC stops data
dmrc_stops_header, dmrc_stops_data = load_csv('dmrc_stops.txt')

# Load stops data
stops_header, stops_data = load_csv('stops.txt')

# Load DMRC trips data
dmrc_trips_header, dmrc_trips_data = load_csv('dmrc_trips.txt')

# Load trips data
trips_header, trips_data = load_csv('trips.txt')

# Load bus trips data
bus_trips_header, bus_trips_data = load_csv('bus_trips.txt')

# Load stop times data
stop_times_header, stop_times_data = load_csv('stop_times.txt')

# Load DMRC stop times data
dmrc_stop_times_header, dmrc_stop_times_data = load_csv('dmrc_stop_times.txt')

