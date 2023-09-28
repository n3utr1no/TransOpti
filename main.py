import networkx as nx
import math

class Node:
    def __init__(self, tripID, stopID, stopSequence):
        self.stopID = stopID
        self.tripID = tripID
        self.stopSequence = stopSequence

    def __str__(self):
        return f"Node({self.tripID}, {self.stopID}, {self.stopSequence})"


def CalTimeDiff(t1,t2):
    h1, h2,m1,m2,s1,s2 = t1[:2], t2[:2], t1[3:5], t2[3:5], t1[6:8], t2[6:8]
    diff = ((int(h1)-int(h2))*3600)+((int(m1)-int(m2))*60)+(int(s1)-int(s2))
    return diff

G = nx.DiGraph()

with open('stop_times.txt', 'r') as file:
    lines = file.readlines()  #reading of stop_times.txt file and
    bus_stops_data = [line.strip().split(',') for line in lines]    #storing the values in a 2D array, stops_data

prevstop = Node(bus_stops_data[1][0], bus_stops_data[1][3], bus_stops_data[1][4])
prevtripID = ""
prevtime = ''
prevStopSequence = ''
lastseqByTrip = {}
for b in bus_stops_data:
    #print(b)
    if(b!=bus_stops_data[0]):
        n1 = Node(b[0], b[3], b[4])
        #n1.printNode()
        currtripID = b[0]
        currSS = b[4]
        if prevtripID != currtripID:
            lastseqByTrip[prevtripID] = prevStopSequence
            prevstop = n1
            prevtripID = currtripID
            prevtime = b[1]
            prevStopSequence = currSS
            continue
        else:
            currentTime = b[1]
            cost = CalTimeDiff(currentTime, prevtime)
            prevtime = currentTime
            G.add_edge(prevstop, n1, weight = cost)
            prevStopSequence = currSS





#Question2:
#make a dictionary to get values of stop name for stop code for buses
with open('stops.txt', 'r') as file:
    lines = file.readlines()
    BusstopsDetail = [line.strip().split(',') for line in lines]

checkbusstopname = {} #creating dictionary for checking the stop name
bus_stop_location = {} #creating dictionary to get latitude by stopid
for stop in BusstopsDetail:
    if stop[0] != BusstopsDetail[0][0]:
        stop3 = float(stop[3])
        stop4 = float(stop[4])
        checkbusstopname[stop[0]] = stop[2]
        bus_stop_location[stop[0]] = (stop3,stop4)

with open('trips.txt', 'r') as file:
    lines = file.readlines()
    tripsDetail = [line.strip().split(',') for line in lines]

routeFromTrip = {}
for trip in tripsDetail:
    routeFromTrip[trip[2]] = trip[0]
def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in kilometers
    radius = 6371.0

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = radius * c

    return distance

def nearestBusStops(current_lat, current_long):
    bus_stops_access = []
    for key in bus_stop_location:
        stop_lat = float(bus_stop_location[key][0])
        stop_long = float(bus_stop_location[key][1])
        k = haversine(current_lat, current_long, stop_lat, stop_long)
        if k<0.5:
            bus_stops_access.append(key)
    return bus_stops_access

#question3: Transfer Links

for node1 in G.nodes():
    for node2 in G.nodes():
        trip1 = node1.tripID
        stop1 = node1.stopID
        trip2 = node2.tripID
        stop2 = node2.stopID
        stop_lat1 = float(bus_stop_location[stop1][0])
        stop_long1 = float(bus_stop_location[stop1][1])
        stop_lat2 = float(bus_stop_location[stop2][0])
        stop_long2 = float(bus_stop_location[stop2][1])
        dist = haversine(stop_lat1, stop_long1, stop_lat2, stop_long2)
        walking_time = 1200*(dist+0.1)
        waiting_time = 
        if routeFromTrip[trip1] != routeFromTrip[trip2] and dist< 0.5 and











"""
#defining a function to calculate the time difference in seconds
def CalTimeDiff(t1,t2):
    h1, h2,m1,m2,s1,s2 = t1[:2], t2[:2], t1[3:5], t2[3:5], t1[6:8], t2[6:8]
    diff = ((int(h1)-int(h2))*3600)+((int(m1)-int(m2))*60)+(int(s1)-int(s2))
    return diff



with open('stop_times.txt', 'r') as file:
    lines = file.readlines()  #reading of stop_times.txt file and
    bus_stops_data = [line.strip().split(',') for line in lines]    #storing the values in a 2D array, stops_data



with open('dmrc_stop_times.txt', 'r') as file:
    lines = file.readlines()  #reading of stop_times.txt file and
    metro_stops_data = [line.strip().split(',') for line in lines]    #storing the values in a 2D array, stops_data



def Makegraph(stops_data):
    G = nx.DiGraph()
    i=0 #initiating a counter
    prev_stop = ""
    prev_time = ""
    #making of graph through lines of stop_times.txt file
    for stops in stops_data:
        stopsequence = stops[4]
        if i<2:
            i=i+1
            prev_stop = stops[3]
            prev_time = stops[1]
        else:
            curr_stop = stops[3]
            curr_time = stops[1]
            if stopsequence != '0':
                w = CalTimeDiff(curr_time, prev_time)
                #print(w)
                G.add_edge(prev_stop, curr_stop, weight = w)
            i=i+1
            prev_stop=curr_stop
            prev_time=curr_time
    return G

BusGraph = Makegraph(bus_stops_data)
MetroGraph = Makegraph(metro_stops_data)
MetroGraph.add_edge('234', "500", weight = 140)
MetroGraph.add_edge('500', "234", weight = 140)



#functiom to print the graph, input: graph
def printGraph(G1):
    for edge in G1.edges(data=True):
        source, target, data = edge
        print(f"Edge ({source} -> {target}): {data}")



#make a dictionary to get values of stop name for stop code for buses
with open('stops.txt', 'r') as file:
    lines = file.readlines()
    BusstopsDetail = [line.strip().split(',') for line in lines]
checkbusstopname = {} #creating dictionary for checking the stop name
bus_stop_location = {} #creating dictionary to get latitude by stopid
for stop in BusstopsDetail:
    if stop[0] != BusstopsDetail[0][0]:
        stop3 = float(stop[3])
        stop4 = float(stop[4])
        checkbusstopname[stop[0]] = stop[2]
        bus_stop_location[stop[0]] = (stop3,stop4)



#make a dictionary to get values of stop name for stop code for Metro
with open('dmrc_stops.txt', 'r') as file:
    lines = file.readlines()
    MetrostopsDetail = [line.strip().split(',') for line in lines]
checkMetrostopname = {} #creating dictionary for checking the stop name
metro_stop_location = {} #creating dictionary to get latitude by stopid
for stop in MetrostopsDetail:
    if stop[0] != MetrostopsDetail[0][0]:
        stop3 = float(stop[4])
        stop4 = float(stop[5])
        checkMetrostopname[stop[0]] = stop[2]
        metro_stop_location[stop[0]] = (stop3,stop4)




def BusstopCodeToName(stopcode): #gives out stop names from stop_codes.
    return checkbusstopname[stopcode]



def MetrostopCodeToName(stopcode): #gives out stop names from stop_codes.
    return checkMetrostopname[stopcode]



#shortest path calculation
def findShortestPath(source_stop, target_stop,G):
    try:
        #shortest_paths = dict(nx.all_pairs_shortest_path(G))
        shortest_path = nx.shortest_path(G, source=source_stop, target=target_stop, weight='weight')
        sp = len(shortest_path)
        path = []
        for i in range(sp):
            if G == MetroGraph:
                path.append(MetrostopCodeToName(shortest_path[i]))
            else:
                path.append(BusstopCodeToName(shortest_path[i]))
        return path
    except nx.NetworkXNoPath:
        print("No path exists between source and target nodes.")

#print(findShortestPath('62', '508', MetroGraph))
#strongly_connected_components = list(nx.strongly_connected_components(BusGraph))
#print(len(strongly_connected_components))

#shortest path length calculations in seconds
def findPathLength(source_stop, target_stop,G):
    shortest_path_length = nx.shortest_path_length(G, source=source_stop, target=target_stop, weight='weight')
    return shortest_path_length



#Nearest Stop calculations.

# Function to calculate distance between two points using Haversine formula
def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in kilometers
    radius = 6371.0

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = radius * c

    return distance



def FindNearestStop(current_latitude, current_longitude, stop_location):
    min_distance = float("inf")
    nearestStopName = -1
    for key in stop_location:
        stop_lat = float(stop_location[key][0])
        stop_long = float(stop_location[key][1])
        k = haversine(current_latitude, current_longitude, stop_lat, stop_long)
        if min_distance>k:
            min_distance = k
            nearestStop = key
    print(min_distance)
    return nearestStop


"""





