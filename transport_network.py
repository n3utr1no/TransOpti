import csv
import heapq
from node import Node
from utils import calculate_time_difference, haversine
class TransportNetwork:
    def __init__(self):
        self.graph = {}
        self.weight = {}
        self.edge_type = {}
        self.arrival_times = {}
        self.departure_times = {}
        self.route = {}
        self.max_stop_sequence = {}
        self.stop_location = {}
        self.dp_dist = {}
        self.stop_set = set()
        self.graph_size = 0
        self.edges_no = 0
        self.component_labels = {}

    def add_node(self, stop_id, trip_id, stop_sequence, agency_id, arrival_time, departure_time):
        node = Node(stop_id, trip_id, stop_sequence, agency_id)
        if 28.5272 <= self.stop_location[(stop_id, agency_id)][0] <= 28.5781 and 77.1461 <= self.stop_location[(stop_id, agency_id)][1] <= 77.2479:
            if node not in self.graph:
                #print(f"Adding node: {node}")
                self.graph[node] = []
                self.arrival_times[node] = arrival_time
                self.departure_times[node] = departure_time
                self.graph_size += 1
            else:
                print(f"Node {node} already exists in the graph.")
        #else:
            #print(f"Node {node} not added due to location criteria.")

    def add_location(self, stop_id, agency_id, lt, ln):
        self.stop_location[(stop_id, agency_id)] = (lt, ln)

    def add_in_vehicle_edge(self, source_node, destination_node):
        self.graph[source_node].append(destination_node)
        try:
            cost = calculate_time_difference(self.departure_times[destination_node], self.arrival_times[source_node])
            self.weight[(source_node, destination_node)] = cost
            self.edge_type[(source_node, destination_node)] = "1"
            self.edges_no += 1
        except:
            return

    def calculate_distance(self, node1, node2):
        stop_id1 = node1.properties["stopID"]
        agency_id1 = node1.properties["agency_id"]
        stop_id2 = node2.properties["stopID"]
        agency_id2 = node2.properties["agency_id"]

        coord1 = self.stop_location.get((stop_id1, agency_id1))
        coord2 = self.stop_location.get((stop_id2, agency_id2))

        if coord1 is not None and coord2 is not None:
            if (node2, node1) not in self.dp_dist:
                distance = haversine(coord1[0], coord1[1], coord2[0], coord2[1])
                self.dp_dist[(node1, node2)] = distance
            else:
                distance = self.dp_dist[(node1, node2)]

            return distance
        else:
            # Handle missing coordinates gracefully
            return float('inf')

    def source_access_links(self, lat, longi, curr_time):
        links = []
        max_latitude_difference = 0.002242
        max_longitude_difference = 0.002948
        for node in self.graph:
            dt = self.departure_times[node]
            try:
                time_diff = calculate_time_difference(dt, curr_time)
                if 0 < time_diff < 600:
                    stop_id = node.properties["stopID"]
                    agency_id1 = '1'
                    stop_loc_lat = self.stop_location[(stop_id, agency_id1)][0]
                    stop_loc_long = self.stop_location[(stop_id, agency_id1)][1]
                    lat_diff = max(lat, stop_loc_lat) - min(lat, stop_loc_lat)
                    long_diff = max(longi, stop_loc_long) - min(longi, stop_loc_long)
                    if lat_diff <= max_latitude_difference or long_diff <= max_longitude_difference:
                        links.append(node)
            except:
                continue
        return links

    def egress_links(self, lt, ln, curr_time):
        links = []
        max_latitude_difference = 0.002242
        max_longitude_difference = 0.002948
        for node in self.graph:
            dt = self.departure_times[node]
            try:
                time_diff = calculate_time_difference(dt, curr_time)
                if 0 < time_diff < 2400:
                    stop_id = node.properties["stopID"]
                    agency_id1 = '1'
                    stop_loc_lat = self.stop_location[(stop_id, agency_id1)][0]
                    stop_loc_long = self.stop_location[(stop_id, agency_id1)][1]
                    lat_diff = max(lt, stop_loc_lat) - min(lt, stop_loc_lat)
                    long_diff = max(ln, stop_loc_long) - min(ln, stop_loc_long)
                    if lat_diff <= max_latitude_difference or long_diff <= max_longitude_difference:
                        links.append(node)
            except:
                continue
        return links


    def create_stop_set(self):
      lat_threshold = 0.002849
      lon_threshold = 0.004057
      stop_items = list(self.stop_location.items())
      for i, (stop1, (lt1, ln1)) in enumerate(stop_items):
          stop_id1, agency_id1 = stop1
          for _, (lt2, ln2) in stop_items[i + 1:]:
              lt_diff = abs(lt1 - lt2)
              ln_diff = abs(ln1 - ln2)
              if lt_diff < lat_threshold and ln_diff < lon_threshold:
                  stop_id2, agency_id2 = _
                  self.stop_set.add(((stop_id1, agency_id1), (stop_id2, agency_id2)))


    def add_transfer_edges(self):
        for node1 in self.graph:
            trip_id1 = node1.properties["tripID"]
            ss1 = node1.properties["stopSequence"]
            agency_id1 = node1.properties["agency_id"]
            stop_id1 = node1.properties["stopID"]
            route_id1 = self.route[trip_id1]
            arrival_time1 = self.arrival_times[node1]

            for node2 in self.graph:
                trip_id2 = node2.properties["tripID"]
                ss2 = node2.properties["stopSequence"]
                route_id2 = self.route[trip_id2]
                departure_time2 = self.departure_times[node2]
                agency_id2 = node2.properties["agency_id"]
                stop_id2 = node2.properties["stopID"]

                if route_id1 != route_id2 and ss1 != '0' and self.max_stop_sequence[trip_id2] != ss2:
                    try:
                        t_diff = calculate_time_difference(departure_time2, arrival_time1)

                        if 0 <= t_diff <= 600:
                            if ((stop_id1, agency_id1), (stop_id2, agency_id2)) in self.stop_set:
                                dist = self.calculate_distance(node1, node2)
                                self.graph[node1].append(node2)
                                walking_time = dist * 720
                                waiting_time = t_diff - walking_time
                                cost = 2.5 * walking_time + 2 * waiting_time  # actual journey time is difference between arrival and dep time
                                self.weight[(node1, node2)] = cost
                                self.edge_type[(node1, node2)] = '2'
                                self.edges_no += 1
                    except:
                        continue

    def find_connected_components(self):

        current_label = 0  # Initial label for the connected component

        def dfs(node, label):
            # Recursive DFS to label connected components
            self.component_labels[node] = label
            for neighbor in self.graph[node]:
                if neighbor not in self.component_labels:
                    dfs(neighbor, label)

        for node in self.graph:
            if node not in self.component_labels:
                dfs(node, current_label)
                current_label += 1

    def find_shortest_path(self, node1, node2):

        if node1 not in self.graph or node2 not in self.graph:
            print("Error: One or both nodes not found in the graph.")
            return None

        if self.component_labels.get(node1) != self.component_labels.get(node2):
            print("No path exists between nodes in different connected components.")
            return None

        # Dijkstra's algorithm to find shortest path and weight
        priority_queue = [(0, node1, [])]  # (cost, current_node, path_so_far)
        visited = set()

        while priority_queue:
            current_cost, current_node, path_so_far = heapq.heappop(priority_queue)

            if current_node in visited:
                continue

            visited.add(current_node)
            path_so_far = path_so_far + [current_node]

            if current_node == node2:
                return path_so_far, current_cost

            for neighbor in self.graph[current_node]:
                if neighbor not in visited:
                    weight = self.weight.get((current_node, neighbor), float('inf'))
                    heapq.heappush(priority_queue, (current_cost + weight, neighbor, path_so_far))

        print("No path found between the given nodes.")
        return None


    def write_edge(self, node):
        for node2 in self.graph[node]:
            return f"{node} to {node2}"