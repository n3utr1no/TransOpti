
class Node:
    def __init__(self, stop_id, trip_id, stop_sequence, agency_id):
        self.properties = {
            "stopID": stop_id,
            "tripID": trip_id,
            "stopSequence": stop_sequence,
            "agency_id": agency_id,
        }

    def __str__(self):
        return str(self.properties)

    def __hash__(self):
        return hash(tuple(sorted(self.properties.items())))

    def __eq__(self, other):
        return isinstance(other, Node) and self.properties == other.properties