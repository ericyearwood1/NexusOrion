from local_multiplayer.enums import SpatialAnchorProgress


class SpatialAnchor:
    def __init__(self, name, anchor_type):
        self.name = name
        self.uuid = ""
        self.room = ""
        self.anchor_type = anchor_type
        self.created_by = ""
        self.details = ""
        self.shared_with = set()

    def serialize(self):
        return {
            'name': self.name,
            'uuid': self.uuid,
            'room': self.room,
            'details': self.details,
            'anchor_type': self.anchor_type,
            'created_by': self.created_by,
            'shared_with': list(self.shared_with)
        }

    @staticmethod
    def deserialize(data_dict):
        anchor = SpatialAnchor(data_dict['name'], data_dict['anchor_type'])
        anchor.uuid = data_dict['uuid']
        anchor.created_by = data_dict['created_by']
        anchor.room = data_dict['room']
        anchor.details = data_dict['details']
        shared_with = set()
        for user_id in data_dict['shared_with']:
            shared_with.add(user_id)
        anchor.shared_with = shared_with
        return anchor
