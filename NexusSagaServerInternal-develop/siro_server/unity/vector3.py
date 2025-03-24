class Vector3:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @staticmethod
    def deserialize(data_dict):
        return Vector3(data_dict['x'], data_dict['y'], data_dict['z'])

    def serialize(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }

