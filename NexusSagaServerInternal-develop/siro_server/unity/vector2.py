class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def deserialize(data_dict):
        return Vector2(data_dict['x'], data_dict['y'])

    def serialize(self):
        return {
            'x': self.x,
            'y': self.y
        }
