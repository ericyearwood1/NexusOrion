class Color:
    def __init__(self, r, g, b, a):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

    @staticmethod
    def deserialize(data_dict):
        return Color(data_dict['r'], data_dict['g'], data_dict['b'], data_dict['a'])

    def serialize(self):
        return {
            'r': self.r,
            'g': self.g,
            'b': self.b,
            'a': self.a
        }
