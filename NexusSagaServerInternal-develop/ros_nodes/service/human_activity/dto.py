from dataclasses import dataclass

# Changes to data objects must be pushed upstream to the main repository

@dataclass
class HumanActivityData:
    activity: str
    object: str
    receptacle: str

    def __init__(self, activity: str, _object: str = None, receptacle: str = None):
        self.activity = activity
        self.object = _object
        self.receptacle = receptacle

    def to_json(self):
        return {
            'activity': self.activity,
            'object': self.object,
            'receptacle': self.receptacle
        }

    @staticmethod
    def from_json(json):
        return HumanActivityData(
            activity=json.get('activity', ""),
            _object=json.get('object', None),
            receptacle=json.get('receptacle', None)
        )
