from dataclasses import dataclass


@dataclass
class User:
    user_id: str
    room: str
    display_name: str
    color: str
    sid: str
    head_entity_id: str
    user_type: str


    @staticmethod
    def deserialize(data_dict):
        return User(**data_dict)
