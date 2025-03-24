import os
import pickle

from local_multiplayer import Room, User


class MultiplayerState:
    _cached_path = 'config/multiplayer_state.pkl'

    def __init__(self):
        self.rooms = {'siro': Room('siro')}

    def find_user_with_sid(self, sid) -> User or None:
        # TODO what happens if the user exists in multiple rooms?
        #   although for know, I think it's hardcoded that we only have one room
        for room in self.rooms.values():
            if room.contains_user(sid):
                return room.get_user_by_sid(sid)
        return None

    def reset(self):
        self.rooms = {'siro': Room('siro')}

    def save(self):
        print(f"Saving multiplayer state to {self._cached_path}")
        with open(self._cached_path, 'wb') as f:
            pickle.dump(self, f)

    @classmethod
    def load(cls):
        if os.path.exists(cls._cached_path):
            print(f"Loading multiplayer state from {cls._cached_path}")
            with open(cls._cached_path, 'rb') as f:
                state = pickle.load(f)
            for room in state.rooms.values():
                room.reset_room()

            return state
        else:
            print(f"Multiplayer state file not found at {cls._cached_path}, creating new state")
            return cls()
