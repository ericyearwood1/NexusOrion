class RoomListMessage:

    def serialize(self):
        serialized_users = []
        try:
            serialized_users = [p.user.serialize() for p in self.users]
        finally:
            return {
                'connected_users': serialized_users
            }
