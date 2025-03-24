import json
import random
import unittest
import uuid

import local_multiplayer
import main
import handlers.multiplayer_state


class TestLocalMultiplayerState(unittest.TestCase):
    maxDiff = 2000

    def generate_user(
        self,
        sid
    ):
        return {
            'user_id': random.randint(0, 10000),
            'sid': sid,
            'display_name': 'display_name',
            'color': 'color',
            'room': 'siro',
            'head_entity_id': 1,
            'user_type': 'player'
        }

    def generate_anchor(self, anchor_type='CoLocation'):
        # anchor_type or 'RobotHome'
        return {
            'name': 'anchor',
            'anchor_type': anchor_type,
            'uuid': str(uuid.uuid4()),
            'created_by': random.randint(0, 10000),
            'room': 'siro',
            'shared_with': [random.randint(0, 10000)]
        }

    def test_get_room(self):
        room = handlers.multiplayer_state.get_room(data={
            'room': 'siro'
        })
        self.assertTrue(room)

    def test_on_connect(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)

        # should receive 1 service status messages,
        # the final one is on connect message
        receiveds = client.get_received()

        for received in receiveds[:1]:
            event = received['name']
            self.assertEqual('service_status', event)

            message = json.loads(received['args'][0])
            self.assertEqual(
                message,
                {
                    'code': 'planner_unavailable',
                    'data': None,
                    'details': None,
                    'status': 'ok'
                }
            )

        on_connect_received = receiveds[-1]
        event = on_connect_received['name']
        # FIXME NOTICE, if event is not message, args will be a list
        message = json.loads(on_connect_received['args'])
        self.assertEqual('message', event)
        self.assertEqual(
            message,
            {
                'event': 'multiplayer',
                'type': 'initial_state',
                'sid': client.sid,
                'rooms': ['siro']
            }
        )

    def test_on_join(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()
        user = self.generate_user(sid=client.sid)
        client.emit(
            'join_room',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        receiveds = client.get_received()
        room_state_message = receiveds[0]
        self.assertEqual(room_state_message['name'], 'message')
        room_state_message = json.loads(room_state_message['args'])
        self.assertEqual(
            room_state_message,
            {
                "event": "multiplayer",
                "type": "room_state",
                "room": {
                    "id": "siro",
                    "space_sync_anchor": None,
                    "robot_home_anchor": None,
                    "connected_users": [user],
                "is_robot_feature_available": True,
                "is_augment_feature_available": True,
                "augments": []}
            }
        )
        user_found = False
        for _user in local_multiplayer.state.rooms['siro'].users:
            if _user.user_id == user['user_id']:
                user_found = True
                break
        assert user_found

    def test_on_anchor_update(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()
        user = self.generate_user(sid=client.sid)
        client.emit(
            'spatial_anchor_update',
            json.dumps({
                'room': 'siro',
                'anchor_data': self.generate_anchor(),
                'user': user
            })
        )
        # receiveds = client.get_received()
        # room_state_message = receiveds[0]
        # self.assertEqual(room_state_message['name'], 'message')
        # room_state_message = json.loads(room_state_message['args'])
        # self.assertEqual(
        #     room_state_message,
        #     {
        #         "event": "multiplayer",
        #         "type": "room_state",
        #         "room": {
        #             "id": "siro",
        #             "space_sync_anchor": None,
        #             "robot_home_anchor": None,
        #             "connected_users": [user],
        #             "is_robot_feature_available": True,
        #             "is_augment_feature_available": True,
        #             "augments": []}
        #     }
        # )
        assert local_multiplayer.state.rooms['siro'].space_sync_anchor

        client.emit(
            'spatial_anchor_update',
            json.dumps({
                'room': 'siro',
                'anchor_data': self.generate_anchor(anchor_type='RobotHome'),
                'user': user
            })
        )

        assert local_multiplayer.state.rooms['siro'].robot_home_anchor

    def test_on_user_edit(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()
        user = self.generate_user(sid=client.sid)
        # join room first
        client.emit(
            'join_room',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        # clear out join room messages
        receiveds = client.get_received()

        user['display_name'] = 'updated'
        client.emit(
            'user_details',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        self.assertEqual(
            local_multiplayer.state.rooms['siro'].users[0].display_name,
            'updated'
        )

    def test_on_create_augment(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()

        # join room firs
        user = self.generate_user(sid=client.sid)
        client.emit(
            'join_room',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        # clear out join room messages
        receiveds = client.get_received()

        self.assertFalse(
            local_multiplayer.state.rooms['siro'].augments
        )

        client.emit(
            'create_augment',
            json.dumps({
                'room': 'siro',
                'augment': {
                    'augment_type': 'CoLocation',
                    'x': 1,
                    'y': 1,
                    'z': 1,
                    'rx': 0,
                    'ry': 0,
                    'rz': 0,
                    'scale': 1
                }
            })
        )

        self.assertTrue(
            local_multiplayer.state.rooms['siro'].augments
        )

    def test_on_delete_augment(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()

        # join room firs
        user = self.generate_user(sid=client.sid)
        client.emit(
            'join_room',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        # clear out join room messages
        receiveds = client.get_received()
        client.emit(
            'create_augment',
            json.dumps({
                'room': 'siro',
                'augment': {
                    'augment_type': 'CoLocation',
                    'x': 1,
                    'y': 1,
                    'z': 1,
                    'rx': 0,
                    'ry': 0,
                    'rz': 0,
                    'scale': 1
                }
            })
        )

        # clear out create augment messages
        receiveds = client.get_received()
        _uuid = list(local_multiplayer.state.rooms['siro'].augments.keys())[0]

        client.emit(
            'delete_augment',
            json.dumps({
                'room': 'siro',
                'augment': {
                    'uuid': _uuid,
                }
            })
        )

        self.assertFalse(local_multiplayer.state.rooms['siro'].augments)

    def test_on_update_augment(self):
        local_multiplayer.state.reset()
        client = main.sio.test_client(main.app)
        # clear out on connect messages
        receiveds = client.get_received()

        # join room firs
        user = self.generate_user(sid=client.sid)
        client.emit(
            'join_room',
            json.dumps({
                'room': 'siro',
                'user': user
            })
        )
        # clear out join room messages
        receiveds = client.get_received()
        client.emit(
            'create_augment',
            json.dumps({
                'room': 'siro',
                'augment': {
                    'augment_type': 'CoLocation',
                    'x': 1,
                    'y': 1,
                    'z': 1,
                    'rx': 0,
                    'ry': 0,
                    'rz': 0,
                    'scale': 1
                }
            })
        )

        # clear out create augment messages
        receiveds = client.get_received()
        _uuid = list(local_multiplayer.state.rooms['siro'].augments.keys())[0]

        client.emit(
            'update_augment',
            json.dumps({
                'room': 'siro',
                'augment': {
                    'uuid': _uuid,
                    'x': 2,
                    'y': 2,
                    'z': 2,
                    'rx': 1,
                    'ry': 1,
                    'rz': 1,
                    'scale': 2
                }
            })
        )

        augment = local_multiplayer.state.rooms['siro'].augments[_uuid]
        self.assertEqual(augment.x, 2)
        self.assertEqual(augment.y, 2)
        self.assertEqual(augment.z, 2)
        self.assertEqual(augment.rx, 1)
        self.assertEqual(augment.ry, 1)
        self.assertEqual(augment.rz, 1)
        self.assertEqual(augment.scale, 2)
