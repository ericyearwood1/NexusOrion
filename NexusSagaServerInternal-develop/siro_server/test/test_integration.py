import copy
import json
import socket
import sys
import unittest
import socketio

import main


class TestIntegration(unittest.TestCase):

    @staticmethod
    def is_port_open():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            try:
                s.connect(('localhost', 7500))
                return True
            except Exception:
                return False

    @classmethod
    def setUpClass(cls):
        # check if the siro_server is running
        if cls.is_port_open() is False:
            raise ConnectionError(
                'siro_server is not running on port 7500. To do integration test, '
                "it's recommended to start the services using docker-compose. "
                "Please refer to Readme how to start it."
            )
        # FIXME connect/disconnect cannot be tested?
        cls._sio = socketio.Client()
        cls._sio.connect('http://localhost:7500')

    @classmethod
    def tearDownClass(cls):
        cls._sio.disconnect()

    def setUp(self):
        self._messages = []

    def tearDown(self):
        self._messages = []

    def get_messages(self):
        ret = copy.deepcopy(self._messages)
        self._messages = []
        return ret

    def test_integration(self):
        @self._sio.on('message')
        def on_message(message):
            self._messages.append(json.loads(message))

        self._sio.emit('get_human_activity')
        self._sio.sleep(1)
        messages = self.get_messages()
        self.assertIn(
            {
                'activity': 'Standing',
                'object': None,
                'receptacle': None,
                'event': 'human_activity',
                'type': 'human_activity'
            },
            messages
        )
