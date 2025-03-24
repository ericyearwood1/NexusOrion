import unittest

from ws_server import main


class TestAllEventHandlers(unittest.TestCase):
    def test_all_events(self):
        client = main.socketio.test_client(main.app)
        client.emit(
            'random event',
            {'aaa': 'aaa'}
        )
