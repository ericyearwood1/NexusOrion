import logging
import unittest.mock
from unittest.mock import MagicMock


import base.dto
import base.data_provider
import base.heartbeat
import base.service
from human_activity import dto as dto
from human_activity.data_provider import HumanActivityDataProvider


class TestBaseService(unittest.TestCase):

    def test_heartbeat(self):
        class MockedCustomDataProvider(HumanActivityDataProvider):
            def get_human_activity(self) -> dto.HumanActivityData:
                pass

            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._heartbeat_count = 0

            def register_listeners(self):
                pass

            def tick(self, delta):
                pass

            def on_heartbeat(self):
                if self._heartbeat_count < 2:
                    ret = base.heartbeat.Heartbeat(status='available')
                else:
                    ret = base.heartbeat.Heartbeat(status='unavailable')
                self._heartbeat_count += 1
                return ret

        class CustomService(base.service.Service):
            name = 'custom_service'
            host = '0.0.0.0'
            port = 7500
            logger = logging.getLogger('test')
            data_provider_class = MockedCustomDataProvider

        service = CustomService()
        dp = MockedCustomDataProvider(service=service)
        dp.reply = MagicMock()
        dp.heartbeat()
        self.assertEqual(dp.reply.call_count, 1)
        self.assertEqual(
            dp.reply.call_args.kwargs,
            {
                'event': 'message',
                'json': {
                    'service': 'human_activity',
                    'type': 'internal-status',
                    'status': 'available'
                }
            }
        )

        dp.heartbeat()
        # still one
        self.assertEqual(dp.reply.call_count, 1)

        dp.heartbeat()
        self.assertEqual(dp.reply.call_count, 2)
        self.assertEqual(
            dp.reply.call_args.kwargs,
            {
                'event': 'message',
                'json': {
                    'service': 'human_activity',
                    'type': 'internal-status',
                    'status': 'unavailable'
                }
            }
        )
