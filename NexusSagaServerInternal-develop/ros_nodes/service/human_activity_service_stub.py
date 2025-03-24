import json
import logging
import random

import base.heartbeat
import base.service
import human_activity.data_provider
from human_activity import dto


logger = logging.getLogger('human_activity_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)
DEFAULT_STATE = "nothing"

with open('stub_data/human_activity.json') as f:
    human_activity_messages = json.load(f)


class HumanActivityDataProviderStub(human_activity.data_provider.HumanActivityDataProvider):
    human_activity_messages = human_activity_messages
    __current_message_index = 0
    __advance_message_chance_per_second = 0.05

    def get_human_activity(self) -> dto.HumanActivityData:
        activity_parts = self.human_activity_messages[self.__current_message_index]['currentActivity'].split(',')
        while len(activity_parts) < 3:
            activity_parts.append(None)

        return dto.HumanActivityData(activity=activity_parts[0],
                                     _object=activity_parts[1],
                                     receptacle=activity_parts[2])

    def tick(self, delta):
        if random.random() < self.__advance_message_chance_per_second * delta:
            self.__current_message_index += 1
            if self.__current_message_index >= len(self.human_activity_messages):
                self.__current_message_index = 0

            self.on_human_activity(None)

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

class HumanActivityServiceStub(base.service.Service):
    host = '0.0.0.0'
    port = 7502
    data_provider_class = HumanActivityDataProviderStub
    logger = logger


if __name__ == '__main__':
    HumanActivityServiceStub().start()
