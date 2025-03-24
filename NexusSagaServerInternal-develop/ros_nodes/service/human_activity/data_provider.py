import abc

import human_activity.dto as dto

from base.data_provider import DataProvider

GET_HUMAN_ACTIVITY = 'get_human_activity'

LISTENERS = [GET_HUMAN_ACTIVITY]


class HumanActivityDataProvider(DataProvider):
    service_name = 'human_activity'

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._latest_heartbeat_status = None

    def register_listeners(self):
        self.on(GET_HUMAN_ACTIVITY, self.on_human_activity)

    @abc.abstractmethod
    def get_human_activity(self) -> dto.HumanActivityData:
        raise NotImplementedError()

    def send_human_activity_state(self, state: dto.HumanActivityData):
        try:
            data = state.to_json()
        except AttributeError as e:
            print(f"Error: {state} does not have a to_json method")
            data = state

        self.reply(json={
            'event': self.service_name,
            'type': 'human_activity',
            'data': data
        })

    def on_human_activity(self, data=None):
        self.send_human_activity_state(self.get_human_activity())
