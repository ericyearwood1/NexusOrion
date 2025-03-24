import abc
from multiprocessing import Queue

import base.heartbeat
from base.service import Service


class DataProvider(abc.ABC):
    def __init__(self, service: Service):
        self.listeners = {}
        self._service = service
        self._queue = Queue()
        self._latest_heartbeat_status = None

    @property
    @abc.abstractmethod
    def service_name(self) -> str:
        raise NotImplementedError()

    @abc.abstractmethod
    def register_listeners(self):
        raise NotImplementedError

    @abc.abstractmethod
    def tick(self, delta: float):
        """
        Called by the service start()

        You would still need to implement this even tick is not used
        in your provider, just return pass
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        raise NotImplementedError()

    def reply(self, event='message', message=None, json=None):
        """
        All events are messages to follow what's in Unity already
        """
        self._service.reply(
            event,
            message,
            json
        )

    def on(self, event: str, callback: callable):
        self.listeners[event] = callback

    def heartbeat(self):
        heartbeat = self.on_heartbeat()
        self._send_heartbeat(service_name=self.service_name, heartbeat=heartbeat)

    def _send_heartbeat(self, service_name, heartbeat):
        if heartbeat.status != self._latest_heartbeat_status:
            self._latest_heartbeat_status = heartbeat.status
            self.reply(
                event='message',
                json={
                    'service': service_name,
                    'type': 'internal-status',
                    'status': heartbeat.status,
                }
            )
