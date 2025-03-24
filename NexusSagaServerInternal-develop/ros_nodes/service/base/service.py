import json
import traceback
import abc
import json as py_json
import multiprocessing
import time

import singletons.states
import ws_server


class Service(abc.ABC):
    enable_socketio_logger = False

    @property
    @abc.abstractmethod
    def host(self):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def port(self):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def data_provider_class(self):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def logger(self):
        raise NotImplementedError

    @property
    def ws_url(self):
        return f'ws://{self.host}:{self.port}'

    def __init__(self):
        self.__socketio = None
        self._data_provider = self.data_provider_class(service=self)
        self._received_messages_queue = ws_server.main.received_messages_queue
        self._messages_to_sent_queue = ws_server.main.messages_to_sent_queue
        self._latest_heartbeat_status = None

    def start(self):
        # must call this first so that self._listeners is populated,
        # it will be used in on_message_loop
        self._data_provider.register_listeners()
        multiprocessing.Process(target=self.tick).start()
        multiprocessing.Process(target=self.on_message_loop).start()

        self.__socketio = ws_server.main.socketio
        self.__socketio.start_background_task(self.messages_replier)
        self.logger.debug(f'Running WS server on {self.ws_url}')
        self.__socketio.run(
            ws_server.main.app,
            host=self.host,
            port=self.port,
        )

    def tick(self):
        """
        blocking code, should use sync sleep e.g. time.sleep
        """
        self.logger.debug(f'Tick started...')
        dt = 0
        while True:
            t = time.time()
            self._data_provider.heartbeat()
            self._data_provider.tick(delta=dt)
            # very short sleep to prevent 100% CPU
            time.sleep(0.001)
            dt = time.time() - t

    def on_message_loop(self):
        """
        blocking code, should use sync sleep e.g. time.sleep
        """
        self.logger.debug(f'on_message_loop started. Waiting for message...')
        while True:
            # sleep a bit so CPU is not 100%..
            time.sleep(0.05)
            messages = self.consume_messages()
            if messages:
                for message in messages:
                    try:
                        listener = self._data_provider.listeners[message.event]
                    except KeyError:
                        self.logger.critical(
                            f'Dropping message: {message} because the'
                            f' {message.event=} is not in registered listener.'
                        )
                        continue
                    self.logger.debug(f'Triggering message: {message} handled by registered listener {listener.__name__}')
                    multiprocessing.Process(
                        target=listener,
                        args=(message.data,)
                    ).start()

    def messages_replier(self):
        while True:
            if singletons.states.SIRO_SERVER_SID:
                while self._messages_to_sent_queue.empty() is False:
                    message = self._messages_to_sent_queue.get()
                    self.__socketio.emit(
                        message.event,
                        message.data,
                        to=singletons.states.SIRO_SERVER_SID
                    )
            # important, do not remove, voluntarily free thread to others
            self.__sleep(0)

    def __sleep(self, seconds):
        """should only be used in a background task"""
        self.__socketio.sleep(seconds)

    def consume_messages(self):
        messages = []
        while self._received_messages_queue.empty() is False:
            messages.append(self._received_messages_queue.get())
        return messages

    def reply(self, event='message', message=None, json: dict = None):
        """json should be a dict, which will be converted to json"""
        type = 'unknown'
        if json:
            type = json.get('type', 'unknown')
            message = py_json.dumps(json)
        self._messages_to_sent_queue.put(
            ws_server.main.MessageInQueue(
                event=event,
                data=message,
                type=type
            )
        )
