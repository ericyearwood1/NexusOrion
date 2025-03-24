import abc
from datetime import datetime
from typing import List

import planner.dto as dto

from base.data_provider import DataProvider

INSTRUCTION = 'instruction'
GET_NEXT_ACTION = 'get_next_action'
GET_SKILL_FEEDBACK = 'get_skill_feedback'
CANCEL = 'cancel'

LISTENERS = [INSTRUCTION, GET_NEXT_ACTION, GET_SKILL_FEEDBACK, CANCEL]


class PlannerDataProvider(DataProvider):
    service_name = 'planner'

    def register_listeners(self):
        self.on(INSTRUCTION, self.on_instruction)
        self.on(GET_NEXT_ACTION, self.on_get_next_action)
        self.on(GET_SKILL_FEEDBACK, self.on_get_skill_feedback)
        self.on(CANCEL, self.on_cancel)

    @abc.abstractmethod
    def get_next_action(self, data=None) -> dto.NextActionData:
        raise NotImplementedError()

    @abc.abstractmethod
    def get_skill_feedback(self, data=None) -> dto.ActionFeedbackData:
        raise NotImplementedError()

    @abc.abstractmethod
    def on_instruction(self, data=None):
        raise NotImplementedError()

    @abc.abstractmethod
    def on_cancel(self, data=None):
        raise NotImplementedError()

    def on_get_next_action(self, data=None):
        self.send_next_action_message(data=self.get_next_action(data))

    def on_get_skill_feedback(self, data=None):
        self.send_skill_feedback_message(data=self.get_skill_feedback(data))

    def send_next_action_message(self, data: dto.NextActionData):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'next_action',
                'timestamp': datetime.now().timestamp(),
                'data': data.to_json()
            }
        )

    def send_skill_feedback_message(self, data: dto.ActionFeedbackData):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'action_feedback',
                'timestamp': datetime.now().timestamp(),
                'data': {
                    'action': data.action,
                    'feedback': data.feedback,
                    'status': data.status,
                    'is_instruction_complete': data.is_instruction_complete
                }
            }
        )

    def send_instruction_not_understood(self, instruction: dto.InstructionData = None):
        if instruction is None:
            instruction = dto.InstructionData(instruction='None', type='None', user='')
        self.reply(
            json={
                'event': self.service_name,
                'type': 'instruction_not_understood',
                'timestamp': datetime.now().timestamp(),
                'data': instruction.to_json(),
            },
        )

    def send_instruction_complete(self, instruction: dto.InstructionData = None):
        if instruction is None:
            instruction = dto.InstructionData(instruction='None', type='None', user='')
        self.reply(
            json={
                'event': self.service_name,
                'type': 'instruction_complete',
                'timestamp': datetime.now().timestamp(),
                'data': instruction.to_json(),
            })

    def send_instruction_failed(self, instruction: dto.InstructionData = None):
        if instruction is None:
            instruction = dto.InstructionData(instruction='None', type='None', user='')
        self.reply(
            json={
                'event': self.service_name,
                'type': 'instruction_failed',
                'timestamp': datetime.now().timestamp(),
                'data': instruction.to_json(),
            })

    def send_compound_next_action_message(self, child_actions: List[str], is_replanning: bool = False):
        self.send_next_action_message(dto.NextActionData(action="Compound",
                                                         is_replanned=is_replanning,
                                                         child_actions=[dto.NextActionData(action=action, is_replanned=is_replanning) for action in child_actions]))

    def send_replanning(self):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'replanning',
                'timestamp': datetime.now().timestamp(),
            }
        )

    def send_status(self, status: dto.PlannerStatus):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'status',
                'timestamp': datetime.now().timestamp(),
                'data': dto.StatusData(status=status).to_json(),

            }
        )

    def send_busy(self, instruction: dto.PlannerBusyData):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'planner_is_busy',
                'timestamp': datetime.now().timestamp(),
                'data': instruction.to_json(),
            }
        )

    def send_human_activity_planner(self, har:dto.PlannerHARData):
        self.reply(
            json={
                'event': self.service_name,
                'type': 'human_activity',
                'timestamp': datetime.now().timestamp(),
                'data': har.to_json(),                
            }
        )
