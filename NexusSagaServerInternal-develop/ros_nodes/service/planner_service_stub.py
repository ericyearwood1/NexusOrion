import json
import logging
import time

import base.heartbeat
import base.service
import planner.data_provider
from planner import dto

logger = logging.getLogger('planner_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

with open('stub_data/next_action.json') as f:
    actions = json.load(f)

with open('stub_data/skill_feedback.json') as f:
    actions_and_skill_feedback = json.load(f)


class StubPlannerDataProvider(planner.data_provider.PlannerDataProvider):
    __current_action_index = 0
    _plan_delay = 2
    actions_and_skill_feedback = actions_and_skill_feedback
    actions = actions
    _current_action_time = 0
    is_instruction_complete: bool = True
    instruction = None

    def cancel(self):
        logger.debug('Cancelling')
        self.send_status(dto.PlannerStatus.IsCancelling)
        self.__current_action_index = 0
        self._current_action_time = 0
        self.is_instruction_complete = True
        self.instruction = None
        time.sleep(1)
        logger.debug('Cancelled')
        self.send_status(dto.PlannerStatus.IsCancelled)
        data = dto.NextActionData(action="", display_message="", is_replanned=False)
        self.send_next_action_message(data=data)

    def tick(self, delta):
        if not self._queue.empty():
            self.instruction = self._queue.get()
            logger.debug(f'Instruction received: {self.instruction}')
            self.send_busy(dto.PlannerBusyData(current_instruction=self.instruction))

        if self.instruction is None and self.is_instruction_complete:
            return

        if self.instruction == planner.data_provider.CANCEL:
            self.cancel()
            return

        self.is_instruction_complete = False

        self._current_action_time += delta

        if self._current_action_time > self._plan_delay:
            print(
                f'Sending feedback for action: {self.actions_and_skill_feedback[self.__current_action_index]["action"]}')
            self.send_skill_feedback_message(data=self.get_skill_feedback())
            self.__current_action_index += 1
            self._current_action_time -= self._plan_delay
        else:
            return

        if self.__current_action_index < len(self.actions):
            print(f'Performing action: {self.actions[self.__current_action_index]["action"]}')
            self.send_next_action_message(data=self.get_next_action())
        else:
            print('Plan finished')
            self.send_instruction_complete()
            self.is_instruction_complete = True
            self.__current_action_index = 0
            self._current_action_time = 0
            self.instruction = None

    def get_next_action(self, data=None) -> dto.NextActionData:
        if self.is_instruction_complete:
            current_action = None
            is_replan = False

        else:
            current_action = actions[self.__current_action_index]['action']
            is_replan = True if ",replanning" in current_action else False
        print(f'Next action: {current_action} | {is_replan}')

        return dto.NextActionData(action=current_action, is_replanned=is_replan, display_message="")

    def get_skill_feedback(self, data=None) -> dto.ActionFeedbackData:
        current_action = actions_and_skill_feedback[self.__current_action_index]
        action = current_action['action']
        feedback = current_action['skill_feedback']
        status = dto.ActionStatus.Complete
        is_instruction_complete = True if "Final Thought: Exit!" in feedback else False
        print(f'Action: {action}, Feedback: {feedback}')
        return dto.ActionFeedbackData(action=action, feedback=feedback, status=status,
                                      is_instruction_complete=is_instruction_complete)

    def on_instruction(self, data=None):
        logger.debug(f'Processing instruction: {data}')

        instruction_data = dto.InstructionData.from_json(data.get('data', {'data': None}))
        if instruction_data.instruction == "test bad":
            self.send_instruction_not_understood(instruction_data)
            return
        elif instruction_data.instruction == "test fail":
            self.send_instruction_failed(instruction_data)
            return
        elif instruction_data.instruction == "" or instruction_data.instruction is None or instruction_data.instruction == "not understood":
            self.send_instruction_not_understood(instruction_data)
            return

        self._queue.put(instruction_data.instruction)

    def on_cancel(self, data=None):
        print('Cancel received')
        self._queue.put(planner.data_provider.CANCEL)

    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')


class StubPlannerService(base.service.Service):
    host = '0.0.0.0'
    port = 7501
    data_provider_class = StubPlannerDataProvider
    logger = logger


if __name__ == '__main__':
    StubPlannerService().start()
