import json
import logging
import time
import base.heartbeat
import base.service
import planner.data_provider
from planner import dto
from typing import List
from utils.str_utils import (
    pretty_parse_object_or_furniture_str,
    remap_od_object_labels_to_ui_object_labels,
)
logger = logging.getLogger('planner_service')
logger.setLevel(logging.DEBUG)
logging.basicConfig(
    format='%(asctime)s %(name)-25s %(levelname)-8s %(message)s',
    datefmt='%H:%M:%S',
)

# with open('stub_data/next_action.json') as f:
#     actions = json.load(f)

# with open('stub_data/skill_feedback.json') as f:
#     actions_and_skill_feedback = json.load(f)

import rospy
from multiprocessing import Queue
import threading


class ActualPlannerDataProvider(planner.data_provider.PlannerDataProvider):
    __current_action_index = 0
    _plan_delay = 2
    is_instruction_complete: bool = True

    def __init__(self, *args, **kwargs):
        super().__init__(service=kwargs["service"])

        self.previous_instruction = "None"
        self.last_processed_timestamp = 0
        thread_subscriber = threading.Thread(target=self.rosparam_fetcher)
        thread_subscriber.start()

        self.instruction_queue = Queue()
        thread_subscriber2 = threading.Thread(target=self.rosparam_intr)
        thread_subscriber2.start()

        # Define the cancelling to get the signal from robot
        self.cancel_queue = Queue()
        thread_subscriber3 = threading.Thread(target=self.rosparam_cancel)
        thread_subscriber3.start()

    def extract_timestamp(self,str_timestamp):
        if str_timestamp == None or str_timestamp == "None":
            return 0.0
        return float(str_timestamp)

    def check_instruction_complete(self, new_instruction):
        return (self.previous_instruction != "None") and (new_instruction == "None")

    def rosparam_intr(self):
        while True:
            next_msg = self.instruction_queue.get()
            print("From instruction_queue", next_msg)
            if next_msg['msg'] == 'send_instruction':
                instruction = next_msg['data']
                print(f'Sending instruction over rospy: {instruction}')
                rospy.set_param("/llm_planner",instruction)

    def rosparam_cancel(self):
        while True:
            next_msg = self.cancel_queue.get()
            print("From cancel_queue", next_msg)
            if next_msg['msg'] == 'cancel_instruction':
                print('Sending cancel instruction over rospy')
                rospy.set_param("cancel",True)

    def beautify_next_action_str(self, msg_list:List[str]):
        # Element0 if this list is always timestamp, Element1 is skill name and Element2 is skill target
        assert len(msg_list) ==3, f"Invalid {msg_list=}. Cannot have more than 3 elements in this list"

        # Get skill name
        action_name = msg_list[1]
        beautiful_str = ""

        if "nav" in action_name or "Nav" in action_name:
            beautiful_str += f"I'm navigating to {pretty_parse_object_or_furniture_str(msg_list[2])}"
        elif "explore" in action_name:
            beautiful_str += f"I'm exploring {pretty_parse_object_or_furniture_str(msg_list[2])} and looking for objects"
        elif "pick" in action_name or "Pick" in action_name:
            object_name = remap_od_object_labels_to_ui_object_labels(
                pretty_parse_object_or_furniture_str(msg_list[2])
            )
            beautiful_str += f"I'm picking up {object_name}"
        elif "place" in action_name or "Place" in action_name:
            place_inputs = msg_list[2].split(';')
            object_name = remap_od_object_labels_to_ui_object_labels(
                pretty_parse_object_or_furniture_str(place_inputs[0])
            )
            relation = place_inputs[1]
            receptacle_name = pretty_parse_object_or_furniture_str(place_inputs[2].strip())
            beautiful_str += f"I'm placing {object_name} {relation} {receptacle_name}"
        elif "dock" in action_name or "Dock" in action_name:
            beautiful_str += "Plan complete!!\n Going to dock"
        else:
            beautiful_str += f"{action_name} called for {msg_list[2]}"

        return beautiful_str

    def beautify_har_str(self, msg_list:List[str]):
        # Element0 if this list is always timestamp, Element1 is action name and Element2 is object name, Element 3 is relationship , Element 4 is receptacle name
        assert len(msg_list) ==5, f"Invalid {msg_list=}. Cannot have more than 5 elements in this list"

        # Get skill name
        action_name = msg_list[1]
        beautiful_str = ""

        if "pick" in action_name or "Pick" in action_name:
            beautiful_str += f"Looks like you picked up the {pretty_parse_object_or_furniture_str(msg_list[2])}. I'm thinking..."
        elif "place" in action_name or "Place" in action_name:
            object_name = pretty_parse_object_or_furniture_str(msg_list[2].strip())
            relation =  msg_list[3].strip()
            receptacle_name = pretty_parse_object_or_furniture_str(msg_list[4].strip())
            beautiful_str += f"Looks like you placed the {object_name} {relation} {receptacle_name}. I'm thinking..."

        else:
            beautiful_str += f"{action_name} called for {msg_list[2]}"

        return beautiful_str

    def rosparam_fetcher(self):
        # Add condition
        while True:
            time.sleep(0.5)
            msg1 = rospy.get_param("/skill_name_input", f"{str(self.last_processed_timestamp)},None,None")
            msg2 = rospy.get_param("/skill_name_suc_msg", f"{str(self.last_processed_timestamp)},None,None,None")
            msg3 = rospy.get_param("/llm_planner", f"{str(self.last_processed_timestamp)},None")
            msg4 = rospy.get_param("/is_replanning", f"{str(self.last_processed_timestamp)},False")
            msg5 = rospy.get_param("/human_activity_ui", f"{str(self.last_processed_timestamp)},None,None,None,None")

            print("\n")
            print("skill_name_input : ", msg1)
            print("skill_name_suc_msg : ", msg2)
            print("llm_planner : ", msg3)
            print("is_replanning : ", msg4)
            print("human_activity_ui : ", msg5)
            print("\n")

            msg1_split = msg1.split(',')  # timestamp, skill_name, skill_input
            msg2_split = msg2.split(',')  # timestamp, skill_name, success_status, str msg
            msg3_split = msg3.split(',')  # timestamp, instruction
            msg4_split = msg4.split(',')  # timestamp, is_replanning
            msg5_split = msg5.split(',')  # timestamp, pick/place, obj name,on,receptacle name

            if len(msg1_split)!=3:
                print(f"Not receiving enough data for \skill_name_input rosparam. Current data size : {len(msg1_split)}")
            if len(msg2_split)!=4:
                print(f"Not receiving enough data for \skill_name_suc_msg rosparam. Current data size : {len(msg2_split)}")
            if len(msg3_split)<2: # This can be more than 2 but cannot be less than 2 (timestamp , instr)
                print(f"Not receiving enough data for \llm_planner rosparam. Current data size : {len(msg3_split)}")
            if len(msg4_split)!=2:
                print(f"Not receiving enough data for \is_replanning rosparam. Current data size : {len(msg4_split)}")
            if len(msg5_split)!=5:
                print(f"Not receiving enough data for \human_activity_ui rosparam. Current data size : {len(msg5_split)}")

            timestamp_msg1 = self.extract_timestamp(msg1_split[0])
            timestamp_msg2 = self.extract_timestamp(msg2_split[0])
            timestamp_msg3 = self.extract_timestamp(msg3_split[0])
            timestamp_msg4 = self.extract_timestamp(msg4_split[0])
            timestamp_msg5 = self.extract_timestamp(msg5_split[0])

            # Santize the nav name
            if "nav" in msg1_split[1]:
                msg1_split[1] = "nav"
                msg1_split[2] = msg1_split[2].strip("|").split(";")[-1]
            elif "explore" in msg1_split[1]:
                msg1_split[2] = msg1_split[2].strip("|").split('|')[-1].split(";")[-1].split(":")[0]
            if "nav" in msg2_split[1]:
                msg2_split[1] = "nav"
            # Sanitize instruction
            if len(msg3_split)>2:
                msg3_split = [msg3_split[0], ' '.join(msg3_split[1:])]

            strings_with_time = [
                (timestamp_msg1, 'skill_name_input', msg1_split),
                (timestamp_msg2, 'skill_name_suc_msg', msg2_split),
                (timestamp_msg3, 'is_instruction_complete', msg3_split),
                (timestamp_msg4, 'is_replanning', msg4_split),
                (timestamp_msg5, 'human_activity_ui', msg5_split)
            ]

            # Sort the list based on the timestamp (ascending order)
            sorted_strings = sorted(strings_with_time, key=lambda x: x[0])

            for msg_time, msg_topic, msg_string_list in sorted_strings:

                # Ignore if data is old
                if msg_time <= self.last_processed_timestamp:
                    continue

                # Update the last processed timestamp for next iteration
                self.last_processed_timestamp = msg_time

                if msg_topic == "skill_name_input" and msg_string_list[1] != "None":
                    # New action received
                    self._queue.put(
                        {
                            'msg': 'next_action',
                            'data': {
                                'action': self.beautify_next_action_str(msg_string_list),
                                'is_replan': False # TODO: Add this logic for replanning once Planner gives a parseable string
                            },
                        }
                    )

                    # Update skill feedback for new action
                    self._queue.put(
                        {
                            'msg': 'skill_feedback',
                            'data': {
                                'action': self.beautify_next_action_str(msg_string_list),
                                'feedback': "Starting " + msg_string_list[1] + " with goal " + msg_string_list[2],  # Parse through a parse for pretty msgs
                                'status': dto.ActionStatus.InProgress,
                                'is_instruction_complete': False,
                            },
                        }
                    )

                elif msg_topic == "skill_name_suc_msg" and msg_string_list[1] != "None":
                    self._queue.put(
                        {
                            'msg': 'skill_feedback',
                            'data': {
                                'action': msg_string_list[1],
                                'feedback': msg_string_list[3],
                                'status': dto.ActionStatus.Complete if msg_string_list[
                                                                        2] == "True" else dto.ActionStatus.Error,
                                'is_instruction_complete': False,
                            },
                        }
                    )

                elif msg_topic == "is_instruction_complete":
                    new_instruction = msg_string_list[1]
                    is_instruction_complete = self.check_instruction_complete(new_instruction)
                    if is_instruction_complete:
                        self._queue.put(
                            {
                                'msg': 'instruction_complete',
                                'data': True if msg_string_list[1] == "None" else False,
                                # Planner resets instruction at the end of run
                            }
                        )
                    self.previous_instruction = new_instruction

                elif msg_topic == "is_replanning":
                    is_replanning = msg_string_list[1] == "True"
                    if is_replanning:
                        self._queue.put(
                            {
                                'msg': 'is_replanning',
                                'data': True
                            }
                        )

                        # Reset replanning flag
                        rospy.set_param("/is_replanning", f"{str(time.time())},False")

                elif msg_topic == "human_activity_ui" and  msg_string_list[1] != "None":
                    self._queue.put(
                        {
                            'msg': 'human_activity',
                            'data': {
                                'action': self.beautify_har_str(msg_string_list)
                            },
                        }
                    )

                    # Reset  flag
                    rospy.set_param("/human_activity_ui", f"{str(time.time())},None,None,None,None")
                

                else:
                    self._queue.put({'msg': 'nothing_to_send'})

                print("\n")

    def process_cancel_for_ui(self):
        logger.debug('Cancelling')
        self.send_status(dto.PlannerStatus.IsCancelling)
        self.is_instruction_complete = True
        self.instruction = None
        time.sleep(1)
        logger.debug('Cancelled')
        self.send_status(dto.PlannerStatus.IsCancelled)
        self.send_next_action_message(data=self.get_next_action())

    def tick(self, delta):

        # Check if queue has anything to be processed
        if self._queue.empty():
            return

        next_msg = self._queue.get()
        if next_msg['msg'] == 'cancel_instruction':
            self.process_cancel_for_ui()
        elif next_msg['msg'] == 'skill_feedback':
            data = next_msg['data']
            print(f'Sending feedback for action: {data["action"]}')
            self.send_skill_feedback_message(data=self.get_skill_feedback(data=data))
        elif next_msg['msg'] == 'next_action':
            data = next_msg['data']
            print(f'Performing action: {data["action"]}')
            self.send_next_action_message(data=self.get_next_action(data=data))
        elif next_msg['msg'] == 'instruction_complete':
            if next_msg['data']:
                print('Plan finished')
                self.send_instruction_complete()
                self.is_instruction_complete = True
        elif next_msg['msg'] == 'is_replanning':
            print(f"Sending replanning", time.time())
            self.send_replanning()
        elif next_msg['msg'] == 'human_activity':
            data = next_msg['data']
            print("Sending Human Interruption Data",time.time())
            self.send_human_activity_planner(har=self.human_action_data(data=data))
        else:
            logger.warning(f"Received invalid msg in queue {next_msg}. {next_msg['msg']} not yet supported")

    def get_next_action(self, data=None) -> dto.NextActionData:
        if data is not None and data != {}:
            current_action = data['action']
            is_replan = data['is_replan']
        else:
            current_action = None
            is_replan = False
        print(f'Next action: {current_action} | {is_replan}')

        return dto.NextActionData(action=current_action, is_replanned=is_replan, display_message="")

    def get_skill_feedback(self, data=None) -> dto.ActionFeedbackData:
        if data is not None and data != {}:
            action = data['action']
            feedback = data['feedback']
            status = data['status']
            is_instruction_complete = data['is_instruction_complete']
        else:
            action = None
            feedback = ""
            status = dto.ActionStatus.NotSet
            is_instruction_complete = True
        print(f'Action: {action}, Feedback: {feedback}')
        return dto.ActionFeedbackData(action=action, feedback=feedback, status=status,
                                      is_instruction_complete=is_instruction_complete)

    def on_instruction(self, data=None):
        logger.debug(f'Processing instruction: {data}')
        instruction_str = data.get('data', {'data': None})['data']
        if instruction_str == "test_bad" or instruction_str == "None":
            self.send_instruction_not_understood('test_bad')
            return

        # Start processing the instruction. Send it to planner
        print("Trying to set llm_planner to ", instruction_str)
        instruction_str = f"{str(time.time())},{instruction_str}"

        self.instruction_queue.put(
            {
                'msg': 'send_instruction',
                'data': instruction_str,
            }
        )

    def human_action_data(self, data=None) -> dto.PlannerHARData:
        if data is not None and data != {}:
            action = data["action"]
            display_message = ""
            is_replanning = False
        else:
            action = None
            display_message = ""
            is_replanning =False
        print(f'Action: {action}, Display Message: {display_message}')
        return dto.PlannerHARData(action=action, is_replanned=is_replanning, display_message=display_message)

    def on_cancel(self, data=None):
        logger.debug('Processing cancel instruction request')

        cancel_instr_dict = {
            "msg": "cancel_instruction",
            "data": True,
        }
        # Start processing the cancel command to send to the planner & skills
        self.cancel_queue.put(cancel_instr_dict)

        # Start processing the cancel command to update the UI
        self._queue.put(cancel_instr_dict)
        
    def on_heartbeat(self) -> base.heartbeat.Heartbeat:
        return base.heartbeat.Heartbeat(status='available')

class ActualPlannerService(base.service.Service):
    host = '0.0.0.0'
    port = 7501
    data_provider_class = ActualPlannerDataProvider
    logger = logger


if __name__ == '__main__':
    ActualPlannerService().start()
