from dataclasses import dataclass
from enum import IntEnum, Enum


# Changes to data objects must be pushed upstream to the main repository
class ActionStatus(IntEnum):
    NotSet = 0
    InProgress = 1
    Complete = 2
    Error = 3

@dataclass
class NextActionData:
    action: str
    is_replanned: bool
    display_message: str
    child_actions: list = None

    def to_json(self):
        return {
            'action': self.action,
            'is_replanned': self.is_replanned,
            'display_message': self.display_message,
            'child_actions': [action.to_json() for action in self.child_actions] if self.child_actions else []
        }

    @staticmethod
    def from_json(json):
        return NextActionData(
            action=json.get('action'),
            display_message=json.get('display_message', ""),
            is_replanned=json.get('is_replanned'),
            child_actions=[NextActionData.from_json(action) for action in json.get('child_actions')] if json.get('child_actions') else []
        )


@dataclass
class ActionFeedbackData:
    action: str
    feedback: str
    status: ActionStatus
    is_instruction_complete: bool

    def to_json(self):
        return {
            'action': self.action,
            'feedback': self.feedback,
            'status': self.status,
            'is_instruction_complete': self.is_instruction_complete
        }

    @staticmethod
    def from_json(json):
        return ActionFeedbackData(
            action=json.get('action'),
            feedback=json.get('feedback'),
            status=json.get('status'),
            is_instruction_complete=json.get('is_instruction_complete')
        )

@dataclass
class CancelData:
    def to_json(self):
        return {}

    @staticmethod
    def from_json(json):
        return CancelData()


@dataclass
class InstructionData:
    user: str  # User that sent the instruction
    instruction: str  # Instruction string
    type: str

    def to_json(self):
        return {
            'data': self.instruction,
            'type': self.type,
            'user': self.user
        }

    @staticmethod
    def from_json(json):
        return InstructionData(
            instruction=json.get('data'),
            user=json.get('user'),
            type=json.get('type')
        )


@dataclass
class ReplanningData:

    def to_json(self):
        return {}

    @staticmethod
    def from_json(json):
        return ReplanningData()

class PlannerStatus(Enum):
    InProgress = "in_progress"
    IsCancelling = "is_cancelling"
    IsCancelled = "is_cancelled"


@dataclass
class StatusData:
    status: PlannerStatus

    def to_json(self):
        return {
            'status': self.status.value
        }

    @staticmethod
    def from_json(json):
        return StatusData(
            status=PlannerStatus(json.get('status'))
        )


@dataclass
class PlannerBusyData:
    current_instruction: str

    def to_json(self):
        return {
            'current_instruction': self.current_instruction
        }

    @staticmethod
    def from_json(json):
        return PlannerBusyData(
            current_instruction=json.get('current_instruction')
        )


@dataclass
class PlannerHARData:
    action: str
    is_replanned: bool
    display_message: str

    def to_json(self):
        return {
            'action': self.action,
            'is_replanned': self.is_replanned, # if is_replanned is True , UI will show replanning slate 
            'display_message': self.display_message,
        }

    @staticmethod
    def from_json(json):
        return NextActionData(
            action=json.get('action'),
            display_message=json.get('display_message', ""),
            is_replanned=json.get('is_replanned'),
        )
