from enum import IntEnum, Enum


class SpatialAnchorProgress(IntEnum):
    NoStatus = 0
    Initialising = 1
    SyncingWithCloud = 2
    SyncedWithCloud = 3
    Ready = 4
    Error = 5


class SpatialAnchorLoadState(IntEnum):
    NotSet = 0
    Sharing = 1
    Shared = 2
    Error = 3


class SpatialAnchorSaveState(IntEnum):
    NotSet = 0
    Saving = 1
    Saved = 2
    Loading = 3
    Loaded = 4
    Error = 5


class UserType(IntEnum):
    NotSpecified = 0
    Observer = 1
    Host = 2
    Guest = 3


class AugmentType(Enum):
    CUBE = 'CUBE'
    BALL = 'BALL'
    PYRAMID = 'PYRAMID'
