"""Module that provides mapping for status property"""
from dataclasses import dataclass


@dataclass
class IjaiVacuumStatusMapping:
    """Dataclass containing mapping for status property"""
    # vacuum service id
    siid: int = 2

    # status property id in vacuum service
    piid: int = 1

    # idle_at is status property values from https://home.miot-spec.com/spec/model
    # 0,1,2,4,8,10 are common idle states for most ijai/xiaomi miot robot-vacuums
    idle_at: tuple[int, ...] = (0, 1, 2, 4, 8, 10)


_NON_STANDARD_STATUS_PROP = [
    (
        [
            "xiaomi.vacuum.c107",
            "xiaomi.vacuum.d101",
            "xiaomi.vacuum.d102gl",
            "xiaomi.vacuum.d102ev",
            "xiaomi.vacuum.d109gl",
        ],
        IjaiVacuumStatusMapping(idle_at=(1, 2, 5, 9, 11, 12, 13, 14, 15, 18))
    ),

    (
        [
            "xiaomi.vacuum.c108"
        ],
        IjaiVacuumStatusMapping(idle_at=(1, 3, 4, 5, 7))
    ),

    (
        [
            "xiaomi.vacuum.b108gl"
        ],
        IjaiVacuumStatusMapping(idle_at=(1, 2, 5, 8, 10))
    ),

    (
        [
            "xiaomi.vacuum.c102gl",
            "xiaomi.vacuum.c102cn",
            "xiaomi.vacuum.d103cn",
            "xiaomi.vacuum.d110ch",
        ],
        IjaiVacuumStatusMapping(piid=2,
                                idle_at=(2, 3, 4, 6, 8, 9, 13, 19, 21, 22, 30))
    )
]

_IsEncryptKeyTypeHex_Models = [
    "v1",
    "v2",
    "v3",
    "v13",
    "v15",
    "v17",
    "v18",
    "v19",
    "sz02",  # unknown model
    "c101",
    "c103",
    "c104",
    "c101eu",
    "b106eu",
    "b106tr",
]

_K3_Models = [
    "v13",
    "v14",
    "v15",
    "v17",
    "v18",
    "v19",
    "v21",
    "sz02",  # unknown model
    "c101",
    "c103",
    "c104",
    "b106eu",
    "b106tr",
]


def is_K3_model(model: str):
    return model.split(".")[-1].lower() in _K3_Models


def is_EncryptKeyTypeHex_model(model: str):
    return model.split(".")[-1].lower() in _IsEncryptKeyTypeHex_Models


def get_status_mapping(model: str) -> IjaiVacuumStatusMapping:
    return next((mapping for models,
                 mapping in _NON_STANDARD_STATUS_PROP if model in models),
                IjaiVacuumStatusMapping())
