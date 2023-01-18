"""A script for sending and receiving data from sensors on the OT3."""

import logging
import asyncio
import argparse
from dataclasses import dataclass
from numpy import float64
import termios
import sys
import tty
from typing import List, Tuple

from typing import Callable
from logging.config import dictConfig

from opentrons_hardware.firmware_bindings.messages.message_definitions import WriteMotorCurrentRequest
from opentrons_hardware.firmware_bindings.utils import UInt32Field
from opentrons_hardware.firmware_bindings.messages import payloads
from opentrons_hardware.drivers.can_bus.can_messenger import CanMessenger
from opentrons_hardware.firmware_bindings.constants import NodeId, PipetteTipActionType
from opentrons_hardware.scripts.can_args import add_can_args, build_settings
from opentrons_hardware.hardware_control.motion import (
    MoveGroupTipActionStep,
    MoveGroupSingleAxisStep,
    MoveStopCondition,
    create_home_step,
)
from opentrons_hardware.hardware_control.move_group_runner import MoveGroupRunner

from opentrons_hardware.drivers.can_bus.build import build_driver

from opentrons_shared_data.pipette.load_data import load_definition as load_pipette_definition
from opentrons_shared_data.pipette.pipette_definition import (
    PipetteVersionType, PipetteChannelType, PipetteModelType, PipetteTipType,
)

AccuracyAdjustTable = List[Tuple[float, float, float]]
# copy the lookup table from EVT single-channel pipettes
PIPETTE_SINGLE_EVT_VERSION = PipetteVersionType(major=3, minor=3)
PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM = 15.904


def _get_accuracy_adjust_table(tip_volume: int) -> AccuracyAdjustTable:
    _volume_to_tip = {50: PipetteTipType.t50, 200: PipetteTipType.t200, 1000: PipetteTipType.t1000}
    _tip = _volume_to_tip[tip_volume]
    _pipette = load_pipette_definition(
        PipetteModelType.p1000,
        PipetteChannelType.SINGLE_CHANNEL,
        PIPETTE_SINGLE_EVT_VERSION,
    )
    return _pipette.supported_tips[_tip].aspirate["default"]


def _get_plunger_distance_for_volume(table: AccuracyAdjustTable, volume: float) -> float:
    for i in range(len(table) - 1):
        this_entry = table[i]
        next_entry = table[i + 1]
        if this_entry[0] < volume < next_entry[0]:
            tv, ts, ti = this_entry
            print(f"\tv={tv}, s={ts}, i={ti}")
            ul_per_mm = (volume * ts) + ti
            mm_travel = volume / ul_per_mm
            return mm_travel
    raise ValueError(f"unable to find volume {volume} in table")


GetInputFunc = Callable[[str], str]
OutputFunc = Callable[[str], None]


class InvalidInput(Exception):
    """Invalid input exception."""
    pass


def _calc_time(distance, speed):
    time = abs(distance/speed)
    return time


async def set_current(messenger: CanMessenger, current: float, node: NodeId):
    await messenger.send(
        node_id=node,
        message=WriteMotorCurrentRequest(
            payload=payloads.MotorCurrentPayload(
                hold_current=UInt32Field(int(0 * (2**16))),
                run_current=UInt32Field(int(current * (2**16))),
            )
        ),
    )


def move_pipette_mechanism(distance, velocity):
    pipette_node = NodeId.pipette_left
    move = MoveGroupRunner(
        move_groups=[
            [
                {
                    pipette_node: MoveGroupTipActionStep(
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                        stop_condition=MoveStopCondition.none,
                        action=PipetteTipActionType.pick_up,
                    )
                }
            ]
        ]
    )
    return move


def home_pipette_jaw():
    velocity = 5.5
    distance = 40
    pipette_node = NodeId.pipette_left
    move = MoveGroupRunner(
        move_groups=[
            [
                {
                    pipette_node: MoveGroupTipActionStep(
                        velocity_mm_sec=float64(-velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                        stop_condition=MoveStopCondition.limit_switch,
                        action=PipetteTipActionType.pick_up,
                    )
                }
            ]
        ]
    )
    return move


def move_z_axis(distance, velocity):
    move_z = MoveGroupRunner(
        move_groups=[
            # Group 1
            [
                {
                    NodeId.head_l: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                    )
                }
            ],
        ]
    )
    return move_z


def move_x_axis(distance, velocity):
    move_x = MoveGroupRunner(
        move_groups=[
            # Group 1
            [
                {
                    NodeId.gantry_x: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                    )
                }
            ],
        ]
    )
    return move_x


def move_y_axis(distance, velocity):
    move_y = MoveGroupRunner(
        move_groups=[
            # Group 1
            [
                {
                    NodeId.gantry_y: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                    )
                }
            ],
        ]
    )
    return move_y


def move_plunger(distance, velocity):
    pipette_node = NodeId.pipette_left
    move_plunger_runner = MoveGroupRunner(
        # Group 0
        move_groups=[
            [
                {
                    pipette_node: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(_calc_time(distance,
                                                        velocity)),
                    )
                }
            ]
        ],
    )
    return move_plunger_runner


def home_z_axis():
    speed = 10.5
    home_z = MoveGroupRunner(
        move_groups=[
            [
                {
                    NodeId.head_l: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(-speed),
                        duration_sec=float64(100),
                        stop_condition=MoveStopCondition.limit_switch,
                    )
                }
            ]
        ]
    )
    return home_z


def home_gantry_xy():
    speed = 20
    home_z = MoveGroupRunner(
        move_groups=[
            [
                {
                    NodeId.gantry_x: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(-speed),
                        duration_sec=float64(100),
                        stop_condition=MoveStopCondition.limit_switch,
                    )
                }
            ],
            [
                {
                    NodeId.gantry_y: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(-speed),
                        duration_sec=float64(100),
                        stop_condition=MoveStopCondition.limit_switch,
                    )
                }
            ],
        ]
    )
    return home_z


def home_plunger():
    pipette_node = NodeId.pipette_left
    home_plunger_runner = MoveGroupRunner(
        move_groups=[
            [
                create_home_step(
                    {pipette_node: float64(100.0)},
                    {pipette_node: float64(-5)}
                )
            ]
        ]
    )
    return home_plunger_runner


def getch():
    """
        fd: file descriptor stdout, stdin, stderr
        This functions gets a single input keyboard character from the user
    """
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()


async def _jog_axis(messenger: CanMessenger, position, only_z=False) -> None:
    step_size = [0.1, 0.5, 1, 10, 20, 50]
    step_length_index = 3
    step = step_size[step_length_index]
    x_speed = 30
    y_speed = 60
    z_speed = 10.5
    x_pos = 0
    y_pos = 0
    z_pos = 0
    information_str = """
        Click  >>   i   << to move up
        Click  >>   k   << to move down
        Click  >>   a  << to move left
        Click  >>   d  << to move right
        Click  >>   w  << to move forward
        Click  >>   s  << to move back
        Click  >>   +   << to Increase the length of each step
        Click  >>   -   << to decrease the length of each step
        Click  >> Enter << to save position
        Click  >> q << to quit the test script
                    """
    print(information_str)
    while True:
        input = getch()
        if not only_z and input == 'a':
            # minus x direction
            sys.stdout.flush()
            x_pos = x_pos + step
            position['gantry_x'] = x_pos
            x_move = move_x_axis(step, x_speed)
            await x_move.run(can_messenger = messenger)

        elif not only_z and input == 'd':
            #plus x direction
            sys.stdout.flush()
            x_pos = x_pos - step
            position['gantry_x'] = x_pos
            x_move = move_x_axis(step, -x_speed)
            await x_move.run(can_messenger = messenger)

        elif not only_z and input == 'w':
            #minus y direction
            sys.stdout.flush()
            y_pos = y_pos - step
            position['gantry_y'] = y_pos
            y_move = move_y_axis(step, -y_speed)
            await y_move.run(can_messenger = messenger)

        elif not only_z and input == 's':
            #plus y direction
            sys.stdout.flush()
            y_pos = y_pos + step
            position['gantry_y'] = y_pos
            y_move = move_y_axis(step, y_speed)
            await y_move.run(can_messenger = messenger)

        elif input == 'i':
            sys.stdout.flush()
            z_pos = z_pos - step
            position['head_l'] = z_pos
            z_move = move_z_axis(step, -z_speed)
            await z_move.run(can_messenger = messenger)

        elif input == 'k':
            sys.stdout.flush()
            z_pos = z_pos + step
            position['head_l'] = z_pos
            z_move = move_z_axis(step, z_speed)
            await z_move.run(can_messenger = messenger)

        elif input == 'q':
            sys.stdout.flush()
            print("TEST CANCELLED")
            quit()

        elif input == '+':
            sys.stdout.flush()
            step_length_index = step_length_index + 1
            if step_length_index >= 5:
                step_length_index = 5
            step = step_size[step_length_index]

        elif input == '-':
            sys.stdout.flush()
            step_length_index = step_length_index -1
            if step_length_index <= 0:
                step_length_index = 0
            step = step_size[step_length_index]

        elif input == '\r' or input == '\n' or input == '\r\n':
            sys.stdout.flush()
            return position
        print('Coordinates: ', round(position['gantry_x'], 2), ',',
                                round(position['gantry_y'], 2), ',',
                                round(position['head_l'], 2), ' Motor Step: ',
                                step_size[step_length_index],
                                end = '')
        print('\r', end='')


@dataclass
class TestParams:
    tip_volume: int
    aspirate_volume_ul: float
    aspirate_speed_ul_per_sec: float
    leading_air_gap_ul: float
    trailing_air_gap_ul: float


TESTS = {
    "t50-1ul": TestParams(tip_volume=50,
                          aspirate_volume_ul=1.0,
                          aspirate_speed_ul_per_sec=20.0,
                          leading_air_gap_ul=15.0,
                          trailing_air_gap_ul=2.0),
    "t50-10ul": TestParams(tip_volume=50,
                           aspirate_volume_ul=10.0,
                           aspirate_speed_ul_per_sec=5.7,
                           leading_air_gap_ul=15.0,
                           trailing_air_gap_ul=0.1),
    "t50-50ul": TestParams(tip_volume=50,
                           aspirate_volume_ul=50.0,
                           aspirate_speed_ul_per_sec=44.2,
                           leading_air_gap_ul=15.0,
                           trailing_air_gap_ul=0.1),
    "t200-10ul": TestParams(tip_volume=200,
                            aspirate_volume_ul=10,
                            aspirate_speed_ul_per_sec=12.5,
                            leading_air_gap_ul=10,
                            trailing_air_gap_ul=5),
    "t200-50ul": TestParams(tip_volume=200,
                            aspirate_volume_ul=50,
                            aspirate_speed_ul_per_sec=37.5,
                            leading_air_gap_ul=10,
                            trailing_air_gap_ul=3.5),
    "t200-200ul": TestParams(tip_volume=200,
                             aspirate_volume_ul=200,
                             aspirate_speed_ul_per_sec=150,
                             leading_air_gap_ul=7.7,
                             trailing_air_gap_ul=2),
    "t1000-10ul": TestParams(tip_volume=1000,
                             aspirate_volume_ul=10,
                             aspirate_speed_ul_per_sec=22,
                             leading_air_gap_ul=10,
                             trailing_air_gap_ul=10),
    "t1000-50ul": TestParams(tip_volume=1000,
                             aspirate_volume_ul=50,
                             aspirate_speed_ul_per_sec=37,
                             leading_air_gap_ul=10,
                             trailing_air_gap_ul=10),
    "t1000-200ul": TestParams(tip_volume=1000,
                              aspirate_volume_ul=200,
                              aspirate_speed_ul_per_sec=150,
                              leading_air_gap_ul=7,
                              trailing_air_gap_ul=10),
    "t1000-500ul": TestParams(tip_volume=1000,
                              aspirate_volume_ul=500,
                              aspirate_speed_ul_per_sec=150,
                              leading_air_gap_ul=2,
                              trailing_air_gap_ul=10),
    "t1000-1000ul": TestParams(tip_volume=1000,
                               aspirate_volume_ul=1000,
                               aspirate_speed_ul_per_sec=150,
                               leading_air_gap_ul=2,
                               trailing_air_gap_ul=10),
}

PLUNGER_BOTTOM = 66
PLUNGER_BLOW_OUT = 71

DEFAULT_DISPENSE_SPEED_UL = 600
DEFAULT_TRAILING_SPEED_MM = 1  # mm/sec

GRAB_SPEED = 5.5
GRAB_DISTANCE = 19
DROP_SPEED = 5.5
DROP_DISTANCE = 29


async def _run(args: argparse.Namespace) -> None:
    driver = await build_driver(build_settings(args))
    messenger = CanMessenger(driver=driver)
    messenger.start()

    grab_tips = move_pipette_mechanism(GRAB_DISTANCE, GRAB_SPEED)
    drop_tips = move_pipette_mechanism(DROP_DISTANCE, DROP_SPEED)
    home_jaw = home_pipette_jaw()
    home_pipette = home_plunger()

    test = TESTS[args.test]
    table = _get_accuracy_adjust_table(test.tip_volume)
    aspirate_mm = _get_plunger_distance_for_volume(table, test.aspirate_volume_ul)
    aspirate_seconds = test.aspirate_volume_ul / test.aspirate_speed_ul_per_sec
    aspirate_speed_mm = aspirate_mm / aspirate_seconds
    leading_air_gap_mm = test.leading_air_gap_ul / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    trailing_air_gap_mm = test.trailing_air_gap_ul / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    dispense_mm = leading_air_gap_mm + aspirate_mm + trailing_air_gap_mm

    print(f"wet-air-gap-mm: {trailing_air_gap_mm}")
    print(f"aspirate-mm: {aspirate_mm}")
    print(f"dry-air-gap-mm: {leading_air_gap_mm}")
    print(f"total travelled: {trailing_air_gap_mm + aspirate_mm + leading_air_gap_mm} / {PLUNGER_BOTTOM}")

    try:
        # only use accuracy-adjustment when moving liquids
        print('HOMING Z')
        await home_z_axis().run(can_messenger=messenger)
        input("ENTER to home PLUNGER")
        await set_current(messenger, 1.5, NodeId.pipette_left)
        await home_pipette.run(can_messenger=messenger)
        input("ENTER to home JAW")
        await home_jaw.run(can_messenger=messenger)
        if "y" in input("Pick-up Tips? (y/n): ").lower():
            print("JOG to the TIPRACK")
            position = {'gantry_x': 0,
                        'gantry_y': 0,
                        'head_l': 0}
            tiprack_pos = await _jog_axis(messenger, position, only_z=True)
            print("grabbing the tips")
            await grab_tips.run(can_messenger=messenger)
            input("ENTER to home jaw")
            await home_jaw.run(can_messenger=messenger)
            input('ENTER to home Z')
            await home_z_axis().run(can_messenger=messenger)
        else:
            tiprack_pos = {'gantry_x': 0,
                           'gantry_y': 0,
                           'head_l': 0}
        # Prepare for aspirate --bottom
        await set_current(messenger, 1.5, NodeId.pipette_left)
        print('preparing for aspirate')
        await move_plunger(PLUNGER_BOTTOM, 10).run(can_messenger=messenger)
        print('aspirating LEADING-AIR-GAP')
        await move_plunger(trailing_air_gap_mm, 10).run(can_messenger=messenger)
        print('JOG to the TROUGH')
        position = {'gantry_x': tiprack_pos['gantry_x'],
                    'gantry_y': tiprack_pos['gantry_y'],
                    'head_l': 0}
        await _jog_axis(messenger, position, only_z=True)
        # Aspirate
        if args.pre_wet:
            pre_wet_count = 5
            for i in range(pre_wet_count):
                print(f"Pre-Wet {i + 1}/{pre_wet_count}")
                await move_plunger(aspirate_mm, -aspirate_speed_mm).run(can_messenger=messenger)
                await move_plunger(aspirate_mm, aspirate_speed_mm).run(can_messenger=messenger)
        print(f"aspirating: {aspirate_mm} mm")
        await move_plunger(aspirate_mm, -aspirate_speed_mm).run(can_messenger=messenger)
        print('JOG to the RETRACT position')
        await _jog_axis(messenger, position, only_z=True)
        print('doing a TRAILING-AIR-GAP')
        await move_plunger(leading_air_gap_mm, -DEFAULT_TRAILING_SPEED_MM).run(can_messenger=messenger)
        print('JOG to the DISPENSE position')
        await _jog_axis(messenger, position, only_z=True)
        print(f'dispensing: {dispense_mm} mm')
        dispense_speed_mm = DEFAULT_DISPENSE_SPEED_UL / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
        await move_plunger(dispense_mm, dispense_speed_mm).run(can_messenger=messenger)
        print('JOG to the BLOWOUT position')
        await _jog_axis(messenger, position, only_z=True)
        blow_out_plunger_distance_mm = PLUNGER_BLOW_OUT - PLUNGER_BOTTOM
        print(f'blowout: {blow_out_plunger_distance_mm} mm')
        await move_plunger(blow_out_plunger_distance_mm, 10).run(can_messenger=messenger)
        print('JOG to CATCH DROPLETS')
        await _jog_axis(messenger, position, only_z=True)
        print('homing Z Axis')
        await home_z_axis().run(can_messenger=messenger)
        input('ENTER to DROP-TIPS')
        await drop_tips.run(can_messenger=messenger)
        input('ENTER to home JAW')
        await home_jaw.run(can_messenger=messenger)
    except asyncio.CancelledError:
        pass
    finally:
        print("\nTesting finishes...\n")
        await messenger.stop()
        driver.shutdown()


log = logging.getLogger(__name__)

LOG_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "basic": {"format": "%(asctime)s %(name)s %(levelname)s %(message)s"}
    },
    "handlers": {
        "file_handler": {
            "class": "logging.handlers.RotatingFileHandler",
            "formatter": "basic",
            "filename": "/var/log/HT_tip_handling.log",
            "maxBytes": 5000000,
            "level": logging.INFO,
            "backupCount": 3,
        },
    },
    "loggers": {
        "": {
            "handlers": ["file_handler"],
            "level": logging.INFO,
        },
    },
}


def main() -> None:
    """Entry point."""
    dictConfig(LOG_CONFIG)

    parser = argparse.ArgumentParser(
        description="96 channel tip handling testing script."
    )
    add_can_args(parser)
    parser.add_argument(
        "--test",
        type=str,
        required=True,
        choices=list(TESTS.keys())
    )
    parser.add_argument("--pre-wet", action="store_true")
    args = parser.parse_args()

    asyncio.run(_run(args))


if __name__ == "__main__":
    main()
