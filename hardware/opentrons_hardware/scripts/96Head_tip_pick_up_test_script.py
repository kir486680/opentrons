"""A script for sending and receiving data from sensors on the OT3."""

import logging
import asyncio
import argparse
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

def prompt_int_input(prompt_name: str) -> int:
    """Prompt to choose a member of the enum.

    Args:
        output_func: Function to output text to user.
        get_user_input: Function to get user input.
        enum_type: an enum type

    Returns:
        The choice.

    """
    try:
        return int(input(f"{prompt_name}: "))
    except (ValueError, IndexError) as e:
        raise InvalidInput(e)

def output_details(i: int, total_i: int) -> None:
    """Print out test details."""
    print(f"\n\033[95mRound {i}/{total_i}:\033[0m")

def calc_time(distance, speed):
    time = abs(distance/speed)
    # print(time)
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

async def hold_current(messenger: CanMessenger, current: float, node: NodeId):
    await messenger.send(
        node_id=node,
        message=WriteMotorCurrentRequest(
            payload=payloads.MotorCurrentPayload(
                hold_current=UInt32Field(int(current * (2**16))),
                run_current=UInt32Field(int(0 * (2**16))),
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
                        duration_sec=float64(calc_time(distance,
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
                        duration_sec=float64(calc_time(distance,
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
                        duration_sec=float64(calc_time(distance,
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
                        duration_sec=float64(calc_time(distance,
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
                        duration_sec=float64(calc_time(distance,
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
                        duration_sec=float64(calc_time(distance,
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
        # print(f'Coordinates: X: {round(position['gantry_x'], 2)}\
        #                     Y: {round(position['gantry_y'], 2)} \
        #                     Z: {round(position['head_l'], 2)}, \
        #                     Motor Step: {step_size[step_length_index]}',
        #                     end='')
        print('\r', end='')


async def run(args: argparse.Namespace) -> None:
    """Entry point for script."""
    print("Test tip pick up for the 96 channel\n")
    delay = 0.5
    # 96 channel can only be mounted to the left
    pipette_node = NodeId.pipette_left
    driver = await build_driver(build_settings(args))
    messenger = CanMessenger(driver=driver)
    messenger.start()
    z_speed = 10
    x_speed = 30
    y_speed = 60
    pick_up_speed = 5
    grap_speed = 5.5
    grap_distance = 19
    drop_speed = 5.5
    drop_distance = 29
    pick_up_distance = 12
    calibrate = True
    trough_calibrate = True
    press = True
    grab_tips = move_pipette_mechanism(grap_distance, grap_speed)
    drop_tips = move_pipette_mechanism(drop_distance, drop_speed)
    home_jaw = home_pipette_jaw()
    home_z = home_z_axis()
    home_gantry = home_gantry_xy()
    home_pipette = home_plunger()
    move_y = move_y_axis(332, 60)
    move_x = move_x_axis(250, 30)
    move_z = move_z_axis(pick_up_distance, z_speed)
    try:
        #-------------------partial Pick up-------------------------------------
        if args.test == 'partial_pick_up':
            input_current = float(input("Enter Current: "))
            await home_z.run(can_messenger = messenger)
            await asyncio.sleep(3)
            await home_gantry.run(can_messenger = messenger)
            await asyncio.sleep(delay)
            if calibrate:
                position = {'gantry_x': 0,
                            'gantry_y': 0,
                            'head_l': 0}
                current_position = await _jog_axis(messenger, position)
            else:
                current_position = {'gantry_x': 249.7,
                                    'gantry_y': 332.7,
                                    'head_l': 151}
                await move_y_axis(current_position['gantry_y'], y_speed).run(can_messenger = messenger)
                await asyncio.sleep(delay)
                await move_x_axis(current_position['gantry_x'], x_speed).run(can_messenger = messenger)
                await asyncio.sleep(delay)
                await move_z_axis(current_position['head_l'], z_speed).run(can_messenger = messenger)
                await asyncio.sleep(delay)
            await set_current(messenger, input_current, NodeId.head_l)
            await hold_current(messenger, 1.4, NodeId.gantry_y)
            await hold_current(messenger, 1.4, NodeId.gantry_x)
            await move_z_axis(pick_up_distance, pick_up_speed).run(can_messenger = messenger)
            current_position['head_l'] = current_position['head_l'] + pick_up_distance
            await asyncio.sleep(delay)
            if press:
                slower_speed = 1
                await set_current(messenger, 1.5, NodeId.head_l)
                await move_z_axis(5, -slower_speed).run(can_messenger = messenger)
                current_position['head_l'] = current_position['head_l'] + pick_up_distance
                await asyncio.sleep(delay)
                await set_current(messenger, input_current+0.05, NodeId.head_l)
                await hold_current(messenger, 1.4, NodeId.gantry_y)
                await hold_current(messenger, 1.4, NodeId.gantry_x)
                await move_z_axis(5+2, slower_speed).run(can_messenger = messenger)
                current_position['head_l'] = current_position['head_l'] + pick_up_distance
                await asyncio.sleep(delay)
            await home_z.run(can_messenger = messenger)
            await asyncio.sleep(3)
            # set current for plunger
            await set_current(messenger, 2.2,NodeId.pipette_left)
            await home_pipette.run(can_messenger = messenger)
            await asyncio.sleep(delay)
            # set current for plunger
            await set_current(messenger, 1.5,NodeId.pipette_left)
            # prepare for aspirate
            await move_plunger(63 ,5).run(can_messenger = messenger)
            await asyncio.sleep(delay)
            current_position['head_l'] = 0
            if trough_calibrate:
                trough_pos = await _jog_axis(messenger, current_position)
            else:
                current_position = {'gantry_x': 251.9,
                                'gantry_y': 329.2,
                                'head_l': 151}
            # Aspirate
            await move_plunger(60, -5).run(can_messenger = messenger)
            await asyncio.sleep(delay)
            await home_z.run(can_messenger = messenger)
            await asyncio.sleep(3)
            input('Press Enter to continue!!')
            await move_z_axis(trough_pos['head_l'], z_speed).run(can_messenger = messenger)
            # Dispense + Blow out
            await asyncio.sleep(delay)
            await move_plunger(65, 5).run(can_messenger = messenger)
            await asyncio.sleep(delay)
            await home_z.run(can_messenger = messenger)
            await asyncio.sleep(3)
            await set_current(messenger, 2.2,NodeId.pipette_left)
            await home_pipette.run(can_messenger = messenger)
            await asyncio.sleep(delay)
        #-------------------Pick up tip Motors----------------------------------
        elif args.test == 'full_pick_up_tip':
            bottom_pos = 66
            blow_out_pos = 71
            plunger_speed_mm = 5  # mm/sec
            trailing_air_gap_speed_mm = 1  # mm/sec
            blow_out_plunger_distance_mm = blow_out_pos - bottom_pos
            vol_str = input("Enter volume for WET air-gap (submerged blow-out): ").strip()
            if vol_str:
                wet_air_gap_mm = float(vol_str) / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
            else:
                wet_air_gap_mm = 0
            vol_str = input("Enter volume for DRY air-gap (carrying air-gap): ").strip()
            if vol_str:
                dry_air_gap_mm = float(vol_str) / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
            else:
                dry_air_gap_mm = 0
            vol_str = input("Enter volume for ASPIRATE volume: ").strip()
            aspirate_ul = float(vol_str)
            tip_volume = int(input("Enter tip size (50, 200, or 1000): "))
            table = _get_accuracy_adjust_table(tip_volume)
            # only use accuracy-adjustment when moving liquids
            aspirate_mm = _get_plunger_distance_for_volume(table, aspirate_ul)
            dispense_mm = dry_air_gap_mm + aspirate_mm + wet_air_gap_mm
            print(f"wet-air-gap-mm: {wet_air_gap_mm}")
            print(f"aspirate-mm: {aspirate_mm}")
            print(f"dry-air-gap-mm: {dry_air_gap_mm}")
            print(f"total travelled: {wet_air_gap_mm + aspirate_mm + dry_air_gap_mm} / {bottom_pos}")
            should_pre_wet = ("y" in input("Pre-wet x5 times? (y/n): "))
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
            await move_plunger(bottom_pos, 10).run(can_messenger=messenger)
            print('aspirating LEADING-AIR-GAP')
            await move_plunger(wet_air_gap_mm, 10).run(can_messenger=messenger)
            print('JOG to the TROUGH')
            position = {'gantry_x': tiprack_pos['gantry_x'],
                        'gantry_y': tiprack_pos['gantry_y'],
                        'head_l': 0}
            await _jog_axis(messenger, position, only_z=True)
            # Aspirate
            if should_pre_wet:
                pre_wet_count = 5
                for i in range(pre_wet_count):
                    print(f"Pre-Wet {i + 1}/{pre_wet_count}")
                    await move_plunger(aspirate_mm, -plunger_speed_mm).run(can_messenger=messenger)
                    await move_plunger(aspirate_mm, plunger_speed_mm).run(can_messenger=messenger)
            print(f"aspirating: {aspirate_mm} mm")
            await move_plunger(aspirate_mm, -plunger_speed_mm).run(can_messenger=messenger)
            print('JOG to the RETRACT position')
            await _jog_axis(messenger, position, only_z=True)
            print('doing a TRAILING-AIR-GAP')
            await move_plunger(dry_air_gap_mm, -trailing_air_gap_speed_mm).run(can_messenger=messenger)
            print('JOG to the DISPENSE position')
            await _jog_axis(messenger, position, only_z=True)
            print(f'dispensing: {dispense_mm} mm')
            await move_plunger(dispense_mm, plunger_speed_mm).run(can_messenger=messenger)
            print('JOG to the BLOWOUT position')
            await _jog_axis(messenger, position, only_z=True)
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

        elif args.test == 'aspirate/dispense':
            for cycle in range(1, args.cycles+1):
                #await home_z.run(can_messenger = messenger)
                await asyncio.sleep(3)
                #await home_gantry.run(can_messenger = messenger)
                await asyncio.sleep(delay)
                await set_current(messenger, 1.5,NodeId.pipette_left)
                await home_pipette.run(can_messenger=messenger)
                await asyncio.sleep(delay)
                #await home_jaw.run(can_messenger=messenger)
                #await asyncio.sleep(delay)
                # full travel of the plunger is 71mm
                # Prepare for aspirate
                await move_plunger(66,10).run(can_messenger = messenger)
                await asyncio.sleep(delay)
                input('Press Enter to Aspirate')
                # Aspirate
                await move_plunger(64, -10).run(can_messenger = messenger)
                await asyncio.sleep(delay)
                input('Press Enter to Dispense!!')
                # Dispense + Blow out
                await asyncio.sleep(delay)
                await move_plunger(71, 10).run(can_messenger = messenger)
                input("Press Enter to Home")

        elif args.test =='drop_tip':
            await home_z.run(can_messenger = messenger)
            await asyncio.sleep(3)
            await home_pipette.run(can_messenger = messenger)
            await asyncio.sleep(delay)
            await home_jaw.run(can_messenger=messenger)
            await asyncio.sleep(delay)
            await drop_tips.run(can_messenger=messenger)
            await asyncio.sleep(delay)
            await home_jaw.run(can_messenger=messenger)
            await asyncio.sleep(delay)
        else:
            raise('Test Not Recognized!!!! Check the Spelling')
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
        "--z_run_current",
        type=float,
        help="Active current of the plunger",
        default=1.5,
    )
    parser.add_argument(
        "--z_hold_current",
        type=float,
        help="Dwell current of the plunger",
        default=0.8,
    )
    parser.add_argument(
        "--plunger_run_current",
        type=float,
        help="Active current of the plunger",
        default=1.5,
    )
    parser.add_argument(
        "--plunger_hold_current",
        type=float,
        help="Active current of the plunger",
        default=1.5,
    )
    parser.add_argument(
        "--speed",
        type=float,
        help="The speed with which to move the plunger",
        default=10.0,
    )
    parser.add_argument(
        "--cycles",
        type=float,
        help="The speed with which to move the plunger",
        default=20,
    )
    parser.add_argument(
        "--distance",
        type=float,
        help="The distance in mm to move the plunger",
        default=30,
    )
    parser.add_argument(
        "--test",
        type=str,
        help="partial_pick_up, full_pick_up, aspirate/dispense",
        default="partial_pick_up",
    )
    args = parser.parse_args()

    asyncio.run(run(args))


if __name__ == "__main__":
    main()
