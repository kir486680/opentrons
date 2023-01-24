"""A script for sending and receiving data from sensors on the OT3."""
import logging
import asyncio
import argparse
from dataclasses import dataclass
from numpy import float64
from typing import List, Tuple
import termios
import sys, tty, os, time

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

AXIS_SPEED_X = 30
AXIS_SPEED_Y = 60
AXIS_SPEED_Z = 10.5

AccuracyAdjustTable = List[Tuple[float, float, float]]
# copy the lookup table from EVT single-channel pipettes
PIPETTE_SINGLE_EVT_VERSION = PipetteVersionType(major=3, minor=3)
PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM = 15.904

PLUNGER_BOTTOM = 66
PLUNGER_BLOW_OUT = 71

DEFAULT_BLOWOUT_SPEED_UL = 80
DEFAULT_TRAILING_SPEED_MM = 1  # mm/sec
DEFAULT_PRE_BLOWOUT_DELAY_SECONDS = 5  # wait for droplets to accumulate

GRAB_SPEED = 5.5
GRAB_DISTANCE = 19
DROP_SPEED = 5.5
DROP_DISTANCE = 29

TIP_OVERLAP = 10  # seems to be about 0.5mm above ideal spot
TIP_LENGTH = {
    50: 57.9 - TIP_OVERLAP,
    200: 58.35 - TIP_OVERLAP,
    1000: 95.6 - TIP_OVERLAP
}

# NOTE: home position is 0, moving down is position (+)
CAL_POS_TIP_RACK_FEATURES = 125
CAL_POS_RESERVOIR_TOP = 218.6  # w/ added tip-length (was 133 using t1000)
CAL_POS_RESERVOIR_DEAD = CAL_POS_RESERVOIR_TOP + 26  # maybe 0.5mm submerged into inset
CAL_POS_MVS_TOP_TO_PLATE_TOP = 235.85
mvs_well_depth = 10.8
mvs_200_ul_height = 6.0
CAL_POS_MVS_TOP_TO_PLATE_200_UL = CAL_POS_MVS_TOP_TO_PLATE_TOP + (mvs_well_depth - mvs_200_ul_height)

CURRENT_Z_POS = 0
CURRENT_HAS_TIP = False
CURRENT_TIP_LENGTH = 0

ASPIRATE_DELAY_SECONDS = 0.6


@dataclass
class TestParams:
    tip_volume: int
    aspirate_volume_ul: float
    aspirate_speed_ul_per_sec: float
    aspirate_delay: float
    dispense_speed_ul_per_sec: float
    leading_air_gap_ul: float
    trailing_air_gap_ul: float


TESTS = {
    "t50-1ul": TestParams(tip_volume=50,
                          aspirate_volume_ul=1.0,
                          aspirate_speed_ul_per_sec=20.0,
                          aspirate_delay=0.2,
                          leading_air_gap_ul=15.0,
                          trailing_air_gap_ul=2.0,
                          dispense_speed_ul_per_sec=6.5),
    "t50-10ul": TestParams(tip_volume=50,
                           aspirate_volume_ul=10.0,
                           aspirate_speed_ul_per_sec=5.7,
                           aspirate_delay=0.2,
                           leading_air_gap_ul=15.0,
                           trailing_air_gap_ul=0.1,
                           dispense_speed_ul_per_sec=6.5),
    "t50-50ul": TestParams(tip_volume=50,
                           aspirate_volume_ul=50.0,
                           aspirate_speed_ul_per_sec=44.2,
                           aspirate_delay=0.2,
                           leading_air_gap_ul=15.0,
                           trailing_air_gap_ul=0.1,
                           dispense_speed_ul_per_sec=6.5),
    "t200-10ul": TestParams(tip_volume=200,
                            aspirate_volume_ul=10,
                            aspirate_speed_ul_per_sec=12.5,
                            aspirate_delay=0.5,
                            leading_air_gap_ul=10,
                            trailing_air_gap_ul=5,
                            dispense_speed_ul_per_sec=80),
    "t200-50ul": TestParams(tip_volume=200,
                            aspirate_volume_ul=50,
                            aspirate_speed_ul_per_sec=37.5,
                            aspirate_delay=0.5,
                            leading_air_gap_ul=10,
                            trailing_air_gap_ul=3.5,
                            dispense_speed_ul_per_sec=80),
    "t200-200ul": TestParams(tip_volume=200,
                             aspirate_volume_ul=200,
                             aspirate_speed_ul_per_sec=150,
                             aspirate_delay=0.5,
                             leading_air_gap_ul=7.7,
                             trailing_air_gap_ul=2,
                             dispense_speed_ul_per_sec=80),
    "t1000-10ul": TestParams(tip_volume=1000,
                             aspirate_volume_ul=10,
                             aspirate_speed_ul_per_sec=22,
                             aspirate_delay=0.2,
                             leading_air_gap_ul=10,
                             trailing_air_gap_ul=10,
                             dispense_speed_ul_per_sec=160),
    "t1000-50ul": TestParams(tip_volume=1000,
                             aspirate_volume_ul=50,
                             aspirate_speed_ul_per_sec=37,
                             aspirate_delay=0.2,
                             leading_air_gap_ul=10,
                             trailing_air_gap_ul=10,
                             dispense_speed_ul_per_sec=160),
    "t1000-200ul": TestParams(tip_volume=1000,
                              aspirate_volume_ul=200,
                              aspirate_speed_ul_per_sec=150,
                              aspirate_delay=0.2,
                              leading_air_gap_ul=7,
                              trailing_air_gap_ul=10,
                              dispense_speed_ul_per_sec=160),
    "t1000-500ul": TestParams(tip_volume=1000,
                              aspirate_volume_ul=500,
                              aspirate_speed_ul_per_sec=150,
                              aspirate_delay=0.2,
                              leading_air_gap_ul=2,
                              trailing_air_gap_ul=10,
                              dispense_speed_ul_per_sec=160),
    "t1000-1000ul": TestParams(tip_volume=1000,
                               aspirate_volume_ul=1000,
                               aspirate_speed_ul_per_sec=150,
                               aspirate_delay=0.2,
                               leading_air_gap_ul=2,
                               trailing_air_gap_ul=10,
                               dispense_speed_ul_per_sec=160),
}


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
            print(f"Lookup Table Values: v={tv}, s={ts}, i={ti}")
            ul_per_mm = (volume * ts) + ti
            mm_travel = volume / ul_per_mm
            return mm_travel
    raise ValueError(f"unable to find volume {volume} in table")


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
                        duration_sec=float64(abs(distance / velocity)),
                        stop_condition=MoveStopCondition.none,
                        action=PipetteTipActionType.pick_up,
                    )
                }
            ]
        ]
    )
    return move

def move_y_axis(distance, velocity):
    pipette_node = NodeId.gantry_y
    move_plunger_runner = MoveGroupRunner(
        # Group 0
        move_groups=[
            [
                {
                    pipette_node: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(abs(distance / velocity)),
                    )
                }
            ]
        ],
    )
    return move_plunger_runner

def move_x_axis(distance, velocity):
    pipette_node = NodeId.gantry_x
    move_plunger_runner = MoveGroupRunner(
        # Group 0
        move_groups=[
            [
                {
                    pipette_node: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(abs(distance / velocity)),
                    )
                }
            ]
        ],
    )
    return move_plunger_runner

def move_z_axis(distance, velocity):
    pipette_node = NodeId.head_l
    move_plunger_runner = MoveGroupRunner(
        # Group 0
        move_groups=[
            [
                {
                    pipette_node: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(abs(distance / velocity)),
                    )
                }
            ]
        ],
    )
    return move_plunger_runner

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
                        duration_sec=float64(abs(distance / velocity)),
                        stop_condition=MoveStopCondition.limit_switch,
                        action=PipetteTipActionType.pick_up,
                    )
                }
            ]
        ]
    )
    return move


def move_z_axis_to(position, velocity):
    global CURRENT_Z_POS
    position -= CURRENT_TIP_LENGTH
    if position > CURRENT_Z_POS:
        velocity = abs(velocity)
    else:
        velocity = abs(velocity) * -1
    distance = abs(CURRENT_Z_POS - position)
    CURRENT_Z_POS = position
    move_z = MoveGroupRunner(
        move_groups=[
            # Group 1
            [
                {
                    NodeId.head_l: MoveGroupSingleAxisStep(
                        distance_mm=float64(0),
                        velocity_mm_sec=float64(velocity),
                        duration_sec=float64(abs(distance / velocity)),
                    )
                }
            ],
        ]
    )
    return move_z


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
                        duration_sec=float64(abs(distance / velocity)),
                    )
                }
            ]
        ],
    )
    return move_plunger_runner


def home_z_axis():
    global CURRENT_Z_POS
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
    CURRENT_Z_POS = 0
    return home_z

def home_xy():
    speed = 10.5
    home = MoveGroupRunner(
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
            ]
        ]
    )
    return home


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


async def _run(args: argparse.Namespace) -> None:
    global CURRENT_HAS_TIP
    global CURRENT_TIP_LENGTH
    driver = await build_driver(build_settings(args))
    messenger = CanMessenger(driver=driver)
    messenger.start()

    grab_tips = move_pipette_mechanism(GRAB_DISTANCE, GRAB_SPEED)
    drop_tips = move_pipette_mechanism(DROP_DISTANCE, DROP_SPEED)

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
    step_size = [0.05, 0.1, 0.5, 1, 5, 10, 20, 50, 100]
    step_length_index = 7
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
            if step_length_index >= 8:
                step_length_index = 8
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
    drop_distance = 27
    pick_up_distance = 12
    calibrate = True
    block = True
    press = False
    grab_tips = move_pipette_mechanism(grap_distance, grap_speed)
    drop_tips = move_pipette_mechanism(drop_distance, drop_speed)
    home_jaw = home_pipette_jaw()
    home_pipette = home_plunger()
    home_z = home_z_axis()
    home_gantry = home_xy()
    test = args.test
    # print("----------")
    # print(f"tip-ul: {test.tip_volume}")
    # print(f"aspirate-ul: {test.aspirate_volume_ul}")
    # print(f"aspirate-speed-ul: {test.aspirate_speed_ul_per_sec}")
    # print(f"aspirate-delay-sec: {test.aspirate_delay}")
    # print(f"leading-air-gap-ul: {test.leading_air_gap_ul}")
    # print(f"trailing-air-gap-ul: {test.trailing_air_gap_ul}")
    # print(f"dispense-speed-ul: {test.dispense_speed_ul_per_sec}")
    #
    # table = _get_accuracy_adjust_table(test.tip_volume)
    # aspirate_mm = _get_plunger_distance_for_volume(table, test.aspirate_volume_ul)
    # aspirate_seconds = test.aspirate_volume_ul / test.aspirate_speed_ul_per_sec
    # aspirate_speed_mm = aspirate_mm / aspirate_seconds
    # leading_air_gap_mm = test.leading_air_gap_ul / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    # trailing_air_gap_mm = test.trailing_air_gap_ul / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    # dispense_mm = leading_air_gap_mm + aspirate_mm + trailing_air_gap_mm
    # dispense_speed_mm = test.dispense_speed_ul_per_sec / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    # blowout_speed_mm = DEFAULT_BLOWOUT_SPEED_UL / PIPETTE_SINGLE_EVT_IDEAL_UL_PER_MM
    # blow_out_plunger_distance_mm = PLUNGER_BLOW_OUT - PLUNGER_BOTTOM
    #
    # print("----------")
    # print(f"aspirate-mm: {aspirate_mm}")
    # print(f"aspirate-speed-mm: {aspirate_speed_mm}")
    # print(f"leading-air-gap-mm: {leading_air_gap_mm}")
    # print(f"trailing-air-gap-mm: {trailing_air_gap_mm}")
    # print(f"total travelled: {trailing_air_gap_mm + aspirate_mm + leading_air_gap_mm} / {PLUNGER_BOTTOM}")
    # print(f'dispensing: {dispense_mm} mm at speed {dispense_speed_mm} mm/sec')
    # print(f'blowout: {blow_out_plunger_distance_mm} mm')
    # print("----------")

    try:
        #-------------------partial Pick up-------------------------------------
        if args.test == 'partial_pick_up':
            await home_gantry.run(can_messenger = messenger)
            while True:
                input_current = float(input("Enter Current: "))
                await set_current(messenger, 1.4, NodeId.head_l)
                await home_z.run(can_messenger = messenger)
                await asyncio.sleep(3)
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
                await set_current(messenger, 1.4, NodeId.head_l)
                await home_z.run(can_messenger = messenger)
                await asyncio.sleep(3)
                # # set current for plunger
                # await set_current(messenger, 2.2,NodeId.pipette_left)
                # await home_pipette.run(can_messenger = messenger)
                # await asyncio.sleep(delay)
                # # set current for plunger
                # await set_current(messenger, 1.5,NodeId.pipette_left)
                # prepare for aspirate
                # await move_plunger(63 ,5).run(can_messenger = messenger)
                # await asyncio.sleep(delay)
                current_position['head_l'] = 0
                if block:
                    trough_pos = await _jog_axis(messenger, current_position)
                else:
                    current_position = {'gantry_x': 251.9,
                                    'gantry_y': 329.2,
                                    'head_l': 151}
                # # Aspirate
                # await move_plunger(60, -5).run(can_messenger = messenger)
                # await asyncio.sleep(delay)
                await home_z.run(can_messenger = messenger)
                await asyncio.sleep(3)
                input('Press Enter to continue!!')
                # await move_z_axis(trough_pos['head_l'], z_speed).run(can_messenger = messenger)
                # Dispense + Blow out
                # await asyncio.sleep(delay)
                # await move_plunger(65, 5).run(can_messenger = messenger)
                # await asyncio.sleep(delay)
                # await home_z.run(can_messenger = messenger)
                # await asyncio.sleep(3)
                # await set_current(messenger, 2.2,NodeId.pipette_left)
                # await home_pipette.run(can_messenger = messenger)
                # await asyncio.sleep(delay)
        #-------------------Pick up tip Motors----------------------------------
        elif args.test == 'full_pick_up_tip':
            blow_out_pos = 69
            bottom_pos = 67
            plunger_speed = 3  ## mm/sec
            aspirate_distance_mm = 13.21  # a / ((a * b) + c)  --- using table entry ABOVE desired volume
            mm_per_ul = aspirate_distance_mm / 200.0
            wet_air_gap_mm = 40 * mm_per_ul  # make this ~2% of total volume
            dry_air_gap_mm = 15 * mm_per_ul
            dispense_distance_mm = aspirate_distance_mm + wet_air_gap_mm
            blow_out_plunger_distance_mm = blow_out_pos - bottom_pos
            input('ENTER to home Z')
            await home_z_axis().run(can_messenger = messenger)
            input('ENTER to home Z')
            await home_gantry.run(can_messenger = messenger)
            input("ENTER to home PLUNGER")
            # await home_gantry.run(can_messenger = messenger)
            # await asyncio.sleep(delay)
            await set_current(messenger, 1.5,NodeId.pipette_left)
            await home_pipette.run(can_messenger=messenger)
            input("ENTER to home JAW")
            await home_jaw.run(can_messenger=messenger)
            input("ENTER to jog to TIPRACK")
            position = {'gantry_x': 0,
                        'gantry_y': 0,
                        'head_l': 0}
            tiprack_pos = await _jog_axis(messenger, position, only_z=False)
            input("ENTER to grab tips")
            await grab_tips.run(can_messenger=messenger)
            CURRENT_HAS_TIP = True
            CURRENT_TIP_LENGTH = TIP_LENGTH[test.tip_volume]
            input("ENTER to release tips and HOME jaw")
            await home_jaw.run(can_messenger=messenger)
            input('ENTER to retract Z')
            await move_z_axis_to(CURRENT_TIP_LENGTH + 1, -AXIS_SPEED_Z).run(can_messenger=messenger)
            await home_z_axis().run(can_messenger = messenger)
            block_pos = await _jog_axis(messenger, position, only_z=False)
        # else:
        #     CURRENT_HAS_TIP = True
        #     CURRENT_TIP_LENGTH = TIP_LENGTH[test.tip_volume]
        #
        # # Prepare for aspirate --bottom
        # await set_current(messenger, 1.5, NodeId.pipette_left)
        # print('preparing for aspirate')
        # await move_plunger(PLUNGER_BOTTOM, 10).run(can_messenger=messenger)
        # print('aspirating LEADING-AIR-GAP')
        # await move_plunger(leading_air_gap_mm, -10).run(can_messenger=messenger)
        # print('moving to TOP of RESERVOIR')
        # await move_z_axis_to(CAL_POS_RESERVOIR_TOP, -AXIS_SPEED_Z).run(can_messenger=messenger)

        # # ASPIRATE
        # input('ENTER to move to DEAD-VOL in RESERVOIR')
        # await move_z_axis_to(CAL_POS_RESERVOIR_DEAD, -AXIS_SPEED_Z).run(can_messenger=messenger)
        # if args.pre_wet:
        #     pre_wet_count = 5
        #     for i in range(pre_wet_count):
        #         print(f"Pre-Wet {i + 1}/{pre_wet_count}")
        #         await move_plunger(aspirate_mm, -aspirate_speed_mm).run(can_messenger=messenger)
        #         await move_plunger(aspirate_mm, aspirate_speed_mm).run(can_messenger=messenger)
        # print(f"aspirating: {aspirate_mm} mm")
        # await move_plunger(aspirate_mm, -aspirate_speed_mm).run(can_messenger=messenger)
        # print(f"delaying: {test.aspirate_delay} seconds")
        # await asyncio.sleep(test.aspirate_delay)
        # print('moving to TOP of RESERVOIR')
        # await move_z_axis_to(CAL_POS_RESERVOIR_TOP, AXIS_SPEED_Z).run(can_messenger=messenger)
        # print(f'doing a TRAILING-AIR-GAP: {trailing_air_gap_mm}')
        # await move_plunger(trailing_air_gap_mm, -DEFAULT_TRAILING_SPEED_MM).run(can_messenger=messenger)
        # print('moving out of the way by 100 mm')
        # await move_z_axis_to(CAL_POS_RESERVOIR_TOP - 100, AXIS_SPEED_Z).run(can_messenger=messenger)
        #
        # # DISPENSE
        # input('ENTER to move to ABOVE the plate')
        # await move_z_axis_to(CAL_POS_MVS_TOP_TO_PLATE_TOP - 1, AXIS_SPEED_Z).run(can_messenger=messenger)
        # input('ALIGN the plate, then press ENTER')
        # await move_z_axis_to(CAL_POS_MVS_TOP_TO_PLATE_200_UL + 1.5, AXIS_SPEED_Z).run(can_messenger=messenger)
        # print(f'dispensing: {dispense_mm} mm')
        # await move_plunger(dispense_mm, dispense_speed_mm).run(can_messenger=messenger)
        # print('moving to BLOWOUT height in well')
        # await move_z_axis_to(CAL_POS_MVS_TOP_TO_PLATE_TOP + 1, AXIS_SPEED_Z).run(can_messenger=messenger)
        # print('PAUSE before BLOWOUT')
        # await asyncio.sleep(DEFAULT_PRE_BLOWOUT_DELAY_SECONDS)
        # print('doing a BLOWOUT')
        # await move_plunger(blow_out_plunger_distance_mm, blowout_speed_mm).run(can_messenger=messenger)
        # input('ENTER to retract Z')
        # await move_z_axis_to(CURRENT_TIP_LENGTH + 1, -AXIS_SPEED_Z).run(can_messenger=messenger)

        # DROPTIP
        input('ENTER to DROP-TIPS')
        await drop_tips.run(can_messenger=messenger)
        input('ENTER to home JAW')
        print("retracting plunger")
        await move_plunger(PLUNGER_BLOW_OUT - 1, -10).run(can_messenger=messenger)
        print('homing jaw')
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
    # dictConfig(LOG_CONFIG)

    parser = argparse.ArgumentParser(
        description="96 channel tip handling testing script."
    )
    add_can_args(parser)
    parser.add_argument(
        "--test",
        type=str,
        required=True
    )
    parser.add_argument("--pre-wet", action="store_true")
    args = parser.parse_args()

    asyncio.run(run(args))


if __name__ == "__main__":
    main()
