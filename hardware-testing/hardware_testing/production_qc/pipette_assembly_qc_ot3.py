"""Pipette Assembly QC Test."""
import argparse
import asyncio
from dataclasses import dataclass
from time import time
from typing import Optional, Callable, List, Any
from typing_extensions import Final

from opentrons.hardware_control.ot3api import OT3API

from hardware_testing import data
from hardware_testing.drivers import list_ports_and_select
from hardware_testing.drivers.pressure_fixture import (
    PressureFixture,
    SimPressureFixture,
)
from hardware_testing.opentrons_api import helpers_ot3
from hardware_testing.opentrons_api.types import OT3Mount, Point

TEST_NAME: Final = "pipette-assembly-qc"
CSV_HEADER_FIXTURE: Final = "time,p1,p2,p3,p4,p5,p6,p7,p8,phase"

TRASH_SLOT: Final = 12
TRASH_HEIGHT_MM: Final = 45
LEAK_HOVER_ABOVE_LIQUID_MM: Final = 50

FIXTURE_LOCATION_A1_LEFT = Point(x=14.4, y=74.5, z=71.2)
FIXTURE_LOCATION_A1_RIGHT = FIXTURE_LOCATION_A1_LEFT._replace(x=128 - 14.4)


@dataclass
class TestConfig:
    """Test Configurations."""

    operator_name: str
    fixture_port: str
    fixture_depth: int
    fixture_side: str
    tip_volume: int
    aspirate_volume: float
    slot_tip_rack: int
    slot_reservoir: int
    slot_fixture: int
    num_trials: int
    wait_seconds: int


@dataclass
class LabwareLocations:
    """Test Labware Locations."""

    trash: Optional[Point]
    tip_rack: Optional[Point]
    reservoir: Optional[Point]
    fixture: Optional[Point]


# start with dummy values, these will be immediately overwritten
# we start with actual values here to pass linting
IDEAL_LABWARE_LOCATIONS: LabwareLocations = LabwareLocations(
    trash=None, tip_rack=None, reservoir=None, fixture=None
)
CALIBRATED_LABWARE_LOCATIONS: LabwareLocations = LabwareLocations(
    trash=None, tip_rack=None, reservoir=None, fixture=None
)


def _get_ideal_labware_locations(test_config: TestConfig) -> LabwareLocations:
    tip_rack_loc_ideal = helpers_ot3.get_theoretical_a1_position(
        test_config.slot_tip_rack, f"opentrons_96_tiprack_{test_config.tip_volume}ul"
    )
    reservoir_loc_ideal = helpers_ot3.get_theoretical_a1_position(
        test_config.slot_reservoir, "nest_1_reservoir_195ml"
    )
    trash_loc_ideal = helpers_ot3.get_slot_calibration_square_position_ot3(TRASH_SLOT)
    trash_loc_ideal += Point(z=TRASH_HEIGHT_MM)
    return LabwareLocations(
        tip_rack=tip_rack_loc_ideal,
        reservoir=reservoir_loc_ideal,
        trash=trash_loc_ideal,
        fixture=Point(x=14.4, y=74.5, z=71.2),  # TODO: get actual location of fixture
    )


def _tip_name_to_xy_offset(tip: str) -> Point:
    tip_rack_rows = ["A", "B", "C", "D", "E", "F", "G", "H"]
    tip_row = tip_rack_rows.index(tip[0])
    tip_column = int(tip[1]) - 1
    return Point(x=tip_column * 9, y=tip_row * 9)


async def _calibrate_and_pick_up_tip(
    api: OT3API, mount: OT3Mount, test_config: TestConfig, tip: str
) -> None:
    if not CALIBRATED_LABWARE_LOCATIONS.tip_rack:
        print("calibrate the tip-rack location")
        assert IDEAL_LABWARE_LOCATIONS.tip_rack
        await helpers_ot3.move_to_arched_ot3(
            api, mount, IDEAL_LABWARE_LOCATIONS.tip_rack + Point(z=10)
        )
        print("jog to the tip-rack")
        await helpers_ot3.jog_mount_ot3(api, mount)
        CALIBRATED_LABWARE_LOCATIONS.tip_rack = await api.gantry_position(mount)
        await api.move_rel(mount, Point(z=5))
    tip_offset = _tip_name_to_xy_offset(tip)
    tip_pos = CALIBRATED_LABWARE_LOCATIONS.tip_rack + tip_offset
    await helpers_ot3.move_to_arched_ot3(api, mount, tip_pos, safe_height=tip_pos.z + 5)
    tip_length = helpers_ot3.get_default_tip_length(test_config.tip_volume)
    await api.pick_up_tip(mount, tip_length=tip_length)


async def _calibrate_and_move_to_liquid(api: OT3API, mount: OT3Mount) -> None:
    if not CALIBRATED_LABWARE_LOCATIONS.reservoir:
        assert IDEAL_LABWARE_LOCATIONS.reservoir
        await helpers_ot3.move_to_arched_ot3(
            api, mount, IDEAL_LABWARE_LOCATIONS.reservoir + Point(z=10)
        )
        print("jog to reservoir, 2mm below the liquid")
        await helpers_ot3.jog_mount_ot3(api, mount)
        CALIBRATED_LABWARE_LOCATIONS.reservoir = await api.gantry_position(mount)
    else:
        print("moving to the reservoir")
        await helpers_ot3.move_to_arched_ot3(
            api, mount, CALIBRATED_LABWARE_LOCATIONS.reservoir
        )


async def _calibrate_and_move_to_fixture(api: OT3API, mount: OT3Mount) -> None:
    if not CALIBRATED_LABWARE_LOCATIONS.fixture:
        assert IDEAL_LABWARE_LOCATIONS.fixture
        await helpers_ot3.move_to_arched_ot3(
            api, mount, IDEAL_LABWARE_LOCATIONS.fixture + Point(z=10)
        )
        print("jog to fixture")
        await helpers_ot3.jog_mount_ot3(api, mount)
        CALIBRATED_LABWARE_LOCATIONS.fixture = await api.gantry_position(mount)
    else:
        print("moving to the fixture")
        await helpers_ot3.move_to_arched_ot3(
            api, mount, CALIBRATED_LABWARE_LOCATIONS.fixture
        )


async def _aspirate_and_look_for_droplets(
    api: OT3API, mount: OT3Mount, test_config: TestConfig
) -> bool:
    print(f"aspirating {test_config.aspirate_volume} microliters")
    await api.aspirate(mount, volume=test_config.aspirate_volume)
    await api.move_rel(mount, Point(z=LEAK_HOVER_ABOVE_LIQUID_MM))
    for t in range(test_config.wait_seconds):
        print(f"waiting for leaking tips ({t + 1}/{test_config.wait_seconds})")
        if not api.is_simulator:
            await asyncio.sleep(1)
    if api.is_simulator:
        leak_test_passed = True
    else:
        user_inp = ""
        while not user_inp or user_inp not in ["y", "n"]:
            user_inp = input("did it pass? no leaking? (y/n)").strip()
        leak_test_passed = "y" in user_inp
    # TODO: save pass/fail to CSV
    print("dispensing back into reservoir")
    await api.move_rel(mount, Point(z=-LEAK_HOVER_ABOVE_LIQUID_MM))
    await api.dispense(mount, test_config.aspirate_volume)
    await api.blow_out(mount)
    return leak_test_passed


async def _fixture_check_for_leak(
    api: OT3API,
    mount: OT3Mount,
    test_config: TestConfig,
    write_cb: Callable,
    fixture: PressureFixture,
) -> bool:
    pre_pressure = fixture.read_all_pressure_channel()
    await api.move_rel(mount, Point(z=-test_config.fixture_depth))
    insert_pressure = fixture.read_all_pressure_channel()
    await api.aspirate(mount, test_config.aspirate_volume)
    aspirate_pressure = fixture.read_all_pressure_channel()
    await api.dispense(mount, test_config.aspirate_volume)
    dispense_pressure = fixture.read_all_pressure_channel()
    await api.move_rel(mount, Point(z=test_config.fixture_depth))
    post_pressure = fixture.read_all_pressure_channel()
    if api.is_simulator:
        return True
    # TODO: figure out how to analyze this data, detect pass/fail
    # TODO: save pass/fail to CSV
    return True


async def _drop_tip_in_trash(api: OT3API, mount: OT3Mount) -> None:
    assert IDEAL_LABWARE_LOCATIONS.trash
    await helpers_ot3.move_to_arched_ot3(
        api,
        mount,
        IDEAL_LABWARE_LOCATIONS.trash,
        safe_height=IDEAL_LABWARE_LOCATIONS.trash.z + 20,
    )
    await api.drop_tip(mount, home_after=False)


async def _test_for_leak(
    api: OT3API,
    mount: OT3Mount,
    test_config: TestConfig,
    tip: str,
    fixture: Optional[PressureFixture],
    write_cb: Optional[Callable],
) -> bool:
    await _calibrate_and_pick_up_tip(api, mount, test_config, tip=tip)
    if fixture:
        assert write_cb, "pressure fixture requires recording data to disk"
        await _calibrate_and_move_to_fixture(api, mount)
        test_passed = await _fixture_check_for_leak(
            api, mount, test_config, write_cb=write_cb, fixture=fixture
        )
    else:
        await _calibrate_and_move_to_liquid(api, mount)
        test_passed = await _aspirate_and_look_for_droplets(api, mount, test_config)
    await _drop_tip_in_trash(api, mount)
    pass_msg = "PASS" if test_passed else "FAIL"
    print(f"tip {tip}: {pass_msg}")
    return test_passed


async def _test_for_leak_by_eye(
    api: OT3API, mount: OT3Mount, test_config: TestConfig, tip: str
) -> bool:
    return await _test_for_leak(api, mount, test_config, tip, None, None)


async def _main(simulate: bool, mount: OT3Mount, test_config: TestConfig) -> None:
    global IDEAL_LABWARE_LOCATIONS
    global CALIBRATED_LABWARE_LOCATIONS

    # create API instance, and get Pipette serial number
    api = await helpers_ot3.build_async_ot3_hardware_api(
        is_simulating=simulate,
        pipette_left="p1000_single_v3.3",
        pipette_right="p1000_single_v3.3",
    )
    pipette = api.hardware_pipettes[mount.to_mount()]
    assert pipette, f"No pipette found on mount: {mount}"
    pipette_sn = helpers_ot3.get_pipette_serial_ot3(pipette)
    print(f"Pipette: {pipette_sn}")

    # setup our labware locations
    IDEAL_LABWARE_LOCATIONS = _get_ideal_labware_locations(test_config)
    CALIBRATED_LABWARE_LOCATIONS = LabwareLocations(
        trash=None, tip_rack=None, reservoir=None, fixture=None
    )

    # connect to the pressure fixture
    if not simulate:
        fixture = PressureFixture.create(
            port=test_config.fixture_port, slot_side=test_config.fixture_side
        )
    else:
        fixture = SimPressureFixture()  # type: ignore[assignment]
    fixture.connect()

    # create the CSV file, using the Pipette serial number as the tag
    run_id = data.create_run_id()
    data.create_folder_for_test_data(TEST_NAME)
    file_name = data.create_file_name(TEST_NAME, run_id, pipette_sn)
    start_time = time()

    # callback function for writing new data to CSV file
    def _append_csv_data(data_list: List[Any]) -> None:
        # prepend the elapsed seconds, so the time is always in the first column
        elapsed_seconds = round(time() - start_time, 2)
        data_list_with_time = [elapsed_seconds] + data_list
        data_str = ",".join([str(d) for d in data_list_with_time])
        data.append_data_to_file(TEST_NAME, file_name, data_str + "\n")

    # add metadata and header to CSV
    if simulate:
        _append_csv_data(["simulating"])
    _append_csv_data(["operator-name", test_config.operator_name])
    _append_csv_data(["run-id", run_id])  # includes a date/time string
    _append_csv_data(["pipette", pipette_sn])

    # prepare to run the test
    tips_liquid = [f"A{i + 1}" for i in range(test_config.num_trials)]
    tips_fixture = [f"B{i + 1}" for i in range(test_config.num_trials)]

    # run the test
    await api.home()
    for tip in tips_liquid:
        test_passed = await _test_for_leak_by_eye(api, mount, test_config, tip)
        _append_csv_data(["droplet-test", tip, "PASS" if test_passed else "FAIL"])
    for tip in tips_fixture:
        test_passed = await _test_for_leak(
            api, mount, test_config, tip, fixture=fixture, write_cb=_append_csv_data
        )
        _append_csv_data(["pressure-test", tip, "PASS" if test_passed else "FAIL"])
    await api.home()


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description="OT-3 Pipette Assembly QC Test")
    arg_parser.add_argument("--operator", type=str, required=True)
    arg_parser.add_argument(
        "--mount", choices=["left", "right", "gripper"], required=True
    )
    arg_parser.add_argument("--tip", type=int, required=True)
    arg_parser.add_argument("--volume", type=float, required=True)
    arg_parser.add_argument("--fixture-side", choices=["left", "right"], default="left")
    arg_parser.add_argument("--port", type=str, default="")
    arg_parser.add_argument("--num-trials", type=int, default=2)
    arg_parser.add_argument("--wait", type=int, default=30)
    arg_parser.add_argument("--slot-tip-rack", type=int, default=1)
    arg_parser.add_argument("--slot-reservoir", type=int, default=5)
    arg_parser.add_argument("--slot-fixture", type=int, default=2)
    arg_parser.add_argument("--insert-depth", type=int, default=14)
    arg_parser.add_argument("--simulate", action="store_true")
    args = arg_parser.parse_args()
    ot3_mounts = {
        "left": OT3Mount.LEFT,
        "right": OT3Mount.RIGHT,
        "gripper": OT3Mount.GRIPPER,
    }
    _mount = ot3_mounts[args.mount]
    if not args.simulate and not args.port:
        _port = list_ports_and_select("pressure-fixture")
    else:
        _port = ""
    assert (
        args.volume <= args.tip
    ), f"aspirate volume {args.volume} must be <= than tip {args.tip}"
    _cfg = TestConfig(
        operator_name=args.operator,
        fixture_port=_port,
        fixture_depth=args.insert_depth,
        fixture_side=args.fixture_side,
        aspirate_volume=args.volume,
        tip_volume=args.tip,
        slot_tip_rack=args.slot_tip_rack,
        slot_reservoir=args.slot_reservoir,
        slot_fixture=args.slot_fixture,
        num_trials=args.num_trials,
        wait_seconds=args.wait,
    )
    asyncio.run(_main(args.simulate, _mount, _cfg))
