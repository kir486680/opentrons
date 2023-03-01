"""Pipette motions."""
from dataclasses import dataclass
from typing import Optional, Callable

from opentrons.protocol_api import InstrumentContext
from opentrons.protocol_api.labware import Well
from opentrons.protocols.api_support.types import APIVersion

from hardware_testing.gravimetric.liquid.height import LiquidTracker
from hardware_testing.gravimetric.liquid.liquid_class import (
    LiquidClassSettings,
    LIQUID_CLASS_DEFAULT,
)
from hardware_testing.gravimetric.helpers import get_pipette_unique_name


LABWARE_BOTTOM_CLEARANCE = 1.5  # FIXME: not sure who should own this

API_VERSION_WITH_PREP_ASPIRATE_FIX = APIVersion(2, 13)


@dataclass
class LiquidSurfaceHeights:
    """Liquid Surface Heights."""

    above: float
    below: float


@dataclass
class PipettingHeights:
    """Pipetting heights."""

    start: LiquidSurfaceHeights
    end: LiquidSurfaceHeights


def _create_pipetting_heights(
    start_mm: float, end_mm: float, liquid_class: LiquidClassSettings
) -> PipettingHeights:
    # Calculates the:
    #     1) current liquid-height of the well
    #     2) the resulting liquid-height of the well, after a specified volume is
    #        aspirated/dispensed
    #
    # Then, use these 2 liquid-heights (start & end heights) to return four Locations:
    #     1) Above the starting liquid height
    #     2) Submerged in the starting liquid height
    #     3) Above the ending liquid height
    #     4) Submerged in the ending liquid height
    assert liquid_class.retract.distance
    assert liquid_class.submerge.distance
    return PipettingHeights(
        start=LiquidSurfaceHeights(
            above=max(
                start_mm + liquid_class.retract.distance, LABWARE_BOTTOM_CLEARANCE
            ),
            below=max(
                start_mm - liquid_class.submerge.distance, LABWARE_BOTTOM_CLEARANCE
            ),
        ),
        end=LiquidSurfaceHeights(
            above=max(end_mm + liquid_class.retract.distance, LABWARE_BOTTOM_CLEARANCE),
            below=max(
                end_mm - liquid_class.submerge.distance, LABWARE_BOTTOM_CLEARANCE
            ),
        ),
    )


@dataclass
class LiquidSettingsRunnerConfig:
    """Pipetting Liquid Settings Config."""

    pipette: InstrumentContext
    well: Well
    heights: PipettingHeights
    settings: LiquidClassSettings
    aspirate: Optional[float]
    dispense: Optional[float]


class LiquidSettingsRunner:
    """Pipetting Liquid Settings."""

    def __init__(
        self,
        delay_method: Callable,
        cfg: LiquidSettingsRunnerConfig,
    ) -> None:
        """Pipetting Liquid Settings."""
        assert (
            cfg.aspirate is not None or cfg.dispense is not None
        ), "must either aspirate or dispense"
        assert (
            cfg.aspirate is None or cfg.dispense is None
        ), "cannot both aspirate and dispense"
        self._delay_method = delay_method
        self._cfg = cfg

    def run(
        self,
        on_pre_submerge: Optional[Callable[[LiquidSettingsRunnerConfig], None]] = None,
        on_post_emerge: Optional[Callable[[LiquidSettingsRunnerConfig], None]] = None,
    ) -> None:
        """Run."""
        self._run_approach()
        if callable(on_pre_submerge):
            on_pre_submerge(self._cfg)
        self._run_gather_air_gaps()
        self._run_submerge()
        if self._cfg.aspirate:
            self._run_aspirate()
        else:
            self._run_dispense()
        self._run_delay()
        self._run_retract()
        self._run_blow_out()
        self._run_finish()
        if callable(on_post_emerge):
            on_post_emerge(self._cfg)

    def _run_approach(self) -> None:
        self._cfg.pipette.move_to(self._cfg.well.top())

    def _run_gather_air_gaps(self) -> None:
        if self._cfg.aspirate and self._cfg.settings.wet_air_gap.volume:
            self._cfg.pipette.aspirate(self._cfg.settings.wet_air_gap.volume)

    def _run_submerge(self) -> None:
        # Note: in case (start.above < end.below)
        start_above = max(self._cfg.heights.start.above, self._cfg.heights.end.below)
        self._cfg.pipette.move_to(
            self._cfg.well.bottom(start_above), force_direct=False
        )
        submerged_loc = self._cfg.well.bottom(self._cfg.heights.end.below)
        self._cfg.pipette.move_to(
            submerged_loc, force_direct=True, speed=self._cfg.settings.submerge.speed
        )

    def _run_aspirate(self) -> None:
        if self._cfg.aspirate:
            self._cfg.pipette.aspirate(self._cfg.aspirate)

    def _run_dispense(self) -> None:
        if self._cfg.dispense:
            self._cfg.pipette.dispense(self._cfg.dispense)
            if self._cfg.pipette.current_volume > 0:
                # temporarily change the dispense speed
                _disp_flow_rate = float(self._cfg.pipette.flow_rate.dispense)
                self._cfg.pipette.flow_rate.dispense = (
                    self._cfg.settings.wet_air_gap.flow_rate
                )
                self._cfg.pipette.dispense()
                # go back to previous speed
                self._cfg.pipette.flow_rate.dispense = _disp_flow_rate

    def _run_delay(self) -> None:
        if self._cfg.aspirate and self._cfg.settings.aspirate.delay:
            delay_time = self._cfg.settings.aspirate.delay
        elif self._cfg.dispense and self._cfg.settings.dispense.delay:
            delay_time = self._cfg.settings.dispense.delay
        else:
            delay_time = 0
        self._delay_method(delay_time)

    def _run_retract(self) -> None:
        self._cfg.pipette.move_to(
            self._cfg.well.bottom(self._cfg.heights.end.above),
            force_direct=True,
            speed=self._cfg.settings.retract.speed,
        )

    def _run_blow_out(self) -> None:
        if self._cfg.dispense and self._cfg.settings.blow_out.volume:
            self._cfg.pipette.blow_out()  # nothing to loose

    def _run_finish(self) -> None:
        self._cfg.pipette.move_to(self._cfg.well.top(), force_direct=True)


class PipetteLiquidClass:
    """Pipette Liquid Class."""

    def __init__(
        self,
        pipette: InstrumentContext,
        test_name: str,
        run_id: str,
        start_time: float,
        delay_method: Callable
    ) -> None:
        """Pipette Liquid Class."""
        self._pipette = pipette
        self._delay_method = delay_method
        self._test_name = test_name
        self._run_id = run_id
        self._start_time = start_time

        self._liq_cls: LiquidClassSettings = LIQUID_CLASS_DEFAULT

        self._on_pre_aspirate: Optional[Callable] = None
        self._on_post_aspirate: Optional[Callable] = None
        self._on_pre_dispense: Optional[Callable] = None
        self._on_post_dispense: Optional[Callable] = None
        self._file_name: Optional[str] = None

    @property
    def unique_name(self) -> str:
        """Unique name."""
        return get_pipette_unique_name(self.pipette)

    @property
    def tag(self) -> str:
        """Tag."""
        return f"{self.__class__.__name__}-{self.unique_name}"

    @property
    def pipette(self) -> InstrumentContext:
        """Pipette."""
        return self._pipette

    def record_timestamp_disable(self) -> None:
        """Disable recording timestamps."""
        self._file_name = None

    def set_liquid_class(self, settings: LiquidClassSettings) -> None:
        """Set Liquid Class."""
        self._liq_cls = settings
        self._apply_pipette_speeds()

    def _apply_pipette_speeds(self) -> None:
        assert self._liq_cls.traverse.speed
        self._pipette.default_speed = self._liq_cls.traverse.speed
        self._pipette.flow_rate.aspirate = self._liq_cls.aspirate.flow_rate
        self._pipette.flow_rate.dispense = self._liq_cls.dispense.flow_rate
        self._pipette.flow_rate.blow_out = self._liq_cls.blow_out.flow_rate

    def assign_callbacks(
        self,
        on_pre_aspirate: Optional[Callable] = None,
        on_post_aspirate: Optional[Callable] = None,
        on_pre_dispense: Optional[Callable] = None,
        on_post_dispense: Optional[Callable] = None,
    ) -> None:
        """Assign callbacks."""
        self._on_pre_aspirate = on_pre_aspirate
        self._on_post_aspirate = on_post_aspirate
        self._on_pre_dispense = on_pre_dispense
        self._on_post_dispense = on_post_dispense

    def aspirate(self, volume: float, well: Well, liquid_level: LiquidTracker) -> None:
        """Aspirate."""
        self._pipette_liquid_settings(well, liquid_level, aspirate=volume)

    def dispense(self, volume: float, well: Well, liquid_level: LiquidTracker) -> None:
        """Dispense."""
        self._pipette_liquid_settings(well, liquid_level, dispense=volume)

    def _pipette_liquid_settings(
        self,
        well: Well,
        liquid_tracker: LiquidTracker,
        aspirate: Optional[float] = None,
        dispense: Optional[float] = None,
    ) -> None:
        """Run a pipette given some Pipetting Liquid Settings."""
        height_before, height_after = liquid_tracker.get_before_and_after_heights(
            self._pipette, well, aspirate=aspirate, dispense=dispense
        )
        pipetting_heights = _create_pipetting_heights(
            start_mm=height_before, end_mm=height_after, liquid_class=self._liq_cls
        )
        pipetting_cfg = LiquidSettingsRunnerConfig(
            pipette=self._pipette,
            well=well,
            heights=pipetting_heights,
            settings=self._liq_cls,
            aspirate=aspirate,
            dispense=dispense,
        )
        pip_runner = LiquidSettingsRunner(
            delay_method=self._delay_method, cfg=pipetting_cfg
        )
        if aspirate:
            pip_runner.run(
                on_pre_submerge=self._on_pre_aspirate,
                on_post_emerge=self._on_post_aspirate,
            )
        else:
            pip_runner.run(
                on_pre_submerge=self._on_pre_dispense,
                on_post_emerge=self._on_post_dispense,
            )
        liquid_tracker.update_affected_wells(
            self._pipette, well, aspirate=aspirate, dispense=dispense
        )