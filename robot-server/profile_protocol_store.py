import cProfile
import asyncio
import json
import logging
from pathlib import Path

# import yappi

from opentrons.protocol_reader import ProtocolReader
from robot_server.persistence.database import create_sql_engine
from robot_server.protocols.protocol_store import ProtocolStore
from robot_server.service import initialize_logging


# Change me if running locally on dev server!
PERSISTENCE_DIRECTORY = Path("/Users/maxpm/Downloads/cytotronics_opentrons_robot_server")
_DATABASE_PATH = PERSISTENCE_DIRECTORY / "robot_server.db"
_PROTOCOLS_DIRECTORY = PERSISTENCE_DIRECTORY / "protocols"

RESULTS_FILE = "profile_results.prof"

_log = logging.getLogger(__name__)


def main():
    logging.basicConfig(level="INFO")

    sql_engine = create_sql_engine(_DATABASE_PATH)
    protocol_reader = ProtocolReader()

    async def rehydrate():
        await ProtocolStore.rehydrate(
            sql_engine=sql_engine,
            protocols_directory=_PROTOCOLS_DIRECTORY,
            protocol_reader=protocol_reader,
        )

    def parse_as_basic_json():
        all_json_files = PERSISTENCE_DIRECTORY.glob("**/*.json")
        for json_file in all_json_files:
            _log.info(f"Parsing {json_file}")
            json.loads(json_file.read_text(encoding="utf-8"))["metadata"]

    cProfile.runctx("asyncio.run(rehydrate())", globals(), locals())
    # cProfile.runctx("parse_as_basic_json()", globals(), locals(), filename=RESULTS_FILE)

    # yappi.set_clock_type("WALL")
    # with yappi.run():
    #     asyncio.run(rehydrate())
    # yappi.get_func_stats().save(RESULTS_FILE, type="pstat")


main()
