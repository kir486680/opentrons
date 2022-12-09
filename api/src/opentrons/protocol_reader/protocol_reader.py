"""Read relevant protocol information from a set of files."""
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, NamedTuple
from typing_extensions import Literal
import json

import anyio
import pydantic

from opentrons.protocols.models import LabwareDefinition

from .input_file import AbstractInputFile
from .file_reader_writer import FileReaderWriter, FileReadError
from .role_analyzer import RoleAnalyzer, RoleAnalysisFile, RoleAnalysisError
from .config_analyzer import ConfigAnalyzer, ConfigAnalysis, ConfigAnalysisError
from .protocol_source import (
    JsonProtocolConfig,
    LabwareDefinitionsReference,
    Metadata,
    ProtocolSource,
    ProtocolSourceFile,
    ProtocolFileRole,
)


class ProtocolFilesInvalidError(ValueError):
    """An error raised if the input files cannot be read to a protocol."""


class MainFileInfo(NamedTuple):
    path: Path
    unvalidated_contents: Dict[str, Any]
    schema_version: int
    metadata: Metadata
    robot_type: Literal["OT-2 Standard", "OT-3 Standard"]


class TrivialLabwareDefinitionsReference(LabwareDefinitionsReference):
    def __init__(self, labware_definitions: List[LabwareDefinition]) -> None:
        self._labware_definitions = labware_definitions

    async def extract(self) -> List[LabwareDefinition]:
        return self._labware_definitions


class JSONParsingLabwareDefinitionsReference(LabwareDefinitionsReference):
    def __init__(self, unvalidated_json_protocol: Dict[str, Any]) -> None:
        self._unvalidated_json_protocol = unvalidated_json_protocol

    async def extract(self) -> List[LabwareDefinition]:
        async def validate_definition(unvalidated_definition: Dict[str, Any]) -> LabwareDefinition:
            # TODO: Does this handle aliases properly? Who knows.
            return await anyio.to_thread.run_sync(LabwareDefinition.parse_obj, unvalidated_definition)

        unvalidated_definitions = self._unvalidated_json_protocol["labware_definitions"]
        return [await validate_definition(unvalidated_definition) for unvalidated_definition in unvalidated_definitions]


class ProtocolReader:
    """Collaborator to turn a set of files into a protocol object."""

    def __init__(
        self,
        file_reader_writer: Optional[FileReaderWriter] = None,
        role_analyzer: Optional[RoleAnalyzer] = None,
        config_analyzer: Optional[ConfigAnalyzer] = None,
    ) -> None:
        """Initialize the reader with its dependencies.

        Arguments:
            directory: The directory into which files will be copied.
            file_reader_writer: Input file reader/writer. Default impl. used if None.
            role_analyzer: File role analyzer. Default impl. used if None.
            config_analyzer: Protocol config analyzer. Default impl. used if None.
        """
        self._file_reader_writer = file_reader_writer or FileReaderWriter()
        self._role_analyzer = role_analyzer or RoleAnalyzer()
        self._config_analyzer = config_analyzer or ConfigAnalyzer()

    async def read_and_save(
        self, files: Sequence[AbstractInputFile], directory: Path
    ) -> ProtocolSource:
        """Compute a `ProtocolSource` from file-like objects and save them as files.

        Arguments:
            files: List of files-like objects. Do not attempt to reuse any objects
                objects in this list once they've been passed to the ProtocolReader.
            directory: Name of the directory to create and place files in.

        Returns:
            A validated ProtocolSource.

        Raises:
            ProtocolFilesInvalidError: Input file list given to the reader
                could not be validated as a protocol.
        """
        try:
            buffered_files = await self._file_reader_writer.read(files)
            role_analysis = self._role_analyzer.analyze(buffered_files)
            config_analysis = self._config_analyzer.analyze(role_analysis.main_file)
        except (FileReadError, RoleAnalysisError, ConfigAnalysisError) as e:
            raise ProtocolFilesInvalidError(str(e)) from e

        # TODO(mc, 2021-12-07): add support for other files, like arbitrary data files
        all_files: List[RoleAnalysisFile] = [
            role_analysis.main_file,
            *role_analysis.labware_files,
        ]

        await self._file_reader_writer.write(directory=directory, files=all_files)
        main_file = directory / role_analysis.main_file.name
        output_files = [
            ProtocolSourceFile(path=directory / f.name, role=f.role) for f in all_files
        ]

        return ProtocolSource(
            directory=directory,
            main_file=main_file,
            files=output_files,
            config=config_analysis.config,
            metadata=config_analysis.metadata,
            robot_type=config_analysis.robot_type,
            labware_definitions=TrivialLabwareDefinitionsReference(role_analysis.labware_definitions),
        )

    async def read_saved(
        self,
        files: Sequence[Path],
        directory: Optional[Path],
    ) -> ProtocolSource:
        """Compute a `ProtocolSource` from protocol source files on the filesystem.

        Arguments:
            files: The files comprising the protocol.
            directory: Passed through to `ProtocolSource.directory`. Otherwise unused.

        Returns:
            A validated ProtocolSource.

        Raises:
            ProtocolFilesInvalidError: Input file list given to the reader
                could not be validated as a protocol.
        """
        try:
            buffered_files = await self._file_reader_writer.read(files)
            role_analysis = self._role_analyzer.analyze(buffered_files)
            config_analysis = self._config_analyzer.analyze(role_analysis.main_file)
        except (FileReadError, RoleAnalysisError, ConfigAnalysisError) as e:
            raise ProtocolFilesInvalidError(str(e)) from e

        # TODO(mc, 2021-12-07): add support for other files, like arbitrary data files
        all_files: List[RoleAnalysisFile] = [
            role_analysis.main_file,
            *role_analysis.labware_files,
        ]

        # TODO(mc, 2022-04-01): these asserts are a bit awkward,
        # consider restructuring so they're not needed
        assert isinstance(role_analysis.main_file.path, Path)
        assert all(isinstance(f.path, Path) for f in all_files)

        main_file = role_analysis.main_file.path
        output_files = [
            ProtocolSourceFile(path=f.path, role=f.role)  # type: ignore[arg-type]
            for f in all_files
        ]

        return ProtocolSource(
            directory=directory,
            main_file=main_file,
            files=output_files,
            config=config_analysis.config,
            metadata=config_analysis.metadata,
            robot_type=config_analysis.robot_type,
            labware_definitions=TrivialLabwareDefinitionsReference(role_analysis.labware_definitions),
        )

    async def read_saved_prevalidated(
        self,
        files: Sequence[Path],
        directory: Optional[Path],
    ) -> ProtocolSource:
        """Like `read_saved()`, except faster.

        This assumes that all files are already validated via a prior call to
        `read_and_save()`.
        """
        # TODO: In real code, we'd probably not want this logic to live in this file.

        main_file_info: Optional[MainFileInfo] = None
        protocol_source_files: List[ProtocolSourceFile] = []

        if any(file_path.name.lower().endswith(".py") for file_path in files):
            # Parse Python protocols the old way.
            return await self.read_saved(files=files, directory=directory)

        for file_path in files:
            assert file_path.name.lower().endswith(".json"), f"{file_path} doesn't seem like a JSON file?"

            def sync_read_and_parse_json() -> Dict[str, Any]:
                # Note: I'm hoping parsing directly from a file instead of slurping into
                # a string makes this more memory- and compute-efficient.
                with file_path.open(mode="rb") as opened_file:
                    return json.load(opened_file)  # type: ignore[no-any-return]

            json_contents = await anyio.to_thread.run_sync(sync_read_and_parse_json)

            try:
                looks_like_protocol = json_contents["$otSharedSchema"].startswith(
                    "#/protocol/schemas"
                )
            except KeyError:
                looks_like_protocol = False

            if looks_like_protocol:
                assert main_file_info is None, "Multiple main files?"
                # Do minimal data extraction without deep validation:
                schema_version: int = json_contents["schemaVersion"]
                metadata: Metadata = json_contents["metadata"]
                robot_type = json_contents["robot"]["model"]
                main_file_info = MainFileInfo(
                    path=file_path,
                    unvalidated_contents=json_contents,
                    schema_version=schema_version,
                    metadata=metadata,
                    robot_type=robot_type,
                )
                protocol_source_files.append(
                    ProtocolSourceFile(path=file_path, role=ProtocolFileRole.MAIN)
                )
            else:
                # This file doesn't look like a protocol, so assume it's a labware def.
                # TODO: Look at some heuristic fields to be more sure that this is a
                # legit labware file.
                protocol_source_files.append(
                    ProtocolSourceFile(path=file_path, role=ProtocolFileRole.LABWARE)
                )

        assert main_file_info is not None, "No main files?"

        return ProtocolSource(
            directory=directory,
            main_file=main_file_info.path,
            files=protocol_source_files,
            metadata=main_file_info.metadata,
            robot_type=main_file_info.robot_type,
            config=JsonProtocolConfig(schema_version=main_file_info.schema_version),
            # TODO: This is wrong right now. But can we avoid thinking about this
            # by removing labware_definitions from ProtocolSource?
            labware_definitions=JSONParsingLabwareDefinitionsReference(unvalidated_json_protocol=main_file_info.unvalidated_contents),
        )
