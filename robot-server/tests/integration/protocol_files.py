"""Tools for temporary protocol files."""
from pathlib import Path
import tempfile
from typing import IO
from typing_extensions import Literal


def get_protocol(
    protocol_name: str, protocol_extension: Literal[".py", ".json"]
) -> str:
    """A NamedTemporaryFile valid json protocol."""
    contents = ""
    if protocol_extension == ".py":
        path = "./tests/integration/protocols/simple.py"
    else:
        # TODO: The old implementation of this test helper appears to be
        # uploading a v3 JSON protocol file, which I'm not sure the server even supports
        # anymore? It breaks my proof of concept because $otSharedSchema doesn't exist
        # in v3 protocol files.
        #
        # If the server supports v3 protocols still, we need to find a better heuristic
        # for detecting protocol files. opentrons.parse might already have a helper for
        # that.
        #
        # If the server doesn't support v3 protocols anymore, we should figure out
        # why it hasn't been rejecting this file, and we'll need to update this test
        # helper.
        path = "./tests/integration/protocols/simple_v6.json"
    with open(Path(path)) as f:
        contents = f.read()
        contents = contents.replace(
            '"protocolName": "simple"', f'"protocolName": "{protocol_name}"'
        )
    return contents


def get_json_protocol(protocol_name: str) -> IO[bytes]:
    """A NamedTemporaryFile valid python protocol."""
    return create_temp_protocol(".json", get_protocol(protocol_name, ".json"))


def get_py_protocol(protocol_name: str) -> IO[bytes]:
    """A NamedTemporaryFile valid python protocol."""
    return create_temp_protocol(".py", get_protocol(protocol_name, ".py"))


def create_temp_protocol(
    protocol_extension: Literal[".py", ".json"], protocol_contents: str
) -> IO[bytes]:
    """Create a temporary protocol file."""
    file = tempfile.NamedTemporaryFile(suffix=protocol_extension)
    file_contents = protocol_contents
    file.write(str.encode(file_contents))
    file.seek(0)
    return file
