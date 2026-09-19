import ast
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "mkmini_cmd_adapter"
READ_ONLY_MODULE = PACKAGE_DIR / "receive_only.py"
TX_MODULE = PACKAGE_DIR / "tx.py"
NATIVE_SENDER_MODULE = PACKAGE_DIR / "native_sender.py"


def test_pure_package_has_no_real_can_import_or_transport_calls():
    forbidden_imports = {"can", "python_can", "socket", "socketcan"}
    forbidden_names = {"socket", "cansend", "candump"}
    for path in PACKAGE_DIR.glob("*.py"):
        if path in (READ_ONLY_MODULE, TX_MODULE, NATIVE_SENDER_MODULE):
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(alias.name.split(".")[0] not in forbidden_imports for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                assert (node.module or "").split(".")[0] not in forbidden_imports
            elif isinstance(node, ast.Name):
                assert node.id not in forbidden_names
            elif isinstance(node, ast.Attribute):
                assert node.attr not in forbidden_names


def test_receive_only_module_has_no_command_or_output_surface():
    tree = ast.parse(READ_ONLY_MODULE.read_text(encoding="utf-8"), filename=str(READ_ONLY_MODULE))
    imported_names = {
        alias.name.rsplit(".", 1)[-1]
        for node in ast.walk(tree)
        if isinstance(node, (ast.Import, ast.ImportFrom))
        for alias in node.names
    }
    assert "MkminiCanCodec" not in imported_names
    assert "KeyboardFrameScheduler" not in imported_names
    method_names = {
        node.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    assert not method_names.intersection({"send", "write", "transmit", "sendall", "sendto"})
    assert {"open", "receive", "close"}.issubset(method_names)


def test_tx_transport_is_not_imported_by_read_only_module():
    tree = ast.parse(READ_ONLY_MODULE.read_text(encoding="utf-8"), filename=str(READ_ONLY_MODULE))
    imported_modules = {
        (node.module or "")
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    assert not any(module.endswith(".tx") or module == "tx" for module in imported_modules)


def test_tx_transport_has_only_explicit_frame_send_surface():
    tree = ast.parse(TX_MODULE.read_text(encoding="utf-8"), filename=str(TX_MODULE))
    methods = {
        node.name
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    assert {"open", "send_frame", "close"}.issubset(methods)
    assert "encode_ctrl_cmd" not in TX_MODULE.read_text(encoding="utf-8")
    assert "require_tx" in TX_MODULE.read_text(encoding="utf-8")


def test_native_sender_python_bridge_uses_only_local_unix_datagrams():
    source = NATIVE_SENDER_MODULE.read_text(encoding="utf-8")
    assert "socket.AF_UNIX" in source
    assert "socket.SOCK_DGRAM" in source
    assert "PF_CAN" not in source
    assert "CAN_RAW" not in source
    assert "SocketCanTxTransport" not in source
