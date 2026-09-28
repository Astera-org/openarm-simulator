"""OpenArm simulation service: owns physics, CAN setup assumptions and metadata."""
import argparse
import contextlib
from http.server import HTTPServer
import json
import os
from pathlib import Path
import select
import signal

from . import ipc
from .http import Handler
from .native import NativeSimulator, native_binary
from .metadata import save_metadata
from .model import model_path


def load_configuration(runtime_config, path=None):
    """A file replaces the runtime simulator object; configuration is startup-only."""
    plant = json.loads(Path(path).expanduser().read_text()) if path else runtime_config.get('simulator', {})
    allowed = {'poses', 'offsets', 'bodies', 'joints', 'friction_scale'}
    if not isinstance(plant, dict) or set(plant)-allowed:
        raise ValueError('Simulator config must contain only poses, offsets, bodies, joints and friction_scale')
    return plant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare', action='store_true', help='Build the Rust engine and exit')
    parser.add_argument('--model', type=Path, help='OpenArm v1 scene.xml (or OPENARM_SIMULATOR_MODEL)')
    parser.add_argument('--host', default='127.0.0.1', help='HTTP bind address (default: loopback)')
    parser.add_argument('--port', type=int, default=8080, help='HTTP port (default: 8080)')
    parser.add_argument('--report-dir', type=Path, help='Write simulator.json on shutdown')
    parser.add_argument('--runtime', type=Path, help='Optional Lab service runtime directory')
    parser.add_argument('--parent-fd', type=int, help='Optional manager lifetime descriptor')
    parser.add_argument('--config', type=Path, default=os.environ.get('OPENARM_SIMULATOR_CONFIG'),
                        help='Startup plant JSON (poses, offsets, bodies, joints, friction_scale)')
    args = parser.parse_args()
    if args.prepare:
        native_binary()
        return
    if (args.runtime is None) != (args.parent_fd is None):
        parser.error('--runtime and --parent-fd must be supplied together')
    try:
        model = model_path(args.model)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    config = json.loads((args.runtime/'config.json').read_text()) if args.runtime else {}
    plant = load_configuration(config, args.config)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)
    sim = NativeSimulator(model=model, **plant)
    try:
        with contextlib.ExitStack() as stack:
            http = stack.enter_context(HTTPServer((args.host, args.port), Handler))
            http.sim = sim
            readers = [http]
            server = None
            if args.runtime:
                socket_path = args.runtime/'simulator.sock'
                server = ipc.listen(socket_path)
                stack.callback(socket_path.unlink, missing_ok=True)
                stack.enter_context(server)
                readers.extend([server, args.parent_fd])
                ipc.write_json(args.runtime/'simulator-physics.json',
                               dict(config_sha256=sim.config_sha256, configuration=sim.configuration))
                ipc.register(args.runtime, 'simulator', dict(socket=str(socket_path),
                             transport='socketcan', interfaces=['can0', 'can1'], config_sha256=sim.config_sha256))
            print(f'HTTP administration: http://{args.host}:{http.server_port}', flush=True)
            # ponytail: serial administration; add concurrency only if inspection throughput needs it.
            while not stopped:
                sim.check()
                readable, _, _ = select.select(readers, [], [], .05)
                if args.parent_fd in readable and not os.read(args.parent_fd, 1):
                    break
                if http in readable:
                    http.handle_request()
                if server is not None and server in readable:
                    client, _ = server.accept()
                    with client:
                        client.settimeout(2)
                        try:
                            message = ipc.decode(client.recv(ipc.MAX_PACKET))
                            if message.get('action', 'inspect') not in ('inspect', 'fault', 'push'):
                                raise ValueError('Unsupported Lab administration action')
                            reply = sim.rpc(message.get('action', 'inspect'), message.get('payload'))
                            client.sendall(ipc.encode(reply))
                        except (OSError, RuntimeError, ValueError) as exc:
                            with contextlib.suppress(OSError):
                                client.sendall(ipc.encode(dict(error=str(exc))))
        if not stopped and sim.process.poll() is not None:
            raise SystemExit(f'Simulator exited ({sim.process.returncode})')
    finally:
        report_dir = args.report_dir or config.get('report_dir')
        if report_dir:
            with contextlib.suppress(OSError, RuntimeError, ValueError):
                save_metadata(sim, Path(report_dir), model)
        sim.close()


if __name__ == '__main__':
    main()
