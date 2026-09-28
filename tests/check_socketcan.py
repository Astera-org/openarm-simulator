"""End-to-end HTTP administration and actual CAN-FD commands. See docs/setup.md."""
import contextlib
import http.client
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import tempfile
from urllib.parse import urlsplit

from openarm_simulator.interfaces import require_virtual_interfaces
from openarm_simulator.model import model_path
from openarm_simulator.native import native_binary
from openarm_simulator import ipc


def main():
    require_virtual_interfaces()
    native_binary()
    with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
        log = stack.enter_context(open(Path(directory)/'service.log', 'w+'))
        service = subprocess.Popen(
            [sys.executable, '-m', 'openarm_simulator.service', '--model', str(model_path()),
             '--port', '0', '--report-dir', directory], stdout=subprocess.PIPE, stderr=log, text=True)
        try:
            assert select.select([service.stdout], [], [], 30)[0], 'Service startup timed out'
            line = service.stdout.readline()
            assert line.startswith('HTTP administration: '), line
            url = urlsplit(line.strip().split(' ', 2)[2])

            def request(path, payload=None, *, status=200, headers=None, raw=None):
                connection = http.client.HTTPConnection(url.hostname, url.port, timeout=5)
                try:
                    body = raw if raw is not None else json.dumps(payload) if payload is not None else None
                    connection.request('POST' if body is not None else 'GET', path, body=body,
                                       headers=headers if headers is not None else {'Content-Type': 'application/json'})
                    response = connection.getresponse()
                    result = json.loads(response.read())
                    assert response.status == status, (response.status, result)
                    return result
                finally:
                    connection.close()

            identity = request('/configuration')['config_sha256']
            frame = struct.Struct('=IBB2x64s')
            for interface, side in [('can0', 'right'), ('can1', 'left')]:
                bus = stack.enter_context(socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW))
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
                bus.settimeout(2)
                bus.bind((interface,))

                def command(data, expected, can_id=1):
                    bus.send(frame.pack(can_id, 8, 1, bytes(data)))
                    reply_id, length, flags, data = frame.unpack(bus.recv(72))
                    assert (reply_id, length, data[0] >> 4) == (17, 8, expected)

                command([255]*7+[0xfc], 1)  # Enable through CAN.
                request('/fault', [side, 1, {'status': 9}])
                command([1, 0, 0xcc, 0, 0, 0, 0, 0], 9, 0x7ff)
                command([255]*7+[0xfb], 0)  # Clear fault through CAN.
                command([255]*7+[0xfc], 1)
                command([255]*7+[0xfd], 0)  # Disable through CAN.

            # Invalid requests must not terminate the engine or enable a motor.
            for payload in (['right', 1, {'status': 1}], ['wrong', 1, {}],
                            ['right', 0, {}], ['right', 1, {'status': 16}], {}, None):
                request('/fault', raw=json.dumps(payload), status=400)
            request('/push', {'right': [1]*8}, status=400)
            request('/reset', {'right': [0]*7}, status=400)
            request('/reset', raw='{"right": [NaN]}', status=400)
            request('/reset', raw='{', status=400)
            request('/reset', raw='{}', headers={'Content-Type': 'text/plain'}, status=415)
            request('/reset', raw=' '*16385, status=413)
            request('/reset', {}, headers={'Origin': 'https://example.com'}, status=403)
            request('/command', {}, status=404)
            request('/step', {}, status=404)
            request('/push', {'right': [0.1]*7})
            before = request('/state')
            assert before['plant']['applied_torque_nm']['right'] == [0.1]*7
            assert before['statistics']['commands'] >= 10
            after = request('/reset', {})
            assert after['time'] == 0
            assert after['plant']['applied_torque_nm']['right'] == [0.0]*7
            assert all(m['status'] == 0 for arm in after['state'].values() for m in arm)
            assert request('/configuration')['config_sha256'] == identity
            assert request('/state')['time'] > 0
            service.terminate()
            assert service.wait(10) == 0
            report = json.loads((Path(directory)/'simulator.json').read_text())
            assert report['config_sha256'] == identity
            service.stdout.close()

            # The optional Lab adapter has no imports or filesystem dependencies on Lab.
            runtime = Path(directory)
            (runtime/'config.json').write_text(json.dumps({'report_dir': directory}))
            manager = stack.enter_context(ipc.listen(runtime/'manager.sock'))
            read_fd, write_fd = os.pipe()
            parent_read = stack.enter_context(os.fdopen(read_fd, 'rb', buffering=0))
            parent_write = stack.enter_context(os.fdopen(write_fd, 'wb', buffering=0))
            service = subprocess.Popen(
                [sys.executable, '-m', 'openarm_simulator.service', '--model', str(model_path()),
                 '--port', '0', '--runtime', directory, '--parent-fd', str(read_fd)],
                pass_fds=(read_fd,), env=dict(os.environ, HQ_SERVICE_INSTANCE='test-instance'),
                stdout=subprocess.PIPE, stderr=log, text=True)
            assert select.select([manager], [], [], 30)[0], 'Manager registration timed out'
            registration, _ = manager.accept()
            with registration:
                message = ipc.decode(registration.recv(ipc.MAX_PACKET))
                assert (message['op'], message['name'], message['instance']) == ('register', 'simulator', 'test-instance')
                registration.sendall(ipc.encode({'ok': True}))
            assert select.select([service.stdout], [], [], 5)[0]
            assert service.stdout.readline().startswith('HTTP administration: ')
            assert ipc.call(runtime/'simulator.sock', {'action': 'inspect'})['config_sha256'] == identity
            # Closing the parent's write end must stop the managed simulator.
            parent_write.close()
            assert service.wait(10) == 0
            assert not (runtime/'simulator.sock').exists()
            print('PASS: HTTP, CAN-FD on both buses, invalid input, reset, reports and Lab IPC lifecycle')
        except BaseException:
            log.flush()
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            if service.poll() is None:
                service.terminate()
                try:
                    service.wait(10)
                except subprocess.TimeoutExpired:
                    service.kill()
                    service.wait()
            service.stdout.close()


if __name__ == '__main__':
    main()
