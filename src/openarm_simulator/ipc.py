"""Shared service registration and local JSON request/response transport."""
import json
import os
from pathlib import Path
import socket
import time

MAX_PACKET = 262144


def encode(message):
    packet = json.dumps(message, allow_nan=False, separators=(',', ':')).encode()
    if len(packet) > MAX_PACKET:
        raise ValueError('Local message too large')
    return packet


def decode(packet):
    if not packet:
        raise ConnectionError('Service disconnected')
    message = json.loads(packet)
    if not isinstance(message, dict):
        raise ValueError('Expected a message object')
    return message


def listen(path):
    path = Path(path)
    path.unlink(missing_ok=True)  # Caller holds the runtime or service ownership lock.
    server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    server.bind(str(path))
    path.chmod(0o600)
    server.listen(16)
    server.setblocking(False)
    return server


def call(path, message, timeout=5):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(timeout)
        client.connect(str(path))
        client.sendall(encode(message))
        deadline = time.monotonic() + timeout
        while True:
            client.settimeout(max(.001, deadline - time.monotonic()))
            reply = decode(client.recv(MAX_PACKET))
            if 'request_id' not in message or reply.get('request_id') == message['request_id']:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('No matching reply')
        if 'error' in reply:
            raise RuntimeError(reply['error'])
        return reply


def register(runtime, name, endpoint):
    return call(Path(runtime)/'manager.sock', dict(op='register', name=name,
                instance=os.environ['HQ_SERVICE_INSTANCE'], endpoint=endpoint))


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w') as file:
        json.dump(value, file, allow_nan=False)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
