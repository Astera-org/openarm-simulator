"""Small HTTP administration API; ordinary motor commands only travel over CAN."""
import contextlib
from http.server import BaseHTTPRequestHandler
import json


class Handler(BaseHTTPRequestHandler):
    timeout = 2

    def reply(self, status, value):
        data = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.dispatch()

    def do_POST(self):
        self.dispatch()

    def dispatch(self):
        try:
            # No CORS: a website must not be able to administer a local simulator.
            if self.headers.get('Origin'):
                self.reply(403, dict(error='Browser-origin requests are not enabled'))
                return
            sim = self.server.sim
            if self.command == 'GET' and self.path == '/state':
                result = sim.rpc()
            elif self.command == 'GET' and self.path == '/configuration':
                result = dict(configuration=sim.configuration, config_sha256=sim.config_sha256)
            elif self.command == 'POST' and self.path in ('/fault', '/push', '/reset'):
                if self.headers.get_content_type() != 'application/json':
                    self.reply(415, dict(error='Use Content-Type: application/json'))
                    return
                lengths = self.headers.get_all('Content-Length', [])
                if len(lengths) != 1 or self.headers.get('Transfer-Encoding'):
                    raise ValueError('Supply one Content-Length; chunked requests are not supported')
                length = int(lengths[0])
                if not 0 < length <= 16384:
                    self.reply(413, dict(error='Request body must be 1..16384 bytes'))
                    return
                body = self.rfile.read(length)
                if len(body) != length:
                    raise ValueError('Incomplete request body')
                payload = json.loads(body)
                # Reject non-finite JSON before it crosses the native boundary.
                json.dumps(payload, allow_nan=False)
                result = sim.rpc(self.path[1:], payload)
            else:
                self.reply(404, dict(error='Unknown administration endpoint'))
                return
            self.reply(200, result)
        except (ValueError, UnicodeError, RecursionError) as exc:
            with contextlib.suppress(OSError):
                self.reply(400, dict(error=str(exc)))
        except (OSError, RuntimeError) as exc:
            with contextlib.suppress(OSError):
                self.reply(503, dict(error=str(exc)))
