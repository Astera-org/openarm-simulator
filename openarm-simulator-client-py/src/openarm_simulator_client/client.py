"""Synchronous HTTP control client. Requests are never automatically retried."""

import json
from http import HTTPStatus
from typing import TypeVar, overload
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from serde import SerdeError, to_dict
from serde.json import from_json

from .models import Advance, Configuration, ErrorResponse, Fault, Push, State

T = TypeVar("T")


class APIError(RuntimeError):
    def __init__(self, status: int, message: str):
        self.status = status
        super().__init__(message)


class Client:
    """Control an existing simulator; process lifetime belongs to its launcher.

    Example: Client("http://127.0.0.1:8080").fault("left_joint1", Fault(status=9))
    """

    def __init__(self, url: str = "http://127.0.0.1:8080", *, timeout: float = 5.0):
        parsed = urlsplit(url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError("expected an HTTP or HTTPS simulator URL")
        if parsed.query or parsed.fragment or parsed.username or parsed.password:
            raise ValueError("simulator URL must not contain credentials, query or fragment")
        self.url = url.rstrip("/")
        self.timeout = timeout

    @overload
    def _request(self, method: str, path: str, response: type[T], payload: object = None) -> T: ...

    @overload
    def _request(self, method: str, path: str, response: None, payload: object = None) -> HTTPStatus: ...

    def _request(self, method: str, path: str, response: type[T] | None, payload: object = None) -> T | HTTPStatus:
        body = None
        if payload is not None:
            body = json.dumps(to_dict(payload, skip_none=True), allow_nan=False).encode()
        request = Request(
            self.url + path,
            data=body,
            headers={"Content-Type": "application/json", "Accept": "application/json"},
            method=method,
        )
        try:
            with urlopen(request, timeout=self.timeout) as reply:
                raw = reply.read()
                return HTTPStatus(reply.status) if response is None else from_json(response, raw)
        except HTTPError as error:
            with error:
                raw = error.read()
            try:
                message = from_json(ErrorResponse, raw).error
            except (SerdeError, ValueError, TypeError):
                message = raw.decode(errors="replace") or error.reason
            raise APIError(error.code, message) from error

    def state(self) -> State:
        return self._request("GET", "/state", State)

    def configuration(self) -> Configuration:
        return self._request("GET", "/configuration", Configuration)

    def reset(self) -> HTTPStatus:
        """Restore startup state and pause the clock."""
        return self._request("POST", "/reset", None)

    def pause(self) -> HTTPStatus:
        """200 when changed; 204 when already paused."""
        return self._request("POST", "/pause", None)

    def unpause(self) -> HTTPStatus:
        """200 when changed; 204 when already unpaused."""
        return self._request("POST", "/unpause", None)

    def advance(self, duration_ns: int) -> HTTPStatus:
        """Advance a paused clock without wall pacing; carry incomplete intervals.

        Completion covers simulator work, not consumption of CAN replies by
        other processes. Coordinate commands/replies before the next advance.
        """
        if type(duration_ns) is not int or not 0 <= duration_ns < 2**64:
            raise ValueError("duration_ns must be an unsigned 64-bit integer")
        return self._request("POST", "/advance", None, Advance(duration_ns))

    def fault(self, motor: str, settings: Fault) -> State:
        return self._request("POST", "/fault", State, (motor, settings))

    def push(self, torques: Push) -> State:
        return self._request("POST", "/push", State, torques)
