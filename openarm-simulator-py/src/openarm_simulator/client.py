"""Synchronous HTTP control client. Requests are never automatically retried."""

import json
from typing import TypeVar
from urllib.error import HTTPError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from serde import SerdeError, to_dict
from serde.json import from_json

from .models import Arm, Configuration, ErrorResponse, Fault, Push, Reset, State

T = TypeVar("T")


class APIError(RuntimeError):
    def __init__(self, status: int, message: str):
        self.status = status
        super().__init__(message)


class Client:
    """Control an existing simulator; process lifetime belongs to its launcher.

    Example: Client("http://127.0.0.1:8080").fault("left", 1, Fault(status=9))
    """

    def __init__(self, url: str = "http://127.0.0.1:8080", *, timeout: float = 5.0):
        parsed = urlsplit(url)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise ValueError("expected an HTTP or HTTPS simulator URL")
        if parsed.query or parsed.fragment or parsed.username or parsed.password:
            raise ValueError("simulator URL must not contain credentials, query or fragment")
        self.url = url.rstrip("/")
        self.timeout = timeout

    def _request(self, method: str, path: str, response: type[T], payload: object = None) -> T:
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
                return from_json(response, reply.read())
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

    def reset(self, poses: Reset | None = None) -> State:
        return self._request("POST", "/reset", State, poses if poses is not None else Reset())

    def fault(self, arm: Arm, joint: int, settings: Fault) -> State:
        return self._request("POST", "/fault", State, (arm, joint, settings))

    def push(self, torques: Push) -> State:
        return self._request("POST", "/push", State, torques)
