# HTTP administration

Start with `openarm-simulator --model PATH`. Bind address defaults to
`127.0.0.1`, port to `8080`; override with `--host` and `--port`.
This is a local/trusted-network development API without authentication or TLS.
Browser-origin requests are rejected; no cross-origin access is enabled.

All motor commands, engagement, feedback and motor register operations use CAN.
HTTP has no motor-command, enable, disable or stepping endpoint. Physics runs
continuously in Rust at 0.5 ms regardless of HTTP requests.

| Method | Path | Request / result |
| --- | --- | --- |
| GET | `/state` | Motor state, simulation time, counters, plant parameters and configuration hash |
| GET | `/configuration` | Effective startup configuration and configuration hash |
| POST | `/fault` | `["right", 1, {"status": 9}]`; arm, motor ID 1–8, injected fault fields |
| POST | `/push` | `{"right": [0,0,0,0,0,0,0.3]}`; external joint torques in Nm |
| POST | `/reset` | `{}` for default poses, or `{"right": [eight radians], "left": [eight radians]}` |

POST bodies require `Content-Type: application/json` and `Content-Length`,
with at most 16 KiB. Responses are JSON. Invalid input returns 400, unknown
routes 404, wrong content type 415, oversized bodies 413 and unavailable native
service 503. A bad request does not stop the simulator. Administration is serial;
a client must finish sending a request within the socket timeout (2 seconds).

Fault fields are `status` (0 or 2–15) and `silent` (boolean, drop replies).
Status 1 is rejected: enabling a motor requires its CAN command. Status 0 clears
an injected fault and leaves the motor disabled. Normally use the driver's CAN
clear-error command to exercise recovery. Faults remain latched across enable
and disable commands. Reset removes all injected faults and dropped replies.

Pushes affect the seven arm joints, not the grippers. Omitted arms get zero
external torque; `{}` clears all pushes. Forces persist until replaced or reset.
They do not change the held motor command.

Reset returns simulation time to zero, clears commands/faults/external forces,
and leaves motors disabled. It retains plant parameters and configuration
identity. Omitted arms start at physical zero with grippers at −10°. CAN clients
must stop commanding during a reset if they require the world to stay reset;
subsequent CAN traffic is processed normally. Scheduling counters are cumulative.

```sh
curl -H 'Content-Type: application/json' \
  -d '["right",4,{"status":9}]' http://127.0.0.1:8080/fault
curl -H 'Content-Type: application/json' \
  -d '{"right":[0,0,0,0,0,0,0.3]}' http://127.0.0.1:8080/push
curl -H 'Content-Type: application/json' -d '{}' http://127.0.0.1:8080/reset
```

Model selection and plant overrides are startup-only: use `--model PATH` and
`--config plant.json` (or `OPENARM_SIMULATOR_MODEL` and
`OPENARM_SIMULATOR_CONFIG`). The JSON file accepts `poses`, `offsets`, `bodies`,
`joints` and `friction_scale`; see [friction](../friction.md). Restart to replace
the model or configuration. `--report-dir PATH` writes `simulator.json` on exit.

## Lab service compatibility

`--runtime PATH --parent-fd FD` retains the existing Lab manager registration,
`simulator.sock` JSON SEQPACKET endpoint and parent-lifetime shutdown behavior.
The Unix endpoint accepts the existing `inspect`, `fault` and `push` actions.
The model path must now be supplied explicitly or through the environment.
No Lab Python packages are required. HTTP runs in the simulator's network
namespace, so the manager must arrange connectivity for external HTTP clients.
