"""Linux virtual CAN isolation checks shared by simulated service entry points."""
import json
import subprocess


def require_virtual_interfaces():
    # Check both buses before allowing simulation-only confirmation bypass.
    # The native simulator independently enforces the same rule.
    for name in ('can0', 'can1'):
        result = subprocess.run(['ip', '-j', '-d', 'link', 'show', 'dev', name],
                                check=True, capture_output=True, text=True)
        link, = json.loads(result.stdout)
        if link.get('linkinfo', {}).get('info_kind') != 'vcan' or link['mtu'] < 72:
            raise RuntimeError(f'{name} must be CAN-FD capable vcan; refusing a physical interface')
