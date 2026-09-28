import json


def save_metadata(sim, directory, model):
    state = sim.rpc()
    metadata = dict(backend='mujoco-socketcan', engine='rust', mujoco_version=state['mujoco_version'], model=str(model),
                        model_sha256=sim.configuration['model_sha256'],
                        timestep_s=state['timestep_s'], statistics=state['statistics'],
                        joint_stop_solref=state['joint_stop_solref'],
                        joint_stop_solimp=state['joint_stop_solimp'],
                        plant=state['plant'],
                        config_sha256=sim.config_sha256, configuration=sim.configuration,
                        final_state=state['state'],
                        assumptions=['Nominal XML plus recorded assembly/friction overrides; synthetic parameters',
                                     'Stiff numerical joint stops; compliance is not measured hardware compliance',
                                     'Ideal MIT current/torque response; no firmware electrical/thermal model',
                                     'Linear symmetric gripper transmission; overlapping finger meshes excluded',
                                     'TIMEOUT=0; no emulated firmware watchdog',
                                     'Virtual CAN has no automatic USB/arbitration delay model'])
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'simulator.json').write_text(json.dumps(metadata, indent=2)+'\n')
