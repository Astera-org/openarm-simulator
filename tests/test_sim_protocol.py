"""Compare the actual Rust wire codec with the official C++ encoder/decoder."""
import json
import random
import struct
import subprocess
import unittest

import openarm_can as oa
from openarm_simulator.experiment import TYPES
from openarm_simulator.native import native_binary


def codec(cases):
    result = subprocess.run([str(native_binary()), 'codec'], input=json.dumps(cases),
                            text=True, capture_output=True, check=True)
    return json.loads(result.stdout)


class ProtocolTests(unittest.TestCase):
    def test_mit_commands_from_official_encoder(self):
        rng = random.Random(41)
        cases, expected = [], []
        for joint in (1, 3, 5):
            official = oa.Motor(TYPES[joint-1], joint, joint+16)
            limits = oa.Motor.get_limit_param(TYPES[joint-1])
            scales = dict(q=(limits.pMax,16), dq=(limits.vMax,12), tau=(limits.tMax,12),
                          kp=(250,12), kd=(2.5,12))
            for _ in range(100):
                values = dict(q=rng.uniform(-limits.pMax,limits.pMax),
                              dq=rng.uniform(-limits.vMax,limits.vMax),
                              tau=rng.uniform(-limits.tMax,limits.tMax),
                              kp=rng.uniform(0,500), kd=rng.uniform(0,5))
                packet = oa.CanPacketEncoder.create_mit_control_command(official, oa.MITParam(**values))
                cases.append(dict(joint=joint, packet=[packet.send_can_id,list(packet.data)]))
                expected.append((values, scales))
        for result, (values, scales) in zip(codec(cases), expected, strict=True):
            for name, value in values.items():
                maximum, bits = scales[name]
                self.assertAlmostEqual(result['motor']['command'][name], value,
                                       delta=2*maximum/((1<<bits)-1)+1e-5)

    def test_feedback_in_official_decoder(self):
        cases = [dict(joint=i,state=[q,v,t,1]) for i in range(1,9)
                 for q,v,t in [(-1.2,-.7,-.5),(1.1,.8,.9),(0,0,0)]]
        for case, output in zip(cases, codec(cases), strict=True):
            i = case['joint']
            official = oa.Motor(TYPES[i-1],i,i+16)
            limits = oa.Motor.get_limit_param(TYPES[i-1])
            result = oa.CanPacketDecoder.parse_motor_state_data(official,output['state'])
            self.assertTrue(result.valid)
            self.assertEqual(result.t_mos,25)
            self.assertEqual(output['state'][0] >> 4,1)
            for field,value,maximum,bits in [('position',case['state'][0],limits.pMax,16),
                                            ('velocity',case['state'][1],limits.vMax,12),
                                            ('torque',case['state'][2],limits.tMax,12)]:
                self.assertAlmostEqual(getattr(result,field),value,delta=2*maximum/((1<<bits)-1)+1e-5)

    def test_parameters_and_rejected_writes(self):
        official = oa.Motor(TYPES[6],7,23)
        cases, expected = [], [(7,23),(8,7),(9,0),(10,1),(21,12.5),(22,30),(23,10)]
        for rid, _ in expected:
            packet = oa.CanPacketEncoder.create_query_param_command(official,rid)
            cases.append(dict(joint=7,packet=[packet.send_can_id,list(packet.data)]))
        for (rid,value), output in zip(expected,codec(cases),strict=True):
            data=output['reply']
            result=oa.CanPacketDecoder.parse_motor_param_data(data)
            self.assertTrue(result.valid)
            self.assertEqual(result.rid,rid)
            self.assertAlmostEqual(struct.unpack('<I' if rid<=10 else '<f',bytes(data[4:]))[0],value)
        cleared, = codec([dict(joint=7,packet=[7,[255]*7+[0xfb]])])
        self.assertEqual(cleared['motor']['status'], 0)
        self.assertEqual(cleared['motor']['command']['kp'], 0)
        cases=[dict(joint=7,packet=[7,[255]*7+[0xfe]])]
        cases += [dict(joint=7,packet=[0x7ff,[7,0,op,rid]+list(struct.pack('<I',value))])
                  for op,rid,value in [(0x55,54,0),(0x55,10,2),(0x33,255,0),(0xaa,0,0)]]
        for output in codec(cases):
            self.assertIn('error',output)
            self.assertEqual(output['motor']['q'],0)
            self.assertEqual(output['motor']['status'],0)
