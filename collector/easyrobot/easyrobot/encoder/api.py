'''
Encoder API.

Author: Hongjie Fang.
'''

import re
from easyrobot.encoder.base import EncoderBase
from easyrobot.encoder.cybergear_encoder import CybergearEncoder
from easyrobot.encoder.unitree_encoder import UnitreeEncoder


def get_encoder(**params):
    '''
    Get the force/torque sensor object from the force/torque sensor library.
    '''
    name = params.get('name', None)
    if name is not None:
        del params['name']
    try:
        if re.fullmatch('[ -_]*angle[ -_]*', str.lower(name)):
            return CybergearEncoder(**params)
        else:
            return EncoderBase(**params)
    except Exception:
        return EncoderBase(**params)