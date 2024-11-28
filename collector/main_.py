import os
import time
import json
import argparse
from pynput import keyboard
from easydict import EasyDict as edict
from easyrobot.encoder.cybergear_encoder import CybergearEncoder
from easyrobot.encoder.unitree_encoder import UnitreeEncoder


if __name__ == '__main__':
    os.system("kill -9 `ps -ef | grep collector | grep -v grep | awk '{print $2}'`")
    os.system('rm -f /dev/shm/*')
    os.system('udevadm trigger')

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--task', '-t', 
        default = 'test', 
        help = 'task name', 
        type = str,
        choices = ['grasp', 'test']
    )
    args = parser.parse_args()

    run_path = 'configs/ave_arm' + '_' + str(args.task) + '.json'

    if not os.path.exists(run_path):
        raise AttributeError('Please provide the configuration file {}.'.format(run_path))
    with open(run_path, 'r') as f:
        cfgs = edict(json.load(f))

    tid = int(input('Task ID: '))
    sid = int(input('Scene ID: '))
    uid = int(input('User ID: '))
 
    Unitree_encoder = UnitreeEncoder(**cfgs.encoder_Unitree)
    Unitree_encoder.streaming()
    Cybergear_encoder = CybergearEncoder(**cfgs.encoder_Cybergear)
    CyberGear_encoder.streaming()
    has_start = False
    has_stop = False
    print("start")

    def _on_press(key):
        global has_start
        global has_stop
        try:
            if key.char == 'q':
                if not has_start:
                    pass
                else:
                    encoder_right.stop_streaming()
                    encoder_left.stop_streaming()
                    has_stop = True
            if key.char == 's':
                if not has_start:
                    os.system('bash collector/collector.sh {} {} {} {} &'.format(run_path, tid, sid, uid))
                    time.sleep(3)
                    has_start = True
                else:
                    pass     
        except AttributeError:
            pass

    def _on_release(key):
        pass

    listener = keyboard.Listener(on_press = _on_press, on_release = _on_release)
    listener.start()
    while not has_stop:
        pass
    listener.stop()
        