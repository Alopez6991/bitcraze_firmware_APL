#!/usr/bin/env python3
"""Send a short ASCII trigger string to the Crazyflie appchannel.
Usage: python3 trigger.py [URI|auto] [MESSAGE]
Example: python3 trigger.py auto TRIGGER
"""
import os
import sys
import time

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.appchannel import Appchannel

def main():
    if len(sys.argv) < 3:
        print("Usage: trigger.py <URI> <CMD>")
        print("Example: trigger.py radio://0/80/2M/E7E7E7E709 MISSION")
        sys.exit(1)

    uri = sys.argv[1]
    cmd = sys.argv[2].encode("ascii")

    # Avoid cache warnings
    cache_dir = os.path.join(os.path.dirname(__file__), ".cf_cache")
    os.makedirs(cache_dir, exist_ok=True)

    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
        ac = Appchannel(scf.cf)

        # Send command
        ac.send_packet(cmd)
        print(f"Sent: {cmd!r}", flush=True)
        time.sleep(0.2)  # give the packet time to go out

if __name__ == "__main__":
    main()
