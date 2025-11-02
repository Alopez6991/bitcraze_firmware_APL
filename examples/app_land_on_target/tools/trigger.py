#!/usr/bin/env python3
"""Send a short ASCII trigger string to the Crazyflie appchannel.
Usage: python3 trigger.py [URI] [MESSAGE]
Example: python3 trigger.py radio://0/80/2M LAND
"""
import sys
import time
from cflib.crazyflie import Crazyflie
import cflib.crtp

if len(sys.argv) < 3:
    print('Usage: trigger.py <URI|auto> <MESSAGE>')
    sys.exit(1)

uri = sys.argv[1]
msg = sys.argv[2].encode('ascii')

# Initialize low-level drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

cf = Crazyflie()

def connected(link_uri):
    print('Connected to', link_uri)
    cf.appchannel.send_packet(msg)
    print('Sent:', msg)
    time.sleep(0.5)
    cf.close_link()

def main():
    if uri == 'auto':
        available = cflib.crtp.scan_interfaces()
        if not available:
            print('No Crazyflies found')
            return
        use_uri = available[0][0]
        print('Auto-using', use_uri)
    else:
        use_uri = uri

    cf.connected.add_callback(connected)
    cf.open_link(use_uri)

    try:
        # Wait until connection closes
        while cf.is_connected:
            time.sleep(0.1)
    except KeyboardInterrupt:
        cf.close_link()

if __name__ == '__main__':
    main()
