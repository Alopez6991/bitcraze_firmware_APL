#!/usr/bin/env python3
"""Send a short ASCII trigger string to the Crazyflie appchannel.
Usage: python3 trigger.py [URI|auto] [MESSAGE]
Example: python3 trigger.py auto TRIGGER
"""
import sys
import time
import threading
from cflib.crazyflie import Crazyflie
import cflib.crtp

if len(sys.argv) < 3:
    print('Usage: trigger.py <URI|auto> <MESSAGE>')
    sys.exit(1)

uri = sys.argv[1]
msg = sys.argv[2].encode('ascii')

cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie()

# Event used to notify the main thread that the packet was sent and
# the connection can be closed from the main thread (avoids joining
# the callback thread from within itself).
done = threading.Event()

def connected(link_uri):
    print('Connected to', link_uri)
    cf.appchannel.send_packet(msg)
    print('Sent:', msg)
    # Small delay to ensure the packet is queued; then signal main thread
    time.sleep(0.2)
    done.set()

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
        # Wait for the connected callback to signal we're done sending
        # or for the user to interrupt. Timeout gives a safety net.
        while not done.wait(timeout=0.1):
            # keep looping until signalled
            continue
    except KeyboardInterrupt:
        pass
    finally:
        # Close the link from the main thread
        if cf.is_connected:
            cf.close_link()

if __name__ == '__main__':
    main()
