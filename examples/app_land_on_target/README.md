app_land_on_target

Simple example that waits for an appchannel packet containing the ASCII string "LAND" and prints when it is received. Use the `tools/trigger.py` helper to send the command from your PC.

Build & flash (from repo root):

```
make APP=app_land_on_target
make APP=app_land_on_target cload
```

Trigger from host (see tools/trigger.py):

```
python3 tools/trigger.py radio://0/80/2M LAND
```
