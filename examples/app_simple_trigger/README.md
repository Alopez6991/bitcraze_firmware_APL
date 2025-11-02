app_simple_trigger

Simple example that waits for an appchannel packet containing the ASCII string "TRIGGER" and prints "app working" when received.

Build & flash (from repo root):

```
make APP=app_simple_trigger
make APP=app_simple_trigger cload
```

Trigger from host (Python):

```
python3 examples/app_simple_trigger/tools/trigger.py auto TRIGGER
```
