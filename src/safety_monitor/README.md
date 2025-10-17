## Description
This package contains two entry points:
1. [`proximity_sensor`](./safety_monitor/proximity_sensor.py): Simulates a noisy proximity sensor by generating a cosine wave at 0.05 Hz and 1200 mm amplitude. 
2. [`red_button`](./safety_monitor/red_button.py): Simulates an emergency shutdown button. Hitting enter on the terminal window toggles the status of the button between `"PRESSED"` and `"RELEASED"`.
Specifications for both of these entry points are input in [`config.py`](./safety_monitor/config.py).