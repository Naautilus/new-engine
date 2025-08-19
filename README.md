Welcome to new-engine: a physics simulation with aircraft, missiles, debris and collision physics.

<img width="2755" height="811" alt="new-engine" src="https://github.com/user-attachments/assets/cbe4eecc-ebf4-4a9e-acee-2179f0b5ab42" />

An executable (/build/main.exe) is available for Windows x86-64. You will need OpenGL and GLFW installed, and you will need the /models/ and /scenarios/ folders alongside the /build/ folder. I run it with MSYS2 MinGW64.

Run it with the arguments:

```./main.exe -scenario ground```

to load /scenarios/scenario_ground.json. You can change `ground` to whichever scenario you want to see.

Controls:

P - Pause/unpause. IT STARTS PAUSED! so you will need to press P to start the simulation.

0 - Free camera. WASDQE for translation, IJKLUO for rotation, Left Shift to speed up, [ and \] for FOV control.

When outside of free camera:

For your jet:

WASDQE - Pitch/yaw/roll controls.

Z/X - Throttle up/down.

Space - Fire missile.


For all other jets (all controlled simultaneously; other jets will only be present in some scenarios):

IJKLUO - Pitch/yaw/roll controls.

N/M - Throttle up/down.

B - Fire missile.
