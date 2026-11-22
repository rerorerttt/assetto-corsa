# AC DriftLab 0.1.0

Experimental AI-control and telemetry app for Assetto Corsa with CSP 0.3.2-preview2.

## Features

- Controls only steering, throttle and brake through ac.overrideCarControls().
- Does not send gear-up, gear-down or requested-gear commands: shifting remains with the car AI.
- Does not read, follow, generate or require fast_lane.ai / AI spline.
- Uses bounded, ramp-limited experimental phases: steering + throttle, steering + coast, short brake pulses, and recovery.
- Stops if the selected car disappears or ceases to be AI-controlled.
- Applies a recovery command if speed exceeds 70 km/h or absolute yaw rate exceeds 2.5 rad/s.
- Records telemetry at 10 Hz and event/phase/error records.

## Install

Copy the AC_DriftLab folder into assettocorsa/apps/lua/AC_DriftLab/.

Enable Lua apps in Content Manager/CSP and open AC DriftLab from the in-game app sidebar.

The app requires CSP Lua I/O access to save logs. If I/O is unavailable, it refuses to start and displays the reason.

## First test

Use an empty practice session and select an AI car (player car is index 0; the first AI is commonly index 1). Keep the surroundings clear and observe the car throughout the experiment. Press STOP at any time.

Logs are saved under the app's CSP ScriptConfig directory, in AC_DriftLab, as:
- <experiment>_telemetry.csv
- <experiment>_events.csv

## Limits

This is a data-gathering prototype, not a finished autonomous drift controller. It does not constrain the car to a track boundary and does not use an AI line. It has not yet been runtime-tested in the user's CSP installation. The next iteration depends on the first telemetry CSV and observed in-game response.

## SDK basis

Implementation follows lua-sdk/ac_apps/lib.lua declarations for ac.overrideCarControls, ac.CarControlsInput, ac.getCar, ac.FolderID.ScriptConfig, io.available, io.createDir, io.save, and ac.onRelease.
