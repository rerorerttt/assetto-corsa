# DriftAI

Autonomous drift for every AI car in Assetto Corsa. CSP Lua App for Custom Shaders Patch 0.3.2-preview2 (build 4305) and newer.

After a regular offline race session starts, the app automatically finds every AI-driven car, neutralizes its stock AI controller and keeps each car in a controlled slide along the track geometry.

## Installation

1. Install CSP 0.3.2-preview2 or newer (build 4305 required).
2. Copy the DriftAI folder to assettocorsa/apps/lua/DriftAI.
3. Enable the DriftAI app in Content Manager.

## Usage

1. Start any offline session (Practice or Race) and add AI opponents.
2. Open the DriftAI app from the app panel.
3. Press the start button. All AI cars begin to drift around the track.
4. Press stop to hand the cars back to the stock AI.

## Architecture

- manifest.ini: app registration, window, CSP requirements.
- DriftAI.lua: lifecycle, AI scanner, control loop, UI.
- driver.lua: per-car drift controller (state machine + PD).
- track.lua: track-geometry navigation (spline + raycast fallback).

## Control interception

Commands reach the physics through ac.overrideCarControls(carIndex) with combineAxis = false (full input replacement, not combining). The stock AI is additionally neutralized with physics.setAIAggression, setAICaution and setAILevel at 0. Interception health is verified each second via CarControlsInput:active().

## Limitations

- Offline sessions only.
- physics.* calls require scripting physics allowed by the track configuration, usually enabled by default.
