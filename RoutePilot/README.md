# RoutePilot — CSP Traffic integration (experimental)

RoutePilot is a Custom Shaders Patch Lua app for Assetto Corsa. This branch adds an optional navigation layer based on the active track's CSP Traffic Tool `traffic.json`. It imports directed lanes and intersections, validates lane samples against the physical track surface, builds a directed road graph and closed routes, and uses the existing RoutePilot driver to control active AI cars. The recorded-route controller remains available as a fallback.

## Requirements

- Assetto Corsa with Custom Shaders Patch. Development and API checks target CSP 0.3.2-preview2.
- A track layout with a CSP Traffic Tool `traffic.json` for graph-based traffic.
- Lua Apps enabled in Content Manager.

## Installation

Copy the complete `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Keep the subfolders `lib`, `shared`, and `tests` in place. Enable RoutePilot in Content Manager's Lua Apps list and open the app in a session.

## Graph-based traffic

When the active layout provides a readable `traffic.json`, RoutePilot imports it automatically. The importer keeps lane direction and intersection definitions, samples lane splines, projects samples onto the physical track surface, rejects unsupported or vertically mismatched samples, then builds road corridors and a directed intersection graph. Route selection prefers lanes aligned with the player's heading and retains other reachable directions for traffic.

Active AI cars are assigned to available closed routes and placed only after a complete spawn plan has passed occupancy checks. The player remains under manual control unless Assetto Corsa's native Ctrl+8 AI mode is active. When Ctrl+8 is active and a player-aligned graph route exists, RoutePilot adopts car 0.

The Dev tab shows import, graph, route, road-boundary and driver diagnostics. The app writes `routepilot_trace.txt` for troubleshooting.

## Recorded-route fallback

If the layout has no usable CSP Traffic Tool data, RoutePilot can use a saved recorded route. In the Route tab, record the line and boundaries, keep the loop option enabled for a lap, then save the route for that track. Route and boundary data are stored per track/layout.

## Validation status

The branch includes automated Lua 5.1 syntax validation and synthetic regression tests for loop topology, directed-cycle search and vertical surface projection. These tests run in GitHub Actions. They do not validate CSP API behavior on a real track, vehicle handling, collisions, overtaking, recycling or long-session stability.

This branch is experimental and is not considered release-ready until the complete app has been run in Assetto Corsa/CSP on representative track layouts with multiple AI cars and the resulting traces have passed review.
