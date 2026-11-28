# RoutePilot — CSP Traffic integration (experimental)

RoutePilot is a Custom Shaders Patch Lua app for Assetto Corsa. This branch adds an optional navigation layer based on the active track's CSP Traffic Tool `traffic.json`. It imports directed lanes and intersections, validates lane samples against the physical track surface, builds a directed road graph and route candidates, and uses the existing RoutePilot driver to control active AI cars. The recorded-route controller remains available as a fallback.

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

Route selection prefers genuine directed loops. If the exported road network has no usable cycle, RoutePilot falls back to long open one-way routes only when they end at a physical lane endpoint, never at an unserved intersection. It does not invent reverse links or cut across the map. Active AI cars are placed only after a complete spawn plan has passed occupancy checks. On open routes, cars brake to a controlled stop before the terminal and are recycled to clear, spaced entrance slots only after leaving render range. The player remains under manual control unless Assetto Corsa's native Ctrl+8 AI mode is active. When Ctrl+8 is active and a player-aligned graph route exists, RoutePilot adopts car 0.

The Dev tab shows import, graph, route, road-boundary and driver diagnostics. The app writes `routepilot_trace.txt` for troubleshooting.

## Recorded-route fallback

If the layout has no usable CSP Traffic Tool data, RoutePilot can use a saved recorded route. In the Route tab, record the line and boundaries, keep the loop option enabled for a lap, then save the route for that track. Route and boundary data are stored per track/layout.

## Validation status

The branch includes automated Lua 5.1 syntax validation and regression tests for loop topology, directed-cycle search, vertical surface projection, CSP Bezier trajectories, and a representative real CSP Traffic Tool export. The fixture graph contains 25 usable lanes, 14 intersections, 31 links and 25 directed movements; its road-corridor builder also generates left/right boundaries for every lane. It has no directed cycles, so the open-route fallback is exercised. These tests run in GitHub Actions with synthetic road-surface hits; render-based wall raycasts are not available in CI. They do not validate physical CSP raycasts, wall clearance, vehicle handling, collisions, overtaking, recycling or long-session stability.

This branch is experimental and is not considered release-ready until the complete app has been run in Assetto Corsa/CSP on representative track layouts with multiple AI cars and the resulting traces have passed review.
