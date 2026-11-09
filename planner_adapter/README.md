# planner_adapter

Runs different graph planners through one shared pipeline, so they're compared on identical inputs
and measured identically, either inside Nav2 or offline on a saved map.

```
global costmap ─► fromCostmap ─► grid_graph::buildGraph ─► fill rule costs ─► snap start/goal
 (or saved map)                                            (from rulebook_file)        │
                                                                                       ▼
          CSV row ◄── validate path uses real edges, time it, per-rule path costs ◄── GraphPlanner::plan
```

Everything except `plan()` is shared (`src/pipeline.cpp`), so a difference in results comes from
the planner and nothing else.

## Planners

| name | what it does |
|---|---|
| `dijkstra` | Shortest path by edge length; ignores rules. Baseline. |
| `rulebook` | The planning library's `RulebookPlanner`, unchanged, on the adapter's graph and rule costs. Needs a totally ordered rulebook. |

## Adding a planner

1. Implement `planner_adapter::GraphPlanner` (`include/planner_adapter/graph_planner.hpp`) in
   `src/planners/`. `src/planners/dijkstra_planner.cpp` is the template and the baseline.
2. Add one line to `plannerRegistry()` in `src/planner_registry.cpp`.
3. Add the `.cpp` to `planner_adapter_core` in `CMakeLists.txt`.

`plan()` gets a `PlanRequest`: a freshly built graph, start and goal node ids, and the loaded
rulebook (or null). With a rulebook, `Edge::costs[i]` already holds rule `i`'s value, so every
planner ranks identical costs. Return node ids; extra numbers go in `PlanResult::stats` and land
in the CSV's `stats` column.

## Rulebooks

A rulebook is a YAML file (`config/rulebooks/example.yaml` documents the format) naming rules,
how each aggregates along a path (`sum` / `max`), which **metric** scores an edge for it, and the
priorities between rules. It is loaded into the planning library's `Rulebook` /
`RulebookCost`, so the same ordering semantics apply.

Point `rulebook_file` at it (Nav2 YAML, or `--set rulebook_file=...` offline). Every CSV row then
records the rulebook (`rulebook` column, e.g. `example.yaml: keep_clearance > min_length`) and
the path's value under each rule (`rule_costs`), whichever planner produced the path.

YAML can choose, reorder, group and tune rules. A new kind of cost needs a metric: add an entry to
`metricRegistry()` in `src/rule_metrics.cpp`. Available metrics: `length`,
`clearance_violation {clearance}`, `clearance_deficit {clearance}`, `unknown_overlap`.

Mistakes in the file (unknown keys, metrics or rule names, bad aggregates or params, priority
cycles, priorities inside an equivalence class) stop startup with the file name and the entry at
fault.
Planner settings are read in `configure()`. The planner's `ParameterSource` is already scoped to
its name, so `getDouble("weight", 1.0)` reads `GridBased.<planner>.weight` from the YAML, or
`--set <planner>.weight=...` offline.

## Running in Nav2

`config/nav2_rpp_params.yaml` → `planner_server.GridBased` selects the planner and the shared
graph parameters. Build and start with the overlay:

```bash
docker compose -f compose.sim.gazebo.yaml -f compose.adapter.yaml up --build
```

Every plan appends a row to `results/plans.csv`.

## Running offline

No ROS needed; same pipeline, same CSV columns:

```bash
git submodule update --init   # the planning library
cmake -S planner_adapter -B build && cmake --build build
./build/compare_planners --map maps/map.yaml --start 0 0 --goal 3.5 1.2 \
    --set rulebook_file=config/rulebooks/example.yaml --csv results.csv --label office
```

Without `--planners a,b` every registered planner runs. If no yaml-cpp is installed, CMake
downloads yaml-cpp 0.8.0 (pinned by hash). On Windows with several MinGW installs on PATH, link
with `-DCMAKE_EXE_LINKER_FLAGS=-static` to avoid loading the wrong `libstdc++-6.dll`.

## Notes

- Only lethal costmap cells (254) count as obstacles and 255 as unknown. Nav2's inflation is
  ignored because `robot_radius + safety_margin` handles clearance.
- Start/goal snap to the nearest node within `max_snap_distance` without a line-of-sight check,
  so a pose right against a thin wall can snap to the far side.
- One rulebook per process: `RulebookCost` keeps it in a static variable. Several
  `planner_plugins` entries are fine as long as they share one `rulebook_file`.
- Clearance metrics compute the clearance map a second time (`buildGraph` keeps its copy
  internal); that time shows up in `cost_ms`.
- The planning library's partial-order search (`getOptimalPaths`) keeps every tied path, which
  explodes on grids, so the `rulebook` planner uses the total-order method only.
