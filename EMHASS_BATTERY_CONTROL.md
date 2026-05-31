# EMHASS Amber Battery Management

Home Assistant script that executes the EMHASS energy plan by driving the Amber
Bridge battery override controls. It is the **executor** for the plan that EMHASS
(the optimiser) produces.

File: `EMHASS battery control script.yaml`

---

## How it relates to the EMHASS plan

EMHASS solves a linear program over a rolling horizon (30-minute granularity) and
publishes the **current timestep's** decision into two sensors:

| EMHASS plan column | Home Assistant sensor | Meaning |
|---|---|---|
| `P_batt` (W) | `sensor.mpc_batt_power` | + = discharge, − = charge |
| `P_grid` (W) | `sensor.mpc_grid_power` | + = import, − = export |

The script triggers whenever either sensor changes (i.e. each time EMHASS
re-optimises), and translates the plan into an Amber Bridge override.

EMHASS re-optimises on every **Amber price update** (every ~5 min in the first
hour, then every 30 min). The override duration is therefore a *safety fallback*,
not the control cadence — see below.

---

## Control logic

Decisions are gated on **grid flow**, so the inverter is only overridden for
arbitrage. When there is no grid flow the inverter is left on its default
load-following (self-consumption) behaviour.

| Condition | `planned_mode` | Action |
|---|---|---|
| `p_grid != 0` and `p_batt < 0` | `charge` | Override → charge for 10 min |
| `p_grid != 0` and `p_batt > 0` | `discharge` | Override → discharge for 10 min |
| otherwise (`p_grid == 0` or `p_batt == 0`) | `load-following` | Cancel any active override |

- **No power limits are set.** Import/export run at the inverter's maximum, which
  the inverter manages automatically.
- The **cancel** is guarded: it only fires when
  `sensor.amber_bridge_..._active_override_value` shows a real override
  (not `none`/`auto`/`selfconsume`/`unknown`/`unavailable`/empty), avoiding
  redundant cancel presses.

### Override duration (10 min)

EMHASS re-issues a fresh override on each ~5-min run, so 10 min comfortably
outlasts the re-trigger interval during normal operation. If price updates stall
during a wide swing, the command auto-reverts to load-following within 10 min,
limiting exposure to a stale decision.

---

## Features

### 1. Enable toggle (gate)

All control actions are wrapped behind `input_boolean.emhass_battery_control`.
When **off**, the script still computes the plan and updates the dashboard, but
issues **no** commands to the inverter.

### 2. Pending-plan display (always runs)

Before the gate, the script writes the pending plan to
`input_text.emhass_battery_status`, so the planned mode is visible **even when
control is disabled**. Examples:

- `Pending: charge 14.76 kW for 10 min`
- `Pending: discharge 12.79 kW for 10 min`
- `Pending: load-following`
- `... — CONTROL DISABLED` appended when the gate is off.

The kW figure is the **planned** battery power from EMHASS (informational); the
actual override runs at the inverter's max.

---

## Required helpers

Create these in **Settings → Devices & Services → Helpers**:

| Helper type | Entity ID |
|---|---|
| Toggle | `input_boolean.emhass_battery_control` |
| Text | `input_text.emhass_battery_status` |

### Dashboard card example

```yaml
type: markdown
content: "🔋 {{ states('input_text.emhass_battery_status') }}"
```

Or an Entities card listing both helpers, so the enable toggle sits beside the
status text.

---

## Entities used

| Entity | Role |
|---|---|
| `sensor.mpc_batt_power` / `sensor.mpc_grid_power` | EMHASS plan inputs (triggers) |
| `select.amber_bridge_ashqb71f39300081_requested_mode` | Mode: charge / discharge / preserve / selfconsume / auto |
| `number.amber_bridge_ashqb71f39300081_requested_duration` | Override duration (min) |
| `button.amber_bridge_ashqb71f39300081_apply_override` | Apply mode change |
| `button.amber_bridge_ashqb71f39300081_cancel_override` | Cancel → back to default |
| `sensor.amber_bridge_ashqb71f39300081_active_override_value` | Current active override (delayed readback) |
| `input_boolean.emhass_battery_control` | Enable gate (feature 1) |
| `input_text.emhass_battery_status` | Pending-plan display (feature 2) |

---

## Future-plan visualisation

Two companion artefacts surface the *upcoming* plan (the EMHASS forecast, not just
the current timestep):

- **`template/emhass_future_plan_template.yaml`** → `sensor.emhass_next_mode_change`
  with attributes `current_mode`, `starts_at`, `duration_min`. Useful for tiles,
  badges, and pre-change notifications.
- **`emhass_future_plan_card.yaml`** → a Markdown dashboard card rendering the
  schedule as collapsed mode blocks: time, `mode - <unit price>`, duration,
  peak kW, and a signed grid `Total $`.

Both derive mode from the same grid-flow logic as the control script and read
these forecast attributes (all 5-minute, index-aligned, string values):

| Entity | Attribute | Value key | Meaning |
|---|---|---|---|
| `sensor.mpc_batt_power` | `battery_scheduled_power` | `mpc_batt_power` | planned battery power |
| `sensor.mpc_grid_power` | `forecasts` | `mpc_grid_power` | planned grid flow |
| `sensor.mpc_general_price` | `unit_load_cost_forecasts` | — | import price ($/kWh) |
| `sensor.mpc_feed_in_price` | `unit_prod_price_forecasts` | — | export price ($/kWh) |

Per block: `Total $ = Σ −(grid_kW × 5/60 h × price)` (import price when importing,
feed-in price when exporting; negative = cost, positive = revenue). The unit price
shown in the mode label is the **energy-weighted** effective rate
`Σ(|grid| × price) ÷ Σ|grid|`.

Loading: requires `template: !include_dir_merge_list template/` in
`configuration.yaml`, with the template file placed under `template/`.

## Notes / limitations

- **Resolution blind spot:** EMHASS works at 30-min resolution and collapses each
  block to a single price. Amber's real 5-min prices can swing widely within a
  block, so sub-30-min arbitrage is invisible to the optimiser. Finer granularity
  would require an EMHASS config change, not a script change.
- The status text always agrees with the control logic because both derive from a
  single `planned_mode` variable.
