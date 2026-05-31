# Changelog — EMHASS Amber Battery Management

All notable changes to `EMHASS battery control script.yaml`.

## [Unreleased] — 2026-05-31

### Added
- **Enable toggle (`input_boolean.emhass_battery_control`)** gating all control
  actions. When off, no commands are sent to the inverter.
- **Pending-plan display (`input_text.emhass_battery_status`)** that updates on
  every run, including the planned battery power in kW, and runs **even when
  control is disabled** (shows `— CONTROL DISABLED`).
- Single `planned_mode` variable so the display and control logic always agree.
- **Future-plan visualisation** derived from the EMHASS forecast attributes:
  - Template sensor `sensor.emhass_next_mode_change` (file
    `template/emhass_future_plan_template.yaml`) exposing next mode, `starts_at`,
    `duration_min`, `current_mode`.
  - Markdown dashboard card (`emhass_future_plan_card.yaml`) rendering the
    upcoming schedule as collapsed mode blocks with peak kW, signed grid `Total $`,
    and the energy-weighted effective unit price embedded in the mode label
    (`charge - 0.089`).
  - Reads `sensor.mpc_batt_power` attr `battery_scheduled_power`,
    `sensor.mpc_grid_power` attr `forecasts`, `sensor.mpc_general_price` attr
    `unit_load_cost_forecasts`, `sensor.mpc_feed_in_price` attr
    `unit_prod_price_forecasts` — all 5-minute, index-aligned.

### Changed
- **Migrated control from Sigen plant entities to the Amber Bridge** override
  controls (`requested_mode` + `requested_duration` + `apply_override` /
  `cancel_override` buttons).
- **Logic gated on grid flow:** override only on arbitrage (`p_grid != 0`);
  otherwise leave the inverter on its default load-following.
- **Override duration set to 10 min** as a safety fallback (EMHASS re-optimises
  every ~5 min on Amber price updates, so this outlasts the re-trigger interval
  while limiting stale-command exposure during price volatility).
- **Cancel-override is now guarded** by `active_override_value` so it only fires
  when a real override is active (no redundant presses).

### Removed
- All Sigen plant entities
  (`select.sigen_plant_remote_ems_control_mode`,
  `number.sigen_plant_ess_max_charging_limit`,
  `number.sigen_plant_ess_max_discharging_limit`).
- Charge/discharge **power-limit** commands — import/export now run at the
  inverter's maximum, managed automatically by the inverter.
- The redundant self-consume mode command (default inverter behaviour already
  load-follows; duration-based auto-revert returns to it).
- Dead variables (`max_charge_rate`, `max_discharge_rate`, `solix_*` sensors).
