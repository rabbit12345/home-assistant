# Changelog

Overall change log for the Home Assistant configuration repository.
Dates are in `YYYY-MM-DD`. Newest first.

---

## 2026-05-31 — EMHASS future-plan visualisation

- Added template sensor `sensor.emhass_next_mode_change`
  (`template/emhass_future_plan_template.yaml`) exposing the next planned mode,
  start time, and duration, derived from the EMHASS forecast attributes.
- Added Markdown dashboard card (`emhass_future_plan_card.yaml`) showing the
  upcoming schedule as mode blocks with peak kW, signed grid `Total $`, and the
  energy-weighted effective unit price embedded in the mode label.
- Wired `template: !include_dir_merge_list template/` into `configuration.yaml`.

## 2026-05-31 — EMHASS Amber battery control

- Added **EMHASS Amber Battery Management** script (`EMHASS battery control script.yaml`)
  that executes the EMHASS energy plan via the Amber Bridge override controls.
  - Control gated on grid flow (arbitrage-only overrides); inverter left on its
    default load-following otherwise.
  - 10-minute override duration as a safety fallback (EMHASS re-optimises on each
    ~5-min Amber price update).
  - Guarded cancel-override (only fires when a real override is active).
  - Enable toggle (`input_boolean.emhass_battery_control`) gating control actions.
  - Pending-plan dashboard display (`input_text.emhass_battery_status`) including
    planned power in kW, shown even when control is disabled.
  - Migrated control from Sigen plant entities to the Amber Bridge; removed
    power-limit commands (inverter runs at max).
- Added documentation `EMHASS_BATTERY_CONTROL.md` and per-script changelog
  `EMHASS_BATTERY_CONTROL_CHANGELOG.md`.
- Renamed script alias to "Amber - Anker Battery Management".

## 2026-04-26 — Configuration & keypad

- Updated Home Assistant configuration and keypad settings.

## 2026-04-19 — Mushroom battery panel

- Added mushroom battery panel cards for sensors and blinds
  (`blind_battery_mushroom.yaml`, `sensor_battery_mushroom.yaml`).

## 2026-04-17 — Motion light blueprint

- Motion light blueprint: cross-midnight end time, sunset/sunrise window, and a
  run-during-day toggle (`motion_light_blueprint.yaml`).

## 2024-08-11 — Initial keypad firmware & config

- Corrected max voltage value to 4930; added timer + insider button; changed
  delay to 800 ms.
- Updated keypad code from cjd; LED proto state additions.
- Initial commit of original keypad YAML and firmware.
