# Manual Scheduled Battery Override

A manual scheduling layer that sits **above EMHASS**. You schedule "force mode X
for N minutes at time T, priority P" jobs; when a job is active it drives the
Amber Bridge and **suppresses the EMHASS executor** for its window. When no job
is active, EMHASS resumes automatically.

Files:
- `packages/manual_override.yaml` — helpers, scripts, automation, display sensor
- `manual_override_dashboard.yaml` — Lovelace view (Mushroom + native cards)
- `EMHASS battery control script.yaml` — gained a priority guard + MANUAL status

## One-time setup

1. **Create the calendar** (cannot be done in YAML):
   Settings → Devices & Services → **Add Integration → Local Calendar** →
   name it exactly **`Battery Overrides`** (entity_id `calendar.battery_overrides`).
2. **Enable packages** — `configuration.yaml` now has
   `packages: !include_dir_named packages` under `homeassistant:`. Restart HA.
3. **HACS frontend cards** for the dashboard: **Mushroom** (required),
   **card-mod** (optional). Add the view from `manual_override_dashboard.yaml`
   via the dashboard raw config editor.
3b. **HACS "Variable" integration** (rogro82) — required for persistent history.
   Install via HACS, then restart. Provides `variable.manual_override_history`
   (the package defines it; `restore: true` keeps records across restarts).
4. Turn on **`input_boolean.manual_overrides_enabled`** (master switch).

## How a job is stored

Each job is one event in `calendar.battery_overrides`:

| Job field | Calendar field | Example |
|---|---|---|
| Start | event start | `2026-06-03 14:00` |
| Duration | `end − start` | end `15:30` ⇒ 90 min |
| Mode | `description` line | `mode: charge` |
| Priority | `description` line | `priority: 80` |
| Recurrence | `rrule` | `FREQ=DAILY` (empty = one-off) |
| Label | `summary` | `Override: charge` (`[OFF]` prefix = paused) |

## Administration

- **Create** — the dashboard "Schedule a new override" form (mode/start/
  duration/priority/repeat) → **Add to schedule**. Runs
  `script.create_override_job`, which writes the calendar event for you.
- **Edit / Delete** — click the event in the calendar card → native HA dialog
  (retime, resize, change description, change/stop recurrence; delete one
  instance or the whole series).
- **Pause without deleting** — prefix the event summary with `[OFF]` (edit in
  the calendar dialog). The coordinator skips `[OFF]` events.
- **Disable the whole layer** — turn off `input_boolean.manual_overrides_enabled`;
  any active hold is released and EMHASS runs unimpeded.

## Priority semantics

- EMHASS has an implicit priority of **0**. Any active manual job (priority ≥ 1)
  beats it.
- Overlapping manual jobs: **highest priority wins**; tie → latest start.
- Resolution is automatic at fire time — you can schedule overlapping jobs
  freely; you don't have to deconflict the calendar by hand.

Full precedence each tick (top wins):

1. **Stop-guards (veto)** — apply to a `charge` winner only: import price ≥
   ceiling, OR SoC ≥ target (one-shot latched), OR SoC unknown while the target
   guard is on (fail-closed). A veto cancels the override → load-following; it
   never forces a discharge.
2. **Manual scheduled jobs** (priority ≥ 1) — winner logic above.
3. **Floor-price auto-charge** — a standing rule, not a calendar event; injects a
   `charge` winner at `input_number.floor_charge_priority` while live import
   price ≤ floor. Competes with scheduled jobs by the same integer.
4. **EMHASS** (priority 0).

## Charge guards & floor-price auto-charge

All three are **off by default and inert** — existing scheduled jobs are
unaffected until you enable a guard.

| Feature | Enable toggle | Setting | Behaviour |
|---|---|---|---|
| Target charge level | `input_boolean.charge_target_enabled` | `input_number.charge_target_soc` (%) | grid-charge stops when SoC ≥ target. **One-shot** — does not resume if SoC drifts below; resets on the next winner. |
| Ceiling import price | `input_boolean.import_ceiling_enabled` | `input_number.import_ceiling_price` ($/kWh) | grid-charge stops while import price ≥ ceiling; resumes below ceiling − deadband. |
| Floor import price | `input_boolean.floor_charge_enabled` | `input_number.import_floor_price` ($/kWh), `input_number.floor_charge_priority` | auto grid-charge while import price ≤ floor; stops above floor + deadband. |

- **Deadband** (`input_number.price_deadband`, $/kWh) adds hysteresis on the
  ceiling/floor thresholds so prices hovering at the boundary don't flap the
  bridge minute-by-minute.
- **Per-job override** — a charge job may carry `target_soc:` and/or
  `ceiling_price:` lines in its event description; these override the global
  values for that job only. **Target SoC** has a field in the "Schedule a new
  override" form (`0` = use global / no per-job target); `ceiling_price:` is
  added by editing the calendar event description. Per-job ceilings use a strict
  compare (no deadband).
- Guards apply to **charge only** — discharge/preserve/auto jobs are never vetoed.

## Control flow

`automation.manual_override_coordinator_tick` calls
`script.evaluate_manual_override` every minute, on HA start, and whenever the
calendar or master switch changes. The coordinator:

1. If the master switch is off → release any hold, stop.
2. `calendar.get_events` for the current minute → pick the winning job.
3. Winner exists → record it in the `active_override_*` helpers (this is what
   suppresses EMHASS) and, when the winner changed or the bridge has reverted,
   (re)apply mode + remaining duration (capped 200 min) to the Amber Bridge.
4. No winner but a hold is active → release: clear helpers, guarded
   `cancel_override`, EMHASS resumes on its next plan update.

Re-applying only on change (or when the bridge shows idle) means >200-min jobs
and HA restarts mid-window are reconciled automatically without spamming the
bridge.

## Helpers created

| Helper | Role |
|---|---|
| `input_boolean.manual_overrides_enabled` | master on/off |
| `input_select.override_form_mode` / `_repeat` | add-job form |
| `input_number.override_form_duration` / `_priority` | add-job form |
| `input_datetime.override_form_start` | add-job form |
| `input_number.active_override_priority` | winning priority (**0 = none**); EMHASS guard reads this |
| `input_select.active_override_mode` | current forced mode |
| `input_text.active_override_owner` | current winner id |
| `input_datetime.active_override_until` | current hold expiry |
| `variable.manual_override_history` | persistent history (last 20; `records` attr; restores across restart) |
| `sensor.battery_override_schedule` | upcoming jobs (display only) |
| `input_boolean.charge_target_enabled` / `_soc` | target-SoC stop-guard toggle + level |
| `input_boolean.import_ceiling_enabled` / `import_ceiling_price` | ceiling-price stop-guard |
| `input_boolean.floor_charge_enabled` / `import_floor_price` / `floor_charge_priority` | floor-price auto-charge |
| `input_number.price_deadband` | hysteresis band for ceiling/floor |
| `input_number.battery_usable_capacity_kwh` / `assumed_charge_power_kw` | battery model for the form's SoC-gain estimate (set once) |
| `input_boolean.charge_target_latched` / `charge_veto_active` / `floor_charge_active` / `ceiling_block_active` | internal latches (coordinator-written) |

## Limitations

- `local_calendar` RRULE supports FREQ DAILY/WEEKLY/MONTHLY/YEARLY only.
- Pause is a `[OFF]` summary convention, not a native per-row toggle.
- History keeps the last 20 records as structured dicts in
  `variable.manual_override_history` (`records` attribute), rendered as a markdown
  table (Time/Mode/Dur/Prio/SoC/$-kWh/Result). Persists across HA restarts via the
  HACS "Variable" integration (`restore: true`). Requires that integration.
- Sub-minute timing precision is not guaranteed (1-minute coordinator tick).
