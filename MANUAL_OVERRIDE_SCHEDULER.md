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
| `input_text.manual_override_log` | history ring buffer (last 6) |
| `sensor.battery_override_schedule` | upcoming jobs (display only) |

## Limitations

- `local_calendar` RRULE supports FREQ DAILY/WEEKLY/MONTHLY/YEARLY only.
- Pause is a `[OFF]` summary convention, not a native per-row toggle.
- History keeps the last 6 records (input_text 255-char limit).
- Sub-minute timing precision is not guaranteed (1-minute coordinator tick).
