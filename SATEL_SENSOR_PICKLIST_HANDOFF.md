# Handoff — make alarm sensor selection a pick list

Self-contained brief. You should not need the prior conversation.

**Status:** design agreed, nothing built. One thing must be verified on the
live instance before any code is worth writing (see "Gate" below).

---

## 1. Context

A Satel INTEGRA panel is integrated into Home Assistant via `satel_integra`
(ETHM-1 Plus, config-flow subentries: partition, zones, outputs). The
integration does **not** report which zone caused an alarm, so HA reconstructs
attribution itself from zone state changes gated on the partition state.

That was built and works. This task extends it.

There is a fuller system reference document kept **locally** by the user
(household manual folder, `satel-dloadx-change-plan.md`). Ask them for the
path if you need it — PART 5 and PART 6 are the relevant sections. **It must
never be committed.**

## 2. Hard constraints

- **The git remote is PUBLIC.** The user's standing instruction is "need to
  keep sensitive info local". Never commit the reference doc. Never write the
  panel IP, port, or zone/output layout into any repo file. Scan diffs for
  credentials and IPv4 literals before staging. Stage named files — never
  `git add .`, the tree has ~60 unrelated untracked files.
- **Dashboards are in storage mode.** There is no `lovelace:` block in
  `configuration.yaml`, so `satel alarm.yaml` in the repo is a **copy**.
  Editing it changes nothing on screen. The user must paste it into the card
  code editor. Tell them explicitly each time — several diagnostic rounds were
  wasted on this before it was understood.
- **Do not change `unique_id` on the template sensors.** The trigger history
  lives in a restored attribute; a new `unique_id` is a new entity and the
  history is gone.

## 3. Current state

### `template/alarm_trigger_template.yaml`
- `sensor.alarm_last_trigger` — trigger-based. Fires on any of six hardcoded
  Satel zone entities going `off` to `on` while the panel is in an armed-ish
  state, plus a fallback trigger on `alarm_control_panel.house` reaching
  `triggered` for violations HA never saw. Attributes: `entity_id`, `at`,
  `alarm_state`, and `history` (rolling list, last 25).
- `sensor.alarm_active_zones` — plain template, counts violated zones now.

### `satel alarm.yaml`
`vertical-stack` (deliberately not `custom:layout-card` masonry — it sizes
children by `getCardSize()`, and the grid over-reported, leaving a dead
vertical band). Contains a header card, an 8-tile `grid`, and a `markdown`
history table.

### The thing being replaced
Six copies in the card and two in the template file of:

```jinja
{%- for e in integration_entities('satel_integra') %}
{%- if e.startswith('binary_sensor.')
       and state_attr(e,'device_class') in ['motion','occupancy']
       and states(e) == 'on' %}
```

This works, but can only ever see entities the Satel panel owns, and the
include/exclude rule is Jinja buried in eight places.

## 4. The design

Create a **binary sensor group helper** in the UI:

> Settings → Devices & Services → Helpers → Create helper → Group →
> Binary sensor group. Name **Alarm Zones** (`binary_sensor.alarm_zones`).
> Tick the members.

That multi-select dialog is the entire selection interface. Everything
downstream reads the group:

```jinja
{%- set ns = namespace(v=[]) %}
{%- for e in expand('binary_sensor.alarm_zones') %}
{%- if e.state == 'on' %}
{%- set ns.v = ns.v + [e.name] %}
{%- endif %}
{%- endfor %}
```

Shorter than the current loop — the device_class filter disappears, because
membership **is** the filter. Works for non-Satel sensors (Zigbee contacts,
camera person-detect, leak probes). No restart or template reload to change
the selection.

Rejected alternatives, do not re-litigate without new information:

- **Labels** (`label_entities()`): applied one entity at a time from each
  entity's settings page, and a label is not an entity you can put on a card.
- **`groups.yaml`** (already included at `configuration.yaml:27`, empty):
  repo-tracked, which is tempting, but editing it needs an editor and a
  reload. The tick box is the point. Accept that the definition lives in
  `.storage` and is not in the repo, and note that in the reference doc.

## 5. Gate — verify this FIRST

The attribution sensor is trigger-based, and HA triggers are static (read once
at load). A group does not make them dynamic by itself.

HA is *supposed* to expand a group in a state trigger and fire per member,
with `trigger.entity_id` being the member. **Prove it before rewriting
anything.** Throwaway automation, state trigger on `binary_sensor.alarm_zones`,
action notifies `{{ trigger.entity_id }}`, two sensors ticked into the group,
walk past one.

**Route A — it printed the member id.** Trigger block becomes:

```yaml
triggers:
  - trigger: state
    entity_id: binary_sensor.alarm_zones
    from: "off"
    to: "on"
    id: zone
```

**Route B — it printed `binary_sensor.alarm_zones`.** Event trigger filtered
by membership. Guaranteed to work; costs a condition evaluation on every state
change in the instance (thousands/hour here — the Anker and Amber sensors are
chatty). Acceptable, but prefer A.

```yaml
triggers:
  - trigger: event
    event_type: state_changed
    id: zone
conditions:
  - condition: template
    value_template: >
      {{ trigger.event.data.entity_id in
         expand('binary_sensor.alarm_zones') | map(attribute='entity_id') | list
         and trigger.event.data.old_state.state == 'off'
         and trigger.event.data.new_state.state == 'on' }}
```

Either way, keep the existing `alarm_control_panel.house` fallback trigger and
its 30-second de-dupe condition.

## 6. Steps

1. User creates the group, ticks the six Satel binary sensors currently
   matched by the device_class filter (four PIRs, panic, sw panic).
   Verify `binary_sensor.alarm_zones` mirrors them.
2. Run the gate. Pick route A or B.
3. Rewrite the two templates in `template/alarm_trigger_template.yaml` to use
   `expand()`. `unique_id` unchanged.
4. Rewrite the six loops in `satel alarm.yaml`. Hand the file to the user to
   paste into the card editor.
5. Add a line to the reference doc (PART 3, Files) recording that the group
   lives in `.storage` and is not in the repo.
6. Commit named files only, after a secret scan.

## 7. Deliberately out of scope

**Tier 2 — sensors that should actually sound the alarm.** Group membership
means *observed*: logged, on the cards, named in the history table. It makes
the panel do nothing, and that is the correct default — a leak probe or a
camera should not be able to set the sirens off on day one. When something
genuinely needs to trigger, add a second group as a subset
(`binary_sensor.alarm_zones_armed_trigger`) plus one automation: member on +
panel armed, then the panel action. Do not build it speculatively.

## 8. Gotchas that have already cost time

1. Satel zone entity ids carry a **`_motion` suffix**
   (`binary_sensor.kitchen_motion`). A separate, non-Satel
   `binary_sensor.kitchen` also exists and silently absorbed a wrong id once —
   nothing errored and grep looked clean. Never write zone ids from memory.
2. Markdown card content must be a **literal** scalar (`|-`), not folded
   (`>`) — folded joins lines with spaces and collapses the table onto one
   line. Use `{%- -%}` so Jinja lines do not render as blank rows.
3. `| int(0)` on a sensor that does not exist yet silently yields `0`. This
   made a tile read "All clear" while zones were violated.
4. The panel-side arm state mapping is confirmed: night arm reports
   `armed_home`. The armed gate list deliberately **excludes** `arming`, so
   walking out during the exit delay is not logged.

## 9. Unrelated open question — do not let it block this, do not lose it

A study zone violation was recorded while the panel was `armed_away`, and the
user has not confirmed the sirens sounded. Study is an instant zone; they
should have. If they did not, something is wrong panel-side (zone bypassed,
not assigned to the partition, or outputs not scoped to it). This is a live
gap in the alarm itself, not the dashboard. Raise it; do not investigate
without being asked.
