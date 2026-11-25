# Doctor: auto-diagnose and self-heal

Status: Phase 1 implemented on branch `feature/doctor`
(`console/doctor.py` pure rule engine + runtime, `tests/test_doctor.py`,
`GET /api/doctor` + `POST /api/doctor/heal`, a background heal task, and doctor
cases in `2pybot doctor`). Phases 2 to 4 (TUI page, broader auto-heal, more rules)
remain. Captures what the "doctor" should become after the deploy-loop incident
(a transient deploy deferral retried forever and made the robot glitch): when
something breaks, the robot works out the root cause on its own and either fixes
it or tells the operator exactly what to do.

## What exists today (reuse, do not rebuild)

- `2pybot doctor` / `/api/...`: a one-shot diagnostics report (telemetry, services,
  USB, Wi-Fi, logs).
- `sysmap_health.evaluate()`: already turns raw console state into per-block health
  with **root-cause suppression** (a child that is only visible through a failed
  parent is marked `blocked`, not `error`, so exactly one block lights up as the
  culprit). This is the heart of root-causing; the doctor should consume it.
- `deployment_status`, firmware job state, service states, the controller log.

The doctor is the layer that turns "something is red" into "this is the cause, and
here is the fix," and can apply the safe fixes itself.

## Shape

A `doctor` module (host side) with three parts:

1. **Signals**: a periodic collector (about 1 Hz, cheap) that gathers the inputs
   `sysmap_health.evaluate()` already uses, plus: deployment phase/defer_count,
   service active/failed states, systemd restart counts, controller reset-reason
   and fault counters from the log, dependency checks (for example onnxruntime
   importable), disk/temp.

2. **Rule engine**: an ordered list of rules. Each rule is
   `match(signals) -> Case | None` where a `Case` has:
   - `id`, `title`, human `cause` (one sentence),
   - `severity` (info / warn / critical),
   - `evidence` (the signals that fired it),
   - `recoveries`: ordered list, each with `label`, `safety` (`auto` | `confirm` |
     `manual`), and either a callable (service restart, config fix, dependency
     install) or instructions for `manual`.
   Rules are pure functions and unit-testable, like `sysmap_health`.

3. **Healer + UI**:
   - A background supervisor evaluates rules; when a `Case` appears it is recorded
     (a "case file" with timestamp, evidence, what was tried).
   - `safety=auto` recoveries run automatically if they are **idempotent and never
     cause motion** (restart a crashed service, reinstall a missing dependency,
     stop a runaway loop). Everything else waits for the operator.
   - The TUI gains a **Doctor page** that auto-opens (or flashes an indicator) when
     a case is active: shows the cause, the evidence, and buttons for each recovery.
     `2pybot doctor` prints the same and can `--fix` the auto-safe ones.

## Safety rules (hard limits)

- The doctor never arms, drives, or unlocks motion. Recoveries are limited to host
  services, deploy/firmware state, config and dependencies.
- Auto recoveries must be idempotent and bounded (a max attempts + cooldown, so a
  healer can never become its own loop — the exact failure mode we just hit).
- Destructive or controller-touching actions (reflash, NVS erase, re-pair) are
  always `confirm`, never `auto`.
- Every action is logged to the case file.

## Seed rules (from real incidents)

| Symptom | Root cause | Recovery (safety) |
| --- | --- | --- |
| Deploy stuck re-running, console restarting | one-pull deploy keeps deferring (controller/console not ready for the lock) | latch after N deferrals (done in deploy.py); doctor: "wake + SELECT + retry" (manual), or stop the deploy timer (confirm) |
| `2pybot-console` restart-looping | console crash on start | show last traceback; restart once (auto, capped); if it keeps failing, roll back release (confirm) |
| Serial stale, port open | ESP32 held in reset / flashing / boot-gate | check boot-gate + firmware job; wait, or reset gate (confirm) |
| `onnxruntime` import fails (visual re-pair off) | dependency missing after a venv rebuild | `pip install` into the venv (auto) |
| Camera `stopped`, not powered down | encoder died | restart stream (auto, capped) |
| One RS485 side dropping replies | cable/connector on that side | manual: inspect that cable |
| Powered down but operator confused | sleep mode on | "Wake up" (confirm) |
| Disk nearly full / high temp | housekeeping | vacuum journal / warn (auto for journal, warn for temp) |

## Phases

1. Package the signal collector + rule engine around the existing
   `sysmap_health.evaluate()`; expose `GET /api/doctor` (cases + recommended
   recoveries) and extend `2pybot doctor` to print cases and `--fix` auto-safe ones.
2. TUI Doctor page that auto-surfaces an active case with recovery buttons.
3. Background auto-healer for the `auto` recoveries, with a global attempt cap and
   cooldown and a full case-file audit trail.
4. Grow the rule table from real incidents; add a regression test per rule.
