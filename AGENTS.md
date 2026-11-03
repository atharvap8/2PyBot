# Project operating instructions

- Do not run automated test suites, test executables, or hardware/motion tests
  unless the user explicitly requests them. This is a standing user preference.
- Compilation, packaging, lint/syntax checks, APK signature checks and verification
  required by an explicitly requested operation (such as confirming an NVS save)
  are allowed. CI test execution is opt-in through `BOT_RUN_TESTS=true` only after
  explicit user authorization.
- Preserve user changes. Do not arm or drive the robot as part of a code/build task.
