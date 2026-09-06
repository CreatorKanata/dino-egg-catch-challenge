<!-- src/robot/README.md: Reserve robot-side implementation boundaries before choosing the runtime. -->
# Robot Control

Place SO-ARM101 arm control, LeKiwi base integration, and operating-mode coordination here. Keep hardware adapters separate from mode logic so behavior can be checked without moving a robot.

Drive Mode, Puppet Mode, and Autonomous Mode are planned in the [concept](../../docs/concept.md); they are not implemented by this scaffold. Emergency stopping, motion limits, and communication-loss handling belong in local control rather than depending on a cloud service.

Choose the runtime and libraries when implementation begins. For Python code, centralize runtime tunables in `config.py`. Establish measured hardware limits and meaningful tests with the first control implementation.
