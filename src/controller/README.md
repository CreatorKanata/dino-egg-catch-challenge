<!-- src/controller/README.md: Reserve attendee input code separately from robot-side actuation. -->
# Attendee Controller

Place attendee input handling and controller interfaces here, including directional input, catch requests, and mode selection when implemented. Keep device-specific input handling separate from the command interface used by robot control.

Controller hardware, runtime, transport, and command format are not selected yet. Define and test the shared command contract with the [robot implementation](../robot/README.md) before adding a live connection. See the [concept](../../docs/concept.md) for the planned experience.
