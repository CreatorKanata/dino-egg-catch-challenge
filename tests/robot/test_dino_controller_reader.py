"""tests/robot/test_dino_controller_reader.py: Hardware-free checks of the dino-controller reader.

Fixtures are the example lines from docs/dino-controller-protocol.md, verbatim, so the
receiver rules (framing, validation, synchronization) are exercised without a serial port.
"""

from dataclasses import replace
import unittest

from robot.config import CONTROLLER_INPUT_TIMEOUT_S, CONTROLLER_LINE_MAX_BYTES
from robot.dino_controller_reader import (
    INITIAL_STATE,
    SerialControllerReader,
    apply,
    feed,
    parse_line,
)

READY = b'{"v":0,"type":"ready","seq":0,"ms":20,"device":"dino-controller","pins":{"up":26,"down":27,"left":21,"right":25},"encoder":{"clk":32,"dt":33,"sw":23,"edges_per_step":4,"rest_ab":3,"inverted":true}}'
STATE = b'{"v":0,"type":"state","seq":1,"ms":20,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}'
JOY_UP_RIGHT = b'{"v":0,"type":"joystick","seq":2,"ms":150,"up":true,"down":false,"left":false,"right":true}'
JOY_RELEASE = b'{"v":0,"type":"joystick","seq":3,"ms":390,"up":false,"down":false,"left":false,"right":false}'
ENC_CW = b'{"v":0,"type":"encoder","seq":4,"ms":470,"direction":"cw","delta":1}'
ENC_CCW = b'{"v":0,"type":"encoder","seq":5,"ms":520,"direction":"ccw","delta":-1}'
BTN_DOWN = b'{"v":0,"type":"button","seq":6,"ms":630,"pressed":true}'
BTN_UP = b'{"v":0,"type":"button","seq":7,"ms":710,"pressed":false}'
ERROR = b'{"v":0,"type":"error","seq":8,"ms":900,"code":"event_overflow"}'
STATE_AFTER_ERROR = b'{"v":0,"type":"state","seq":9,"ms":900,"device":"dino-controller","up":false,"down":false,"left":false,"right":false,"button":false,"ab":3}'
BOOT_TEXT = b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)"


def run(lines, state=INITIAL_STATE, now=1.0):
    """Apply parsed lines in order; return the final state and all encoder deltas."""
    deltas = []
    for line in lines:
        state, delta = apply(state, parse_line(line), now)
        deltas.append(delta)
    return state, deltas


def synced():
    return run([READY, STATE])[0]


class LineBufferTests(unittest.TestCase):
    def test_partial_chunks_are_joined(self):
        rest, lines = feed(b"", STATE[:40])
        self.assertEqual(lines, ())
        rest, lines = feed(rest, STATE[40:] + b"\n")
        self.assertEqual((rest, lines), (b"", (STATE,)))

    def test_multiple_lines_in_one_chunk_keep_order(self):
        rest, lines = feed(b"", JOY_UP_RIGHT + b"\n" + ENC_CW + b"\n" + BTN_DOWN[:10])
        self.assertEqual(lines, (JOY_UP_RIGHT, ENC_CW))
        self.assertEqual(rest, BTN_DOWN[:10])

    def test_trailing_cr_is_stripped(self):
        self.assertEqual(feed(b"", STATE + b"\r\n"), (b"", (STATE,)))

    def test_oversized_line_is_discarded_through_next_lf(self):
        junk = b"x" * (CONTROLLER_LINE_MAX_BYTES + 40)
        rest, lines = feed(b"", junk[:200])
        rest, lines = feed(rest, junk[200:])
        self.assertEqual(lines, ())
        self.assertLessEqual(len(rest), CONTROLLER_LINE_MAX_BYTES)
        rest, lines = feed(rest, b'"tail"}\n' + STATE + b"\n")
        self.assertEqual((rest, lines), (b"", (STATE,)))

    def test_line_at_the_frame_limit_is_kept(self):
        line = b"y" * (CONTROLLER_LINE_MAX_BYTES - 1)
        self.assertEqual(feed(b"", line + b"\n")[1], (line,))
        self.assertEqual(feed(b"", line + b"y\n")[1], ())

    def test_boot_text_is_ignored(self):
        _, lines = feed(b"", BOOT_TEXT + b"\r\n\xff\xfe\n" + STATE + b"\n")
        parsed = [parse_line(line) for line in lines]
        self.assertEqual(parsed[:2], [None, None])
        self.assertEqual(parsed[2].type, "state")


class ParseTests(unittest.TestCase):
    def test_every_documented_example_parses(self):
        expected = ["ready", "state", "joystick", "joystick", "encoder", "encoder",
                    "button", "button", "error", "state"]
        lines = [READY, STATE, JOY_UP_RIGHT, JOY_RELEASE, ENC_CW, ENC_CCW,
                 BTN_DOWN, BTN_UP, ERROR, STATE_AFTER_ERROR]
        self.assertEqual([parse_line(line).type for line in lines], expected)

    def test_payload_fields(self):
        self.assertEqual(parse_line(ENC_CW).delta, 1)
        self.assertEqual(parse_line(ENC_CCW).delta, -1)
        self.assertIs(parse_line(BTN_DOWN).button, True)
        self.assertEqual(parse_line(ERROR).code, "event_overflow")
        joy = parse_line(JOY_UP_RIGHT)
        self.assertEqual((joy.up, joy.down, joy.left, joy.right), (True, False, False, True))
        self.assertEqual((parse_line(STATE).seq, parse_line(STATE).ms), (1, 20))

    def test_unknown_extra_fields_are_ignored(self):
        line = BTN_DOWN[:-1] + b',"extra":{"a":1}}'
        self.assertIs(parse_line(line).button, True)

    def test_malformed_lines_are_rejected(self):
        cases = {
            "wrong version": BTN_DOWN.replace(b'"v":0', b'"v":1'),
            "bool version": BTN_DOWN.replace(b'"v":0', b'"v":false'),
            "delta mismatch": ENC_CW.replace(b'"delta":1', b'"delta":-1'),
            "delta two": ENC_CW.replace(b'"delta":1', b'"delta":2'),
            "unknown direction": ENC_CW.replace(b'"cw"', b'"left"'),
            "non-bool joystick": JOY_UP_RIGHT.replace(b'"up":true', b'"up":1'),
            "missing joystick field": JOY_UP_RIGHT.replace(b',"right":true', b""),
            "missing device": STATE.replace(b'"device":"dino-controller",', b""),
            "wrong device": STATE.replace(b"dino-controller", b"other"),
            "ab out of range": STATE.replace(b'"ab":3', b'"ab":4'),
            "string pressed": BTN_DOWN.replace(b"true", b'"true"'),
            "unknown type": BTN_DOWN.replace(b'"button"', b'"heartbeat"'),
            "negative seq": BTN_DOWN.replace(b'"seq":6', b'"seq":-1'),
            "seq too large": BTN_DOWN.replace(b'"seq":6', b'"seq":4294967296'),
            "float ms": BTN_DOWN.replace(b'"ms":630', b'"ms":630.5'),
            "non-string code": ERROR.replace(b'"event_overflow"', b"7"),
            "not an object": b"[1,2,3]",
            "truncated json": STATE[:-5],
            "empty": b"",
        }
        for name, line in cases.items():
            with self.subTest(name=name):
                self.assertIsNone(parse_line(line))


class ApplyTests(unittest.TestCase):
    def test_input_ignored_before_first_state(self):
        state, deltas = run([JOY_UP_RIGHT, ENC_CW, BTN_DOWN])
        self.assertFalse(state.synchronized)
        self.assertEqual((state.up, state.right, state.button), (False, False, False))
        self.assertEqual(deltas, [0, 0, 0])

    def test_state_synchronizes_and_replaces_levels(self):
        line = STATE.replace(b'"up":false', b'"up":true').replace(b'"button":false', b'"button":true')
        state, _ = run([line])
        self.assertTrue(state.synchronized)
        self.assertEqual((state.up, state.button), (True, True))
        self.assertEqual((state.last_seq, state.last_update_monotonic), (1, 1.0))

    def test_joystick_replaces_all_four_directions(self):
        state, _ = run([JOY_UP_RIGHT], synced())
        self.assertEqual((state.up, state.down, state.left, state.right), (True, False, False, True))
        state, _ = run([JOY_RELEASE], state)
        self.assertEqual((state.up, state.down, state.left, state.right), (False,) * 4)

    def test_button_is_independent_of_joystick(self):
        state, _ = run([JOY_UP_RIGHT, JOY_RELEASE, ENC_CW, ENC_CCW, BTN_DOWN], synced())
        self.assertTrue(state.button)
        self.assertFalse(state.up)
        pressed = run([BTN_DOWN.replace(b'"seq":6', b'"seq":2')], synced())[0]
        self.assertEqual(run([JOY_UP_RIGHT.replace(b'"seq":2', b'"seq":3')], pressed)[0].button, True)

    def test_encoder_delta_is_returned_once(self):
        state, deltas = run([JOY_UP_RIGHT, JOY_RELEASE, ENC_CW, ENC_CCW], synced())
        self.assertEqual(deltas, [0, 0, 1, -1])
        self.assertTrue(state.synchronized)

    def test_ready_starts_a_fresh_session(self):
        held = run([JOY_UP_RIGHT], synced())[0]
        state, delta = apply(held, parse_line(READY), 2.0)
        self.assertEqual(delta, 0)
        self.assertFalse(state.synchronized)
        self.assertFalse(state.up or state.right)
        self.assertEqual((state.last_seq, state.last_update_monotonic), (0, 2.0))

    def test_error_clears_and_requires_fresh_snapshot(self):
        held = run([JOY_UP_RIGHT, JOY_RELEASE, ENC_CW, ENC_CCW, BTN_DOWN, BTN_UP], synced())[0]
        held = replace(held, up=True, button=True)
        state, _ = run([ERROR], held)
        self.assertFalse(state.synchronized)
        self.assertFalse(state.up or state.button)
        state, _ = run([STATE_AFTER_ERROR], state)
        self.assertTrue(state.synchronized)

    def test_sequence_gap_desynchronizes(self):
        state, deltas = run([JOY_UP_RIGHT, ENC_CCW], synced())  # seq 2 then 5
        self.assertFalse(state.synchronized)
        self.assertFalse(state.up or state.right)
        self.assertEqual(deltas, [0, 0])

    def test_sequence_gap_on_state_still_synchronizes(self):
        state, _ = run([STATE_AFTER_ERROR.replace(b'"up":false', b'"up":true')], synced())
        self.assertTrue(state.synchronized)
        self.assertTrue(state.up)
        self.assertEqual(state.last_seq, 9)

    def test_sequence_wrap_is_not_a_gap(self):
        state = replace(synced(), last_seq=4294967295, up=True)
        wrapped = BTN_DOWN.replace(b'"seq":6', b'"seq":0')
        state, _ = run([wrapped], state)
        self.assertTrue(state.synchronized)
        self.assertEqual((state.up, state.button, state.last_seq), (True, True, 0))

    def test_inputs_are_not_mutated(self):
        before = synced()
        run([JOY_UP_RIGHT], before)
        self.assertEqual(before, synced())


class FakeSerial:
    """Minimal stand-in for serial.Serial: queued reads, recorded writes."""

    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.written = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        return self.chunks.pop(0) if self.chunks and size else b""

    def write(self, data):
        self.written.append(data)

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True


class SerialReaderTests(unittest.TestCase):
    def make(self, chunks):
        fake = FakeSerial(chunks)
        reader = SerialControllerReader("fake", serial_factory=lambda **_: fake)
        reader.open(now=0.0)
        return reader, fake

    def test_open_requests_state_and_poll_accumulates_deltas(self):
        stream = b"\n".join([BOOT_TEXT, READY, STATE, JOY_UP_RIGHT, JOY_RELEASE, ENC_CW]) + b"\n"
        reader, fake = self.make([stream, ENC_CCW + b"\n" + BTN_DOWN[:5]])
        self.assertEqual(fake.written, [b"STATE\n"])
        state, delta = reader.poll(0.05)
        self.assertEqual((state.synchronized, delta), (True, 1))
        state, delta = reader.poll(0.1)
        self.assertEqual((delta, state.last_seq), (-1, 5))
        self.assertFalse(reader.is_stale(0.1 + CONTROLLER_INPUT_TIMEOUT_S))
        self.assertTrue(reader.is_stale(0.11 + CONTROLLER_INPUT_TIMEOUT_S))

    def test_state_is_polled_periodically(self):
        reader, fake = self.make([])
        reader.poll(0.1)
        self.assertEqual(len(fake.written), 1)
        reader.poll(0.25)
        self.assertEqual(fake.written, [b"STATE\n", b"STATE\n"])

    def test_stale_before_any_message_and_close(self):
        reader, fake = self.make([])
        self.assertTrue(reader.is_stale(0.0))
        reader.close()
        self.assertTrue(fake.closed)
        self.assertEqual(reader.state, INITIAL_STATE)

    def test_poll_before_open_raises(self):
        with self.assertRaises(RuntimeError):
            SerialControllerReader("fake").poll(0.0)


if __name__ == "__main__":
    unittest.main()
