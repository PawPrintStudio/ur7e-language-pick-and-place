"""Offline tests: actual sockets, fragmented frames, faults, and no retry."""
import socket
import struct
import threading
import unittest

from urcap_handoff_protocol import Settings, Wire, run_session, validate_result


class Hooks:
    def __init__(self, fail_handback=False):
        self.events = []
        self.fail_handback = fail_handback

    def check(self):
        pass

    def wait_ready(self):
        self.events.append("ready")

    def move(self, phase):
        self.events.append(f"move{phase}")

    def hand_back(self):
        self.events.append("handback")
        if self.fail_handback:
            raise RuntimeError("handback rejected")

    def wait_left(self):
        self.events.append("left")

    def report(self, event):
        self.events.append(f"complete{event['phase']}")


class ProtocolTest(unittest.TestCase):
    def cycle(self, alter=None, fail_handback=False, hello=None):
        settings = Settings()
        local, remote = socket.socketpair()
        self.grants = []
        errors = []

        def pendant():
            try:
                with remote:
                    wire = Wire(remote, lambda: None, timeout=0.5)
                    wire.send(*(hello or (*settings.hello, 700)))
                    nonce, = wire.receive(1)
                    for phase, target in ((1, 590), (2, 700)):
                        # Exercise TCP fragmentation rather than assuming a
                        # single recv matches one pendant message.
                        for byte in struct.pack("!ii", nonce, -phase):
                            remote.sendall(bytes([byte]))
                        grant = wire.receive(2)
                        self.grants.append(grant)
                        result = (nonce, phase, target, 0, 0)
                        wire.send(*(alter(result) if alter else result))
                        self.assertEqual(wire.receive(2), (nonce, phase))
            except (ConnectionError, TimeoutError, OSError):
                pass  # Expected when the coordinator aborts without an ACK.
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=pendant)
        thread.start()
        self.hooks = Hooks(fail_handback)
        try:
            with local:
                result = run_session(local, settings, self.hooks, timeout=0.5)
            return result
        finally:
            thread.join(2)
            self.assertFalse(thread.is_alive())
            if errors:
                raise errors[0]

    def test_complete_two_phase_exchange(self):
        result = self.cycle()
        self.assertEqual([x["width_mm"] for x in result], [59, 70])
        self.assertEqual(self.hooks.events, [
            "ready", "move1", "ready", "handback", "left", "ready", "complete1",
            "move2", "ready", "handback", "left", "ready", "complete2",
        ])

    def test_wrong_settings_never_hands_back(self):
        with self.assertRaisesRegex(RuntimeError, "settings"):
            self.cycle(hello=(*Settings(force_n=6).hello, 700))
        self.assertEqual(self.hooks.events, [])
        self.assertEqual(self.grants, [])

    def test_initial_width_mismatch_never_hands_back(self):
        with self.assertRaisesRegex(RuntimeError, "initial width"):
            self.cycle(hello=(*Settings().hello, 584))
        self.assertEqual(self.grants, [])

    def test_service_failure_never_grants_grip_or_retries(self):
        with self.assertRaisesRegex(RuntimeError, "handback rejected"):
            self.cycle(fail_handback=True)
        self.assertEqual(self.grants, [])
        self.assertEqual(self.hooks.events.count("handback"), 1)

    def test_stale_nonce_does_not_advance_to_second_phase(self):
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            self.cycle(alter=lambda r: (r[0] - 1, *r[1:]))
        self.assertEqual(len(self.grants), 1)
        self.assertNotIn("move2", self.hooks.events)

    def test_wrong_phase_does_not_advance(self):
        with self.assertRaisesRegex(RuntimeError, "out-of-order"):
            self.cycle(alter=lambda r: (r[0], 2, *r[2:]))
        self.assertEqual(len(self.grants), 1)

    def test_busy_completion_does_not_advance(self):
        with self.assertRaisesRegex(RuntimeError, "busy"):
            self.cycle(alter=lambda r: (*r[:3], 1, r[4]))
        self.assertEqual(len(self.grants), 1)

    def test_off_target_completion_does_not_advance(self):
        with self.assertRaisesRegex(RuntimeError, "target"):
            self.cycle(alter=lambda r: (*r[:2], 400, *r[3:]))
        self.assertEqual(len(self.grants), 1)

    def test_read_timeout_checks_health_and_fails(self):
        checks = []
        local, peer = socket.socketpair()
        with local, peer:
            with self.assertRaises(TimeoutError):
                Wire(local, lambda: checks.append(1), timeout=0.05).receive(2)
        self.assertGreater(len(checks), 1)

    def test_disconnect_is_not_completion(self):
        local, peer = socket.socketpair()
        peer.close()
        with local, self.assertRaises(ConnectionError):
            Wire(local, lambda: None).receive(2)

    def test_health_failure_prevents_write(self):
        def failure():
            raise RuntimeError("safety stop")
        local, peer = socket.socketpair()
        with local, peer:
            with self.assertRaisesRegex(RuntimeError, "safety stop"):
                Wire(local, failure).send(123, 1)
            peer.settimeout(0.02)
            with self.assertRaises(socket.timeout):
                peer.recv(1)

    def test_grasp_detection_required_when_configured(self):
        settings = Settings(close_mm=30, open_mm=45, expect_object=True)
        self.assertEqual(validate_result((7, 1, 330, 0, 1), 7, 1, settings), 33)
        with self.assertRaisesRegex(RuntimeError, "not detected"):
            validate_result((7, 1, 300, 0, 0), 7, 1, settings)
        with self.assertRaisesRegex(RuntimeError, "object still detected"):
            validate_result((7, 2, 450, 0, 1), 7, 2, settings)

    def test_nonfinite_or_excessive_settings_rejected(self):
        for kwargs in ({"force_n": 40}, {"close_mm": float("nan")},
                       {"close_mm": 10}, {"open_mm": 111}, {"force_n": 5.05}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Settings(**kwargs)


if __name__ == "__main__":
    unittest.main()
