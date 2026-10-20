"""USB serial link to the Uno, without ROS: reconnects, splits lines, parses the sketch's replies, and
says what is wrong (E-stop open, wrong sketch, nothing arriving). Free of rclpy so it can be tested
with a fake serial port.
"""
import time

from . import protocol as P


class Link:
    """All methods are safe to call while the Uno is unplugged."""

    RESET_WAIT_S = 2.0      # the Uno resets when the port opens
    SILENT_DROP_S = 3.0     # nothing at all for this long -> close and reopen

    def __init__(self, port, baud, on_open=None, log=None, opener=None, clock=None, sleep=None):
        self.port, self.baud = port, baud
        self.on_open = on_open or (lambda: None)
        self.log = log or (lambda text, warn=False: None)
        self.opener = opener
        self.clock = clock or (lambda: time.time())
        self.sleep = sleep or (lambda s: time.sleep(s))
        self.ser = None
        self.buf = b""
        self.next_try = 0.0
        self.opened_at = 0.0
        self.last_rx = 0.0          # any line
        self.last_known = 0.0       # a line from this sketch's protocol
        self.counts = {"ok": 0, "ok_stop": 0, "ready": 0, "enc": 0, "err": 0, "unknown": 0}
        self.errors = {}            # ERR code -> count
        self.estop_open = None      # None = not known yet, True after ERR,ESTOP, False after an OK
        self.sample_unknown = ""

    def _open(self):
        if self.opener is not None:
            return self.opener(self.port, self.baud)
        import serial
        return serial.Serial(self.port, self.baud, timeout=0)

    def ok(self):
        return self.ser is not None

    def ensure(self):
        if self.ser is not None:
            return True
        if self.clock() < self.next_try:
            return False
        self.next_try = self.clock() + 1.0
        try:
            s = self._open()
        except Exception as e:  # SerialException, OSError (unplugged, no permission)
            hint = " (add yourself to 'dialout' and log in again)" if "ermission" in str(e) else ""
            self.log(f"cannot open {self.port}: {e}{hint}; retrying every 1 s", warn=True)
            return False
        self.sleep(self.RESET_WAIT_S)
        try:
            s.reset_input_buffer()
        except Exception:
            pass
        now = self.clock()
        self.ser, self.buf = s, b""
        self.last_rx = self.opened_at = now
        self.log(f"connected to {self.port}")
        self.write(P.CMD_PING)
        self.on_open()
        return True

    def drop(self, why):
        if self.ser is not None:
            self.log(f"serial link lost: {why}", warn=True)
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write(self, s):
        if self.ser is None:
            return
        try:
            self.ser.write(s.encode())
        except Exception as e:
            self.drop(e)

    def read(self):
        """Read what has arrived; return parsed messages (see protocol.parse_line) in order."""
        if self.ser is None:
            return []
        try:
            self.buf += self.ser.read(self.ser.in_waiting or 0)
        except Exception as e:
            self.drop(e)
            return []
        out = []
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode(errors="ignore").strip()
            if not line:
                continue
            self.last_rx = self.clock()
            m = P.parse_line(line)
            if m is None:
                self.counts["unknown"] += 1
                self.sample_unknown = line[:60]
                continue
            self.last_known = self.last_rx
            self.counts[m[0]] += 1
            if m[0] == "err":
                self.errors[m[1]] = self.errors.get(m[1], 0) + 1
                if m[1] == "ESTOP":
                    if self.estop_open is not True:
                        self.log("Uno refuses motor commands: ERR,ESTOP (the E-stop loop on D4 is open; "
                                 "D4 must be connected to GND through the closed E-stop)", warn=True)
                    self.estop_open = True
            elif m[0] == "ok":
                if self.estop_open:
                    self.log("E-stop loop closed again: motor commands accepted")
                self.estop_open = False
            out.append(m)
        if self.ser is not None and self.clock() - self.last_rx > self.SILENT_DROP_S:
            self.drop(f"nothing from the Uno for {self.SILENT_DROP_S:.0f} s (powered? right port?)")
        return out

    def alive(self):
        """The sketch answered in the last second (it replies OK/ERR to every M, so 1 s is generous)."""
        return self.ser is not None and self.clock() - self.last_known < 1.0

    def diagnosis(self):
        """One line for the status log: what the Uno is doing, and what to do about it."""
        if self.ser is None:
            return f"Uno NOT connected on {self.port}"
        settled = self.clock() - self.opened_at > 3.0
        if self.counts["unknown"] and not self.last_known and settled:
            return (f"WRONG SKETCH: the Uno sends '{self.sample_unknown}', which is not the ARC cart sketch "
                    "(M,/ENC,/OK). Upload firmware/arc_cart_motor/arc_cart_motor.ino - motors stay off")
        if self.estop_open:
            return "E-STOP OPEN: the Uno answers ERR,ESTOP to every command (close the loop on D4) - motors stay off"
        if not self.alive():
            return f"Uno connected but not answering yet ({sum(self.counts.values())} lines so far)"
        bad = {k: v for k, v in self.errors.items() if k != "ESTOP"}
        extra = f", errors {bad}" if bad else ""
        return f"Uno OK ({self.counts['ok']} OK, {self.counts['enc']} ENC{extra})"
