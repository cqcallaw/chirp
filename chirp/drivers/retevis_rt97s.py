# Copyright 2026
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import struct
import logging

from chirp import chirp_common, directory, memmap, errors, bitwise
from chirp.settings import MemSetting, RadioSettingGroup, RadioSettings
from chirp.settings import RadioSettingValueBoolean, RadioSettingValueList

LOG = logging.getLogger(__name__)

MEM_FORMAT = """
#seekto 0x0010;
struct {
  lbcd rxfreq[4];
  lbcd txfreq[4];
  ul16 rx_tone;
  ul16 tx_tone;
  u8 unknown12;
  u8 unknown13;
  u8 unknown_hi:3,
     highpower:1,
     narrow:1,
     unknown_lo:3;
  u8 unknown15;
} memory[16];

#seekto 0x0330;
struct {
  u8 unknown0_hi:7,
     tail:1;
  u8 opt_bit7:1,
     transit:1,
     opt_bit5:1,
     keylock:1,
     opt_lo:4;
  u8 unknown1;
  u8 unknown2;
  u8 squelch;
  u8 unknown3;
  u8 relay;
} settings;
"""

MEM_SIZE = 0x03C0
BLOCK_SIZE = 0x10
# The factory program reads 0x0000-0x03C0 and writes every block except
# 0x0350-0x0370, which is a calibration curve it leaves on the radio.
WRITE_RANGES = [(0x0000, 0x0350), (0x0380, 0x03C0)]
CMD_ACK = b"\x06"
# Low power level is inferred from Internet sources
POWER_LEVELS = [chirp_common.PowerLevel("Low", watts=5.00),
                chirp_common.PowerLevel("High", watts=10.00)]
# Repeater receive is the 467 MHz input. Repeater transmit is the 462 MHz
# output. The factory program only offers these eight pairs.
TX_FREQS = [462550000, 462575000, 462600000, 462625000,
            462650000, 462675000, 462700000, 462725000]
RX_FREQS = [freq + 5000000 for freq in TX_FREQS]
SQUELCH_CHOICES = ["%i" % i for i in range(10)]
RELAY_CHOICES = ["%i" % i for i in range(10)]


def _read_exact(pipe, count):
    data = b""
    while len(data) < count:
        chunk = pipe.read(count - len(data))
        if not chunk:
            break
        data += chunk
    return data


def _checksum(data):
    return sum(data) & 0xFF


def _encode_tone(mode, value, polarity):
    if mode == "Tone":
        return int("%i" % int(round(value * 10)), 16)
    if mode == "DTCS":
        flag = 0xC000 if polarity == "R" else 0x8000
        return flag | int("%i" % value, 16)
    return 0xFFFF


def _decode_tone(value):
    if value in (0, 0xFFFF):
        return "", None, None
    if value & 0x8000:
        polarity = "R" if value & 0x4000 else "N"
        return "DTCS", int("%x" % (value & 0x0FFF)), polarity
    return "Tone", int("%x" % value) / 10.0, None


def _enter_programming_mode(radio):
    pipe = radio.pipe
    pipe.timeout = 2
    pipe.write(b"PROGRAM")
    ack = _read_exact(pipe, 1)
    if not ack:
        raise errors.RadioNoResponse()
    if ack != CMD_ACK:
        raise errors.RadioError("Radio refused to enter programming mode")

    pipe.write(b"\x02")
    ident = _read_exact(pipe, 8)
    if len(ident) != 8:
        raise errors.RadioNoResponse()
    if not ident.startswith(b"P3118"):
        raise errors.RadioError(
            "Radio identification failed: %r" % ident)

    pipe.write(CMD_ACK)
    ack = _read_exact(pipe, 1)
    if ack != CMD_ACK:
        raise errors.RadioError("Radio refused to enter programming mode")

    pipe.write(b"\x07")
    ack = _read_exact(pipe, 1)
    if ack != b"\x4e":
        raise errors.RadioError("Radio refused to enter programming mode")
    return ident


def _exit_programming_mode(radio):
    radio.pipe.write(b"E")


def _read_block(radio, addr):
    cmd = struct.pack(">cHB", b"R", addr, BLOCK_SIZE)
    radio.pipe.write(cmd)
    response = _read_exact(radio.pipe, 4 + BLOCK_SIZE + 1)
    expected = struct.pack(">cHB", b"W", addr, BLOCK_SIZE)
    if len(response) != 4 + BLOCK_SIZE + 1 or response[:4] != expected:
        raise errors.RadioError("No block at %04x" % addr)
    data = response[4:4 + BLOCK_SIZE]
    if _checksum(data) != response[-1]:
        raise errors.RadioError("Bad checksum at %04x" % addr)
    return data


def _write_block(radio, addr):
    data = radio.get_mmap()[addr:addr + BLOCK_SIZE]
    cmd = struct.pack(">cHB", b"W", addr, BLOCK_SIZE)
    radio.pipe.write(cmd + data + bytes([_checksum(data)]))
    ack = _read_exact(radio.pipe, 1)
    if ack != CMD_ACK:
        raise errors.RadioError("Radio refused block at %04x" % addr)


def do_download(radio):
    _enter_programming_mode(radio)
    data = b""
    status = chirp_common.Status()
    status.msg = "Cloning from radio"
    status.max = MEM_SIZE
    for addr in range(0, MEM_SIZE, BLOCK_SIZE):
        status.cur = addr + BLOCK_SIZE
        radio.status_fn(status)
        data += _read_block(radio, addr)
    _exit_programming_mode(radio)
    return memmap.MemoryMapBytes(data)


def do_upload(radio):
    _enter_programming_mode(radio)
    status = chirp_common.Status()
    status.msg = "Cloning to radio"
    blocks = []
    for start, end in WRITE_RANGES:
        blocks.extend(range(start, end, BLOCK_SIZE))
    status.max = len(blocks)
    for i, addr in enumerate(blocks, start=1):
        status.cur = i
        radio.status_fn(status)
        _write_block(radio, addr)
    _exit_programming_mode(radio)


@directory.register
class RT97SRadio(chirp_common.CloneModeRadio):
    """Retevis RT97S"""
    VENDOR = "Retevis"
    MODEL = "RT97S"
    BAUD_RATE = 9600

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_name = False
        rf.has_tuning_step = False
        rf.has_nostep_tuning = True
        rf.has_rx_dtcs = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.can_odd_split = True
        rf.valid_modes = ["FM", "NFM"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone", "DTCS->", "->DTCS",
                                "Tone->DTCS", "DTCS->Tone", "->Tone",
                                "DTCS->DTCS"]
        rf.valid_duplexes = ["split"]
        rf.valid_skips = []
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_bands = [(TX_FREQS[0], TX_FREQS[-1] + 25000),
                          (RX_FREQS[0], RX_FREQS[-1] + 25000)]
        rf.memory_bounds = (1, 16)
        return rf

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.info = _(
            "Channel frequencies are from the repeater's point of view. "
            "The receive frequency is the 467 MHz channel on which "
            "the repeater listens, and the offset is the 462 MHz channel where"
            " it transmits. A station using this repeater has those two "
            "frequencies swapped.")
        return rp

    def sync_in(self):
        try:
            self._mmap = do_download(self)
        except errors.RadioError:
            raise
        except Exception as exc:
            raise errors.RadioError(str(exc))
        self.process_mmap()

    def sync_out(self):
        try:
            do_upload(self)
        except errors.RadioError:
            raise
        except Exception as exc:
            raise errors.RadioError(str(exc))

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number - 1])

    @classmethod
    def match_model(cls, filedata, filename):
        return (len(filedata) >= MEM_SIZE and
                filedata[:6] == b"P3118\x13")

    def get_memory(self, number):
        _mem = self._memobj.memory[number - 1]
        mem = chirp_common.Memory()
        mem.number = number
        if _mem.get_raw() == b"\xff" * 16:
            mem.empty = True
            return mem

        mem.freq = int(_mem.rxfreq) * 10
        mem.duplex = "split"
        mem.offset = int(_mem.txfreq) * 10
        mem.immutable = ["duplex"]
        mem.mode = "NFM" if _mem.narrow else "FM"
        mem.power = POWER_LEVELS[1 if _mem.highpower else 0]
        txtone = _decode_tone(int(_mem.tx_tone))
        rxtone = _decode_tone(int(_mem.rx_tone))
        chirp_common.split_tone_decode(mem, txtone, rxtone)
        return mem

    def set_memory(self, mem):
        _mem = self._memobj.memory[mem.number - 1]
        if mem.empty:
            _mem.fill_raw(b"\xFF")
            return

        if _mem.get_raw() == b"\xff" * 16:
            _mem.fill_raw(b"\x00")

        _mem.rxfreq = mem.freq // 10
        if mem.duplex == "split":
            txfreq = mem.offset
        elif mem.duplex == "-":
            txfreq = mem.freq - mem.offset
        elif mem.duplex == "+":
            txfreq = mem.freq + mem.offset
        else:
            txfreq = mem.freq
        _mem.txfreq = txfreq // 10

        txtone, rxtone = chirp_common.split_tone_encode(mem)
        _mem.tx_tone = _encode_tone(*txtone)
        _mem.rx_tone = _encode_tone(*rxtone)
        _mem.narrow = mem.mode == "NFM"
        if mem.power is not None:
            _mem.highpower = str(mem.power) == "High"
        elif _mem.get_raw()[14] == 0:
            _mem.highpower = 1

    def validate_memory(self, mem):
        msgs = super().validate_memory(mem)
        if mem.empty:
            return msgs
        if mem.freq not in RX_FREQS:
            msgs.append(chirp_common.ValidationError(
                "Receive frequency must be a GMRS repeater input, "
                "467.550 through 467.725 MHz"))
        if mem.duplex == "split" and mem.offset not in TX_FREQS:
            msgs.append(chirp_common.ValidationError(
                "Offset must be the repeater transmit frequency, "
                "462.550 through 462.725 MHz"))
        return msgs

    def get_settings(self):
        _settings = self._memobj.settings
        group = RadioSettingGroup("basic", "Repeater")

        squelch = int(_settings.squelch)
        if squelch >= len(SQUELCH_CHOICES):
            squelch = 0
        group.append(MemSetting(
            "settings.squelch", "Squelch Level",
            RadioSettingValueList(SQUELCH_CHOICES, current_index=squelch)))

        relay = int(_settings.relay)
        if relay >= len(RELAY_CHOICES):
            relay = 0
        group.append(MemSetting(
            "settings.relay", "Relay Delay",
            RadioSettingValueList(RELAY_CHOICES, current_index=relay)))

        group.append(MemSetting(
            "settings.transit", "Transit Function",
            RadioSettingValueBoolean(_settings.transit)))
        group.append(MemSetting(
            "settings.keylock", "Key Lock",
            RadioSettingValueBoolean(_settings.keylock)))
        group.append(MemSetting(
            "settings.tail", "Tail Tone Eliminate",
            RadioSettingValueBoolean(_settings.tail)))
        return RadioSettings(group)

    def set_settings(self, settings):
        settings.apply_to(self._memobj)
