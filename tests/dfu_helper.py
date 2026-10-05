"""Host-only tests for DFU identity verification; no physical device access."""
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import enter_dfu as dfu

DEVICE = 'Found DFU: [0483:df11] ver=2200, alt=0, name="@Internal Flash", serial="ABC"'


class DfuTests(unittest.TestCase):
    def test_bundled_utility_takes_priority_over_system_install(self):
        expected = Path(dfu.__file__).resolve().parent / 'vendor/dfu-util/dfu-util.exe'
        with patch.object(Path, 'is_file', return_value=True), \
             patch.object(dfu.shutil, 'which') as which:
            self.assertEqual(dfu.find_dfu_util(), str(expected))
        which.assert_not_called()

    def test_only_rom_internal_flash_counts(self):
        self.assertEqual(dfu.dfu_devices(DEVICE), [DEVICE])
        for value in (DEVICE.replace('alt=0', 'alt=1'), DEVICE.replace('df11', '5740'),
                      DEVICE.replace('Found DFU:', 'Found Runtime:'), 'USB disconnected'):
            self.assertEqual(dfu.dfu_devices(value), [])

    def run_command(self, listings, **kwargs):
        device = MagicMock()
        device.__enter__.return_value = device
        # Reset disconnects the serial handle immediately.
        device.flush.side_effect = dfu.serial.SerialException('disconnected')
        device.readline.side_effect = dfu.serial.SerialException('disconnected')
        with patch.object(dfu, 'find_dfu_util', return_value='dfu-util'), \
             patch.object(dfu, 'dfu_list', side_effect=listings), \
             patch.object(dfu.serial, 'Serial', return_value=device):
            result = dfu.request_dfu('COM5', **kwargs)
        device.write.assert_called_once_with(b'\ndfu\n')
        device.__exit__.assert_called_once()
        return result

    def test_disconnect_followed_by_verified_dfu(self):
        self.assertEqual(self.run_command(['', DEVICE]), DEVICE)

    def test_serial_disconnect_is_not_success(self):
        with self.assertRaisesRegex(RuntimeError, 'DFU not verified'):
            self.run_command([''], timeout=0)

    def test_already_present_device_cannot_false_confirm(self):
        with self.assertRaisesRegex(RuntimeError, 'already present'):
            self.run_command([DEVICE])

    def test_multiple_devices_are_ambiguous(self):
        with self.assertRaisesRegex(RuntimeError, 'Multiple STM32'):
            self.run_command(['', DEVICE + '\n' + DEVICE.replace('ABC', 'DEF')])

    def test_firmware_refusal(self):
        device = MagicMock()
        device.__enter__.return_value = device
        device.readline.return_value = b'NAK,DFU,DISARM_IDLE_REQUIRED\n'
        with patch.object(dfu, 'find_dfu_util', return_value='dfu-util'), \
             patch.object(dfu, 'dfu_list', return_value=''), \
             patch.object(dfu.serial, 'Serial', return_value=device):
            with self.assertRaisesRegex(RuntimeError, 'Firmware refused'):
                dfu.request_dfu('COM5')
        device.__exit__.assert_called_once()


if __name__ == '__main__':
    unittest.main()
