import sys
import time
import serial
from serial.tools.list_ports import comports


_SUPPORTED_STAGES = {
    'ZFM2020': (0.2116667, 1e3 * 12.7),
    'ZFM2030': (0.2116667, 1e3 * 12.7),
    'MMP-2XY': (0.5, 1e3 * 25.4),
    'PLS-X': (0.2116667, 1e3 * 12.7),
    'PLS-XY': (0.2116667, 1e3 * 12.7)
}


def supported_stages() -> dict:
    """Return the supported stages dictionary."""
    return _SUPPORTED_STAGES


class MCM3000:
    """
    Serial interface for MCM3000 motorized stage controller from Thorlabs.
    """

    def __init__(self, which_port: str, stages=(None, None, None), reverse=(False, False, False), verbose=False, very_verbose=False):
        """
        Initialize the MCM3000 controller.
        :param which_port: The serial port to use, or 'auto' to automatically detect.
        :param stages: Tuple of connected stages names, e.g., (None, None, 'ZFM2030'). For supported stage names, make a call to `supported_stages`.
        :param reverse: Tuple indicating whether the stage translation direction is reversed.
        :param verbose: Enable verbose output.
        :param very_verbose: Enable very verbose output describing each serial message.
        """
        self.stages = stages
        self.reverse = reverse
        self.verbose = verbose
        self.very_verbose = very_verbose

        # Automatically detect the MCM3000 on any port if specified
        if which_port == 'auto':
            which_port = None
            for port in list(comports()):
                if "MCM3000" in port.description:
                    if self.verbose:
                        print('Found MCM3000 on port', port.name)
                    which_port = str(port.name)
            if which_port is None:
                raise IOError('Unable to automatically detect MCM3000 on any port.')

        self.name = f'MCM3000 ({which_port})'
        if self.verbose:
            print(f"{self.name}: opening...", end='')

        try:
            self.port = serial.Serial(port=which_port, baudrate=460800, timeout=5)
        except serial.serialutil.SerialException:
            raise IOError(f'{self.name}: no connection on port {which_port}')

        if self.verbose:
            print(" done.")

        assert len(self.stages) == 3 and len(self.reverse) == 3, 'Stages and reverse tuples must have length 3'
        for element in self.reverse:
            assert isinstance(element, bool), 'Reverse elements must be boolean'

        self._encoder_counts = [None] * 3
        self._encoder_counts_tol = [1] * 3  # Can hang if < 1 count
        self._target_encoder_counts = [None] * 3
        self._um_per_count = [None] * 3
        self._position_limit_um = [None] * 3
        self.position_um = [None] * 3
        self.channels = []

        for channel, stage in enumerate(self.stages):
            if stage is not None:
                assert stage in _SUPPORTED_STAGES, f"{self.name}: stage '{stage}' not supported"
                self.channels.append(channel)
                self._um_per_count[channel] = _SUPPORTED_STAGES[stage][0]
                self._position_limit_um[channel] = _SUPPORTED_STAGES[stage][1]
                self._get_encoder_counts(channel)

        self.channels = tuple(self.channels)
        if self.verbose:
            print(f"{self.name}: stages: {self.stages}")
            print(f"{self.name}: reverse: {self.reverse}")
            print(f"{self.name}: um_per_count: {self._um_per_count}")
            print(f"{self.name}: position_limit_um: {self._position_limit_um}")
            print(f"{self.name}: position_um: {self.position_um}")

    def _send(self, cmd: bytes, channel: int, response_bytes: int = None) -> bytes:
        """Send a command to the specified channel and optionally read the response."""
        assert channel in self.channels, f"{self.name}: channel '{channel}' is not available"
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): sending cmd: {cmd}')
        self.port.write(cmd)
        response = self.port.read(response_bytes) if response_bytes is not None else None
        assert self.port.inWaiting() == 0, f"{self.name}: unexpected data in the input buffer"
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): -> response: {response}')
        return response

    def _encoder_counts_to_um(self, channel: int, encoder_counts: int) -> float:
        """Convert encoder counts to micrometers for the specified channel."""
        um = encoder_counts * self._um_per_count[channel]
        if self.reverse[channel]:
            um = -um + 0  # +0 avoids -0.0
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): -> encoder counts {encoder_counts} = {um:.2f}um')
        return um

    def _um_to_encoder_counts(self, channel: int, um: float) -> int:
        """Convert micrometers to encoder counts for the specified channel."""
        encoder_counts = int(round(um / self._um_per_count[channel]))
        if self.reverse[channel]:
            encoder_counts = -encoder_counts + 0  # +0 avoids -0.0
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): -> {um:.2f}um = encoder counts {encoder_counts}')
        return encoder_counts

    def _get_encoder_counts(self, channel: int) -> int:
        """Get the current encoder counts for the specified channel."""
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): getting encoder counts')
        channel_byte = channel.to_bytes(1, byteorder='little')
        cmd = b'\x0a\x04' + channel_byte + b'\x00\x00\x00'
        response = self._send(cmd, channel, response_bytes=12)
        assert response[6:7] == channel_byte, f"{self.name} (ch{channel}): response channel mismatch"
        self._encoder_counts[channel] = int.from_bytes(response[-4:], byteorder='little', signed=True)
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): -> encoder counts = {self._encoder_counts[channel]}')
        self.position_um[channel] = self._encoder_counts_to_um(channel, self._encoder_counts[channel])
        return self._encoder_counts[channel]

    def _set_encoder_counts_to_zero(self, channel: int) -> None:
        """Set the encoder counts to zero for the specified channel."""
        if self.verbose:
            print(f'{self.name} (ch{channel}): setting encoder counts to zero')
        channel_byte = channel.to_bytes(2, byteorder='little')
        encoder_bytes = (0).to_bytes(4, byteorder='little', signed=True)
        cmd = b'\x09\x04\x06\x00\x00\x00' + channel_byte + encoder_bytes
        self._send(cmd, channel)
        while True:
            self._get_encoder_counts(channel)
            if self._encoder_counts[channel] == 0:
                break
        if self.verbose:
            print(f'{self.name} (ch{channel}): -> done')

    def _move_to_encoder_count(self, channel: int, encoder_counts: int, timeout_s: float = None) -> None:
        """Move to the specified encoder counts for the given channel."""
        if self._target_encoder_counts[channel] is not None:
            self._finish_move(channel)
        if self.very_verbose:
            print(f'{self.name} (ch{channel}): moving to encoder counts = {encoder_counts}')
        self._target_encoder_counts[channel] = encoder_counts
        encoder_bytes = encoder_counts.to_bytes(4, byteorder='little', signed=True)
        channel_bytes = channel.to_bytes(2, byteorder='little')
        cmd = b'\x53\x04\x06\x00\x00\x00' + channel_bytes + encoder_bytes
        self._send(cmd, channel)
        if timeout_s is not None:
            self._finish_move(channel, timeout_s)

    def _finish_move(self, channel: int, timeout_s: float = 5) -> None:
        """Wait for the move to finish or timeout."""
        if self._target_encoder_counts[channel] is None:
            return
        start = time.time()
        while time.time() - start < timeout_s:
            encoder_counts = self._get_encoder_counts(channel)
            target = self._target_encoder_counts[channel]
            tolerance = self._encoder_counts_tol[channel]
            if self.very_verbose:
                print(f'{self.name} (ch{channel}): {abs(target - encoder_counts)} counts from target (tolerance {tolerance})')
                time.sleep(0.1)
            if abs(target - encoder_counts) <= tolerance:
                if self.verbose:
                    print(f'\n{self.name} (ch{channel}): -> finished move.')
                self._target_encoder_counts[channel] = None
                return
        if self.verbose:
            print(f'\n{self.name} (ch{channel}): -> timed out {abs(self._target_encoder_counts[channel] - self._get_encoder_counts(channel))} steps from target.')
        self._target_encoder_counts[channel] = None

    def _legalize_move_um(self, channel: int, move_um: float, relative: bool) -> float:
        """Ensure the requested move is within legal limits."""
        if self.verbose:
            print(f'{self.name} (ch{channel}): requested move_um = {move_um:.2f} (relative={relative})')
        if relative:
            self._get_encoder_counts(channel)
            move_um += self.position_um[channel]
        limit_um = self._position_limit_um[channel]
        assert -limit_um <= move_um <= limit_um, f"{self.name}: ch{channel} -> move_um ({move_um:.2f}) exceeds position_limit_um ({limit_um:.2f})"
        move_counts = self._um_to_encoder_counts(channel, move_um)
        legal_move_um = self._encoder_counts_to_um(channel, move_counts)
        if self.verbose:
            print(f'{self.name} (ch{channel}): -> legal move_um = {legal_move_um:.2f} ({move_um:.2f} requested)')
        return legal_move_um

    def get_position_um(self, channel: int) -> float:
        """Get the current position in micrometers for the specified channel."""
        self._get_encoder_counts(channel)
        return self.position_um[channel]

    def get_position_encoder_counts(self, channel: int) -> int:
        """Get the current position in encoder counts for the specified channel."""
        return int(self._get_encoder_counts(channel))

    def get_position_limits_um(self, channel: int) -> tuple:
        """Get the position limits in micrometers for the specified channel."""
        return -self._position_limit_um[channel], self._position_limit_um[channel]

    def move_um(self, channel: int, move_um: float, relative: bool = True, timeout_s: float = None) -> float:
        """Move the specified channel by the given distance in micrometers."""
        legal_move_um = self._legalize_move_um(channel, move_um, relative)
        if self.verbose:
            print(f'{self.name} (ch{channel}): moving to position_um = {legal_move_um:.2f}')
        encoder_counts = self._um_to_encoder_counts(channel, legal_move_um)
        self._move_to_encoder_count(channel, encoder_counts, timeout_s)
        return legal_move_um

    def close(self) -> None:
        """Close the serial port connection."""
        if self.verbose:
            print(f"{self.name}: closing...", end=' ')
        self.port.close()
        if self.verbose:
            print("done.")


if __name__ == '__main__':
    controller = MCM3000(
        which_port='auto',
        stages=('ZFM2020', 'ZFM2020', 'ZFM2020'),
        reverse=(True, True, True),
        verbose=True,
        very_verbose=False
    )

    input('\n\nWARNING!!! This script will attempt to home (zero) all 3 motors connected to the MCM3000! If you are uncertain where these zeros are, COLLISIONS MAY OCCUR! Press ENTER to proceed.')

    for ch in range(3):
        controller.move_um(ch, 0, relative=False, timeout_s=False)

    start = time.time()
    while time.time() - start < 5.0:
        for ch in range(3):
            print('\n# Homing channel', ch, '...')
            print('-> position_um = {:.2f}'.format(controller.get_position_um(ch)))
            print('-> position_encoder = {:.2f}'.format(controller.get_position_encoder_counts(ch)))
        time.sleep(1)

    for ch in range(3):
        print('\n# Final position for channel', ch)
        print('-> position_um = {:.2f}'.format(controller.get_position_um(ch)))
        print('-> position_encoder = {:.2f}'.format(controller.get_position_encoder_counts(ch)))

    input('Press ENTER to proceed with testing.')

    for channel in range(3):
        print('\n# Some relative moves:')
        for _ in range(3):
            controller.move_um(channel, 10, timeout_s=10)
        for _ in range(3):
            controller.move_um(channel, -10, timeout_s=10)

        print('\n# Encoder tolerance check:')
        for _ in range(3):
            controller.move_um(channel, 0, relative=False)
            controller.move_um(channel, 0.2116667, relative=False)

    controller.close()
