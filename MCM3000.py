import sys
import time
import serial
from serial.tools.list_ports import comports


_SUPPORTED_STAGES = {
#   'Type': (_um_per_count, +- _position_limit_um, )
    'ZFM2020':( 0.2116667, 1e3 * 12.7),
    'ZFM2030':( 0.2116667, 1e3 * 12.7),
    'MMP-2XY':(0.5, 1e3 * 25.4),
    'PLS-X':(0.2116667, 1e3 * 12.7),
    'PLS-XY':(0.2116667, 1e3 * 12.7)
}


def supported_stages() -> dict:
    return _SUPPORTED_STAGES


class MCM3000:
    '''
    Serial interface for MCM3000 motorized stage controller from Thorlabs.
    '''
    def __init__(self,
                 which_port,
                 stages=3*(None,), # connected e.g. (None, None, 'ZFM2030')
                 reverse=3*(False,), # reverse e.g. (False, False, True)
                 verbose=True,
                 very_verbose=False
         ):
        self.stages = stages
        self.reverse = reverse        
        self.verbose = verbose
        self.very_verbose = very_verbose
        if which_port == 'auto':
            which_port = None
            for port in list(comports()):
                if "MCM3000" in port.description:
                    if self.verbose:
                        print('Found MCM3000 on port', port.name)
                    which_port = str(port.name)
            if which_port is None:
                raise IOError('Unable to automatically detect MCM3000 on any port.')
        self.name = 'MCM3000 ({})'.format(which_port)            
        if self.verbose: print("%s: opening..."%self.name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=460800, timeout=5)
        except serial.serialutil.SerialException:
            raise IOError(
                '%s: no connection on port %s'%(self.name, which_port))
        if self.verbose: print(" done.")
        assert type(self.stages) == tuple and type(self.reverse) == tuple
        assert len(self.stages) == 3 and len(self.reverse) == 3
        for element in self.reverse: assert type(element) == bool
        self._encoder_counts= 3*[None]
        self._encoder_counts_tol= 3*[1] # can hang if < 1 count
        self._target_encoder_counts= 3*[None]
        self._um_per_count = 3*[None]
        self._position_limit_um = 3*[None]
        self.position_um = 3*[None]
        self.channels = []
        for channel, stage in enumerate(self.stages):
            if stage is not None:
                assert stage in _SUPPORTED_STAGES, (
                    '%s: stage \'%s\' not supported'%(self.name, stage))
                self.channels.append(channel)
                self._um_per_count[channel] = _SUPPORTED_STAGES[stage][0]
                self._position_limit_um[channel] = _SUPPORTED_STAGES[stage][1]
                self._get_encoder_counts(channel)
        self.channels = tuple(self.channels)
        if self.verbose:
            print("%s: stages:"%self.name, self.stages)
            print("%s: reverse:"%self.name, self.reverse)
            print("%s: um_per_count:"%self.name, self._um_per_count)
            print("%s: position_limit_um:"%self.name, self._position_limit_um)
            print("%s: position_um:"%self.name, self.position_um)

    def _send(self, cmd, channel, response_bytes=None):
        assert channel in self.channels, (
            '%s: channel \'%s\' is not available'%(self.name, channel))
        if self.very_verbose:
            print('%s (ch%s): sending cmd: %s'%(self.name, channel, cmd))
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        if self.very_verbose:
            print('%s (ch%s): -> response: %s'%(self.name, channel, response))
        return response

    def _encoder_counts_to_um(self, channel, encoder_counts):
        um = encoder_counts * self._um_per_count[channel]
        if self.reverse[channel]:
            um = - um + 0  # +0 avoids -0.0
        if self.very_verbose:
            print('%s (ch%s): -> encoder counts %i = %0.2fum'%(
                self.name, channel, encoder_counts, um))
        return um

    def _um_to_encoder_counts(self, channel, um):
        encoder_counts = int(round(um / self._um_per_count[channel]))
        if self.reverse[channel]:
            encoder_counts = - encoder_counts + 0  # +0 avoids -0.0
        if self.very_verbose:
            print('%s (ch%s): -> %0.2fum = encoder counts %i'%(
                self.name, channel, um, encoder_counts))
        return encoder_counts

    def _get_encoder_counts(self, channel):
        if self.very_verbose:
            print('%s (ch%s): getting encoder counts'%(self.name, channel))
        channel_byte = channel.to_bytes(1, byteorder='little')
        cmd = b'\x0a\x04' + channel_byte + b'\x00\x00\x00'
        response = self._send(cmd, channel, response_bytes=12)
        assert response[6:7] == channel_byte # channel = selected
        self._encoder_counts[channel] = int.from_bytes(
            response[-4:], byteorder='little', signed=True)
        if self.very_verbose:
            print('%s (ch%s): -> encoder counts = %i'%(
                self.name, channel, self._encoder_counts[channel]))
        self.position_um[channel] = self._encoder_counts_to_um(
            channel, self._encoder_counts[channel])
        return self._encoder_counts[channel]

    def _set_encoder_counts_to_zero(self, channel):
        # assumes the stage encoder will be set to zero at the centre of its range when checking limits.
        if self.verbose:
            print('%s (ch%s): setting encoder counts to zero'%(
                self.name, channel))
        channel_byte = channel.to_bytes(2, byteorder='little')
        encoder_bytes = (0).to_bytes(4, 'little', signed=True) # set to zero
        cmd = b'\x09\x04\x06\x00\x00\x00' + channel_byte + encoder_bytes
        self._send(cmd, channel)
        while True:
            self._get_encoder_counts(channel)
            if self._encoder_counts[channel] == 0:
                break
        if self.verbose:
            print('%s (ch%s): -> done'%(self.name, channel))
        return
    
    def _move_to_encoder_count(self, channel, encoder_counts, timeout_s=False):
        if self._target_encoder_counts[channel] is not None:
            self._finish_move(channel)
        if self.very_verbose:
            print('%s (ch%s): moving to encoder counts = %i'%(
                self.name, channel, encoder_counts))
        self._target_encoder_counts[channel] = encoder_counts
        encoder_bytes = encoder_counts.to_bytes(4, 'little', signed=True)
        channel_bytes = channel.to_bytes(2, byteorder='little')
        cmd = b'\x53\x04\x06\x00\x00\x00' + channel_bytes + encoder_bytes
        self._send(cmd, channel)
        if timeout_s is not False:
            self._finish_move(channel, timeout_s=timeout_s)
        return

    def _finish_move(self, channel, timeout_s=5):
        if self._target_encoder_counts[channel] is None:
            return
        start = time.time()
        while time.time() - start < timeout_s:
            encoder_counts = self._get_encoder_counts(channel)
            target = self._target_encoder_counts[channel]
            tolerance = self._encoder_counts_tol[channel]
            if self.very_verbose:
                print('%s (ch%s): %i counts from target (tolerance %i)'%(
                    self.name, channel, abs(target - encoder_counts), tolerance))
                time.sleep(0.1)
            if abs(target - encoder_counts) <= tolerance:
                if self.verbose:
                    print('\n%s (ch%s): -> finished move.'%(self.name, channel))
                self._target_encoder_counts[channel] = None
                return
        self._target_encoder_counts[channel] = None
        if self.verbose:
            print('\n%s (ch%s): -> timed out %i steps from target.'%(self.name, channel, abs(target - encoder_counts)))
        return

    def _legalize_move_um(self, channel, move_um, relative):
        if self.verbose:
            print('%s (ch%s): requested move_um = %0.2f (relative=%s)'%(
                self.name, channel, move_um, relative))
        if relative:
            # update position in case external device was used (e.g. joystick)
            self._get_encoder_counts(channel)
            move_um += self.position_um[channel]
        limit_um = self._position_limit_um[channel]
        assert - limit_um <= move_um <= limit_um, (
            '%s: ch%s -> move_um (%0.2f) exceeds position_limit_um (%0.2f)'%(
                self.name, channel, move_um, limit_um))
        move_counts = self._um_to_encoder_counts(channel, move_um)
        legal_move_um = self._encoder_counts_to_um(channel, move_counts)
        if self.verbose:
            print('%s (ch%s): -> legal move_um = %0.2f '%(
                self.name, channel, legal_move_um) +
                  '(%0.2f requested)'%move_um)
        return legal_move_um
    
    def get_position_um(self, channel) -> float:
        self._get_encoder_counts(channel)
        return self.position_um[channel]
    
    def get_position_encoder_counts(self, channel) -> int:
        return int(self._get_encoder_counts(channel))
    
    def get_position_limits_um(self, channel) -> tuple:
        return (-self._position_limit_um[channel], self._position_limit_um[channel])
    
    def move_um(self, channel, move_um, relative=True, timeout_s=False):
        legal_move_um = self._legalize_move_um(channel, move_um, relative)
        if self.verbose:
            print('%s (ch%s): moving to position_um = %0.2f'%(
                self.name, channel, legal_move_um))
        encoder_counts = self._um_to_encoder_counts(channel, legal_move_um)
        self._move_to_encoder_count(channel, encoder_counts, timeout_s)
        return legal_move_um

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        self.port.close()
        if self.verbose: print("done.")
        

if __name__ == '__main__':
    
    controller = MCM3000(which_port='auto',
                            stages=('ZFM2020', 'ZFM2020', 'ZFM2020'),
                            reverse=(True, True, True),
                            verbose=True,
                            very_verbose=True)
    input('\n\nWARNING!!! This script will attempt to home (zero) all 3 motors connected to the MCM3000! If you are uncertain where these zeros are, COLLISIONS MAY OCCUR! Press ENTER to proceed.')
     
    for ch in range(3):    
        controller.move_um(ch, 0, relative=False, timeout_s=False)

    start = time.time()
    while time.time() - start < 5.0:
        for ch in range(3):
            print('\n# Homing channel', ch, '...')        
            print('-> position_um = %0.2f'%controller.get_position_um(ch))     
            print('-> position_encoder = %0.2f'%controller.get_position_encoder_counts(ch))
        time.sleep(1)
        
    for ch in range(3):
        print('\n# Final position for channel', ch)        
        print('-> position_um = %0.2f'%controller.get_position_um(ch))     
        print('-> position_encoder = %0.2f'%controller.get_position_encoder_counts(ch))
    
    input('Press ENTER to proceed with testing.')
    
    for channel in range(3):
    
        print('\n# Some relative moves:')
        for moves in range(3):
            controller.move_um(channel, 10, timeout_s=10)
        for moves in range(3):
            controller.move_um(channel, -10, timeout_s=10)
    
    
        print('\n# Encoder tolerance check:')
        # hangs indefinetly if self._encoder_counts_tol[channel] < 1 count
        for i in range(3):
            controller.move_um(channel, 0, relative=False)
            controller.move_um(channel, 0.2116667, relative=False)

    controller.close()
