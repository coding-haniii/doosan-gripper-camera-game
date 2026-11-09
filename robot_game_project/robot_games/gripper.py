"""Optional separate RS-485 USB adapter control; no motion by default."""
class RobotisGripper:
    MODEL=35074
    def __init__(self, config):
        if not config.get('enabled'): raise PermissionError('Gripper disabled')
        if config.get('open_position') is None or config.get('closed_position') is None:
            raise ValueError('Teach open/closed positions on real gripper')
        from dynamixel_sdk import PortHandler,PacketHandler,COMM_SUCCESS
        self.COMM_SUCCESS=COMM_SUCCESS; self.cfg=config
        self.port=PortHandler(config['port']); self.packet=PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(config['baud']): raise ConnectionError('RS-485 adapter connection failed')
        model,comm,err=self.packet.read2ByteTxRx(self.port,config['id'],0)
        if comm!=COMM_SUCCESS or err or model!=self.MODEL:
            self.port.closePort();raise RuntimeError(f'Unexpected gripper ID/model/read: {model,comm,err}')
    def _write(self,addr,val,width):
        func=self.packet.write1ByteTxRx if width==1 else self.packet.write4ByteTxRx
        comm,err=func(self.port,self.cfg['id'],addr,val)
        if comm!=self.COMM_SUCCESS or err: raise RuntimeError(f'Gripper write failed: {comm}/{err}')
    def torque(self,on): self._write(512,int(on),1)
    def move(self,open_=True):
        self._write(564,int(self.cfg['open_position'] if open_ else self.cfg['closed_position']),4)
    def close(self):
        self.torque(False);self.port.closePort()
