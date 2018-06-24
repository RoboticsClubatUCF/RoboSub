#!/usr/bin/env python3
import smbus, os, struct, time
from datetime import datetime

class FuelGauge:
    def __init__(self, address=0x64, bus=None, shunt=0.002):
        self.address = address
        
        self.charge = None
        self.voltage = None
        self.current = None
        self.temperature = None
        
        self.timestamp = None
        
        self.shunt = shunt
        self.prescalar = 4096
        
        self.prescalarLookup = {1:0x00,
                                4:0b00001000,
                                16:0b00010000,
                                64:0b00011000,
                                256:0b00100000,
                                1024:0b00101000,
                                4096:0b00111000}
        
        self.registerLookup = {"status" : 0x00,
                               "control" : 0x01,
                               "chargeMSB": 0x02,
                               "voltageMSB": 0x08,
                               "currentMSB": 0x0E,
                               "temperatureMSB": 0x14}
        
        if bus is None:
            for b in range(0,3):
                path = '/dev/i2c-' + str(b)
                try:
                    s = os.stat(path)
                    bus = b
                    break
                except:
                    pass
            if bus is None:
                bus = 1
			
        self.bus = bus
		
        try:
            self.i2c = smbus.SMBus(self.bus)
        except:
            raise IOError("Cant open i2c bus")

        return
	
    def initSensor(self, prescalar):
        if prescalar not in self.prescalarLookup:
            raise ValueError("Prescalar value not valid")
            return
	    
        self.prescalar = prescalar
	    
        controlByte = 0b11000000
        controlByte |= self.prescalarLookup[self.prescalar]
	    
        try:
            self.i2c.write_byte_data(self.address, self.registerLookup["control"], controlByte)
        except:
            raise IOError("Could not write control data to device at %s" % self.address)
	    
    def resetCharge(self):
        data = [0x7F, 0xFF]
        try:
            self.i2c.write_block_data(self.address, self.registerLookup["chargeMSB"], data)
        except:
            raise IOError("Could not write charge data to device at %s" % self.address)
	    
    def read(self):
        try:
            chargeBuf = bytes(self.i2c.read_i2c_block_data(self.address, self.registerLookup["chargeMSB"], 2))
            voltageBuf = bytes(self.i2c.read_i2c_block_data(self.address, self.registerLookup["voltageMSB"], 2))
            currentBuf = bytes(self.i2c.read_i2c_block_data(self.address, self.registerLookup["currentMSB"], 2))
            temperatureBuf = bytes(self.i2c.read_i2c_block_data(self.address, self.registerLookup["temperatureMSB"], 2))

            self.charge = float(struct.unpack('>H', chargeBuf)[0]-0x7FFF) * 0.34 * 0.05/self.shunt * self.prescalar / 4096
            self.voltage = 23.6 * float(struct.unpack('>H', voltageBuf)[0])/65535.0
            self.current = 0.06/self.shunt * float(struct.unpack('>H', currentBuf)[0]-0x7FFF)/32767.0
            self.temperature = 510 * float(struct.unpack('>H', temperatureBuf)[0])/0xFFFF
            self.temperature = self.temperature * 9/5 - 459.67
            
            self.timestamp = datetime.now()
            
        except:
	        raise IOError("Could not read data from device at %s" % self.address)
	        
	
if __name__ == "__main__":
    import rospy
    from sensor_msgs.msg import BatteryState
    
    fg = FuelGauge(address=0x64, bus=1)
    fg.initSensor(256)
    
    r = rospy.rate(4)
    
    batPub = rospy.Publisher("/BatteryStatus", BatteryState, queue_size = 10)
    msg = BatteryState()
    msg.header.frame_id("battery1")
    msg.design_capacity = 12.6
    
    while not rospy.is_shutdown():
        fg.read()
        msg.voltage = fg.voltage
        msg.current = fg.current/1000
        msg.charge = fg.charge/1000
        msg.percentage = 1+(msg.charge/msg.design_capacity)
        msg.header.stamp = rospy.Time.from_sec(fg.timestamp.timestamp())
        batPub.publish(msg)
        rate.sleep(0.25)