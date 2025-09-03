#!/usr/bin/env python
"""
Mock implementation of scservo_sdk for testing purposes.
This is a minimal implementation that provides the necessary constants and classes
used by the Feetech motors implementation.
"""

# Constants
COMM_SUCCESS = 0
COMM_RX_TIMEOUT = 1
COMM_RX_CORRUPT = 2
COMM_TX_FAIL = 3
COMM_TX_ERROR = 4
COMM_RX_FAIL = 5
COMM_TX_SUCCESS = 6

MAX_ID = 253
BROADCAST_ID = 254

# Instruction types
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_FACTORY_RESET = 6
INST_REBOOT = 8
INST_STATUS = 85
INST_SYNC_READ = 130
INST_SYNC_WRITE = 131
INST_BULK_READ = 146
INST_BULK_WRITE = 147

# Packet indices
PKT_ID = 2
PKT_LENGTH = 3
PKT_INSTRUCTION = 4
PKT_ERROR = 4
PKT_PARAMETER0 = 5

# Error codes
ERR_RESULT_FAIL = 1
ERR_INSTRUCTION = 2
ERR_CRC = 3
ERR_DATA_RANGE = 4
ERR_DATA_LENGTH = 5
ERR_DATA_LIMIT = 6
ERR_ACCESS = 7

# Utility functions
def SCS_LOBYTE(x):
    return x & 0xFF

def SCS_HIBYTE(x):
    return (x >> 8) & 0xFF

def SCS_LOWORD(x):
    return x & 0xFFFF

def SCS_HIWORD(x):
    return (x >> 16) & 0xFFFF

def SCS_MAKEWORD(low, high):
    return (high << 8) | low

def SCS_MAKEDWORD(low, high):
    return (high << 16) | low

# Mock classes
class PortHandler:
    def __init__(self, port):
        self.port = port
        self.is_open = False
        self.baudrate = 1000000
        self.port_name = port
        
    def openPort(self):
        self.is_open = True
        return True
        
    def closePort(self):
        self.is_open = False
        return True
        
    def setBaudRate(self, baudrate):
        self.baudrate = baudrate
        return True
        
    def setPacketTimeoutMillis(self, timeout):
        self.timeout = timeout
        return True
        
    def setPacketTimeout(self, packet_length):
        return True
        
    def writePort(self, packet):
        return len(packet)
        
    def readPort(self, packet):
        return 0

class PacketHandler:
    def __init__(self, protocol_version):
        self.protocol_version = protocol_version

class GroupSyncRead:
    def __init__(self, port_handler, packet_handler, start_address, data_length):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.start_address = start_address
        self.data_length = data_length
        self.data_list = {}
        
    def addParam(self, id, start_address, data_length):
        pass
        
    def clearParam(self):
        pass
        
    def txPacket(self):
        return COMM_SUCCESS
        
    def isAvailable(self, id, start_address, data_length):
        return True
        
    def getData(self, id, start_address, data_length):
        return 0

class GroupSyncWrite:
    def __init__(self, port_handler, packet_handler, start_address, data_length):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.start_address = start_address
        self.data_length = data_length
        
    def addParam(self, id, start_address, data_length, data):
        pass
        
    def changeParam(self, id, start_address, data_length, data):
        pass
        
    def removeParam(self, id):
        pass
        
    def changeParam(self, id, start_address, data_length, data):
        pass
        
    def clearParam(self):
        pass
        
    def txPacket(self):
        return COMM_SUCCESS
