'''
TA151601047
Author: Fadhil Mochammad / 13212118
Electrical Engineering Institut Teknologi Bandung
'''

import serial, time, struct
import sys, os, csv
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

'''global variable'''
FC_RAWDATA = 5
FC_CHVID = 10


class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class MsgHeader(object):
	DeviceId = FC_CHVID
	FunctionCode = FC_RAWDATA


class MsgTail(object): 
	CrcHi = None
	CrcLo = None


class SerDataHeader(object):
	FunctionCode = None
	NumOfBytes = None
	NumOfBytes16Hi = None
	NumOfBytes16Lo = None

class DataMeasurement(object):
	Valid = None
	InvalidityReason = None
	ConcClassification = None
	GasName = []
	StateHi = None
	StateLo = None

class DataGPS(object):
	Latitude = 0
	Longitude = 0
	Altitude = 0

class SerialInterface(object):
	''' Serial Interface with Sensor'''
	def __init__(self, device, baudrate=38400):
		self.device = device
		self.baudrate = baudrate
		self.port = serial.Serial(self.device, baudrate=self.baudrate,
                                  dsrdtr=False, rtscts=False, xonxoff=False, 
                                  stopbits=serial.STOPBITS_TWO)
		print "[Chempro100i] Use %s baudrate %d\n" % (self.device, self.baudrate)
		time.sleep(1)

	def set_baudrate(self, baudrate):
		'''set baudrate'''
		try:
			self.port.setBaudrate(baudrate)
		except Exception:
			# for pySerial 3.0, which doesn't have setBaudrate()
			self.port.baudrate = baudrate
		time.sleep(1)

	def close(self):
		'''close serial'''
		self.port.close()

	def flush(self):
		'''flush buffer serial'''
		self.port.flush()

	def recv(self,n):
		'''receive message from serial port'''
		ret = self.port.read(n)
		return ret

	def write(self, buf):
		'''send message to serial port'''
		self.port.flush()
		Buffer = [chr(i) for i in buf]
		#Join the list into a string
		self.port.write ("".join(Buffer))

	def set_timeout(self, timeout, inter_timeout):
		'''set serial timeout'''
		self.port.timeout = timeout
		self.port.inter_byte_timeout = inter_timeout


class Chempro100i(object):
	''' Class for Serial Interface with Chempro100i '''
	def __init__(self, device, baudrate=38400):
		self.device = device
		self.baudrate = baudrate
		self.auchCRCHi = [
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 ]


		self.auchCRCLo = [
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
		0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
		0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
		0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
		0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
		0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
		0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
		0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
		0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
		0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
		0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
		0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
		0x43, 0x83, 0x41, 0x81, 0x80, 0x40 ]


	def CRC16(self,puchMsg,usDataLen,PrevCRC=0xffff):
		uchCRCHi = PrevCRC #>> 8
		uchCRCLo = PrevCRC

		uIndex = 0
		DataIndex=0

		while(usDataLen != 0):
			uIndex = uchCRCHi ^ puchMsg[DataIndex]
			uchCRCHi = uchCRCLo ^ self.auchCRCHi[uIndex]
			uchCRCLo = self.auchCRCLo[uIndex]
			usDataLen -= 1
			DataIndex += 1

		RetValue = (uchCRCHi << 8)|uchCRCLo
		return RetValue

	def IsOk_CRC16(self,puchMsg,usDataLen):
		crc = 0
		if(usDataLen >= 4):
			crc = self.CRC16(puchMsg,usDataLen-2,0xff)
			if((puchMsg[usDataLen-1]==(crc & 0xff)) and (puchMsg[usDataLen-2]==crc>>8)):
				return 1
		return 0

	def debug(self,message):
		print "[Chempro100i] [Debug] ",message

	def req_baudrate(self):
		'''send request message to change baudrate via serial'''
		WriteBufferDataBaudRate = [10,5,159,6,0,6,3,0,224,42]
		ReqLen = 6 #Request Message Baud Rate Length
		RespBytes = 8 #Response Message Length
		RepLen = 4 
		VarLen = 0
		self.ResponseMessage = []

		SerialSensor.flush()
		SerialSensor.write(WriteBufferDataBaudRate)
		SerialSensor.set_timeout(1,0.1)
		self.ResponseMessage = SerialSensor.recv(8)
		Converted = [ord(i) for i in self.ResponseMessage]

		if (len(Converted)==8):
			MsgHeader.DeviceId,MsgHeader.FunctionCode,SerDataHeader.FunctionCode,SerDataHeader.NumOfBytes,SerDataHeader.NumOfBytes16Hi,SerDataHeader.NumOfBytes16Lo,MsgTail.CrcHi,MsgTail.CrcLo = struct.unpack("!BBBBBBBB",self.ResponseMessage)

			ReadBuffer = [MsgHeader.DeviceId,MsgHeader.FunctionCode,SerDataHeader.FunctionCode,SerDataHeader.NumOfBytes,
			SerDataHeader.NumOfBytes16Hi,SerDataHeader.NumOfBytes16Lo,MsgTail.CrcHi,MsgTail.CrcLo]
			self.debug(ReadBuffer)
			if (not(self.IsOk_CRC16(ReadBuffer,8))):
				ResultText = "[Chempro100i] ERROR! CRC error!\n"
				RetCode = 4 #Failure

			else:
				#Frame CRC, size and modbus header ok.
				msg_len = SerDataHeader.NumOfBytes
				if (msg_len == 0):
					#Using 16 bit length frame
					msg_len = (SerDataHeader.NumOfBytes16Hi<<8)|(SerDataHeader.NumOfBytes16Lo)
					ResultText = "[Chempro100i] ERROR! Reply Message not match.\n"

				if (SerDataHeader.FunctionCode==159):
					if (((RespBytes != RepLen + 4) or not(VarLen)or(RespBytes < PayloadLen - 4))):
						#We didnt get bytes number in the frame header
						ResultText = "[Chempro100i] ERROR! Premature end of frame!\n"
						RetCode = 4 #Failure

					if (VarLen or (msg_len==RepLen)):
						#Frame lenght is correct.
						ResultText = "[Chempro100i] CONNECTION SUCCEED!!!\n"
						RetCode = 0 #DRR_OKE

					else:
						ResultText = "[Chempro100i] ERROR! Response length mismatch.\n"
						RetCode = 4 #Failure
		else:
			ResultText = "[Chempro100i] ERROR! Invalid Data\n"
			RetCode = 4 #Failure

		print ResultText
		return RetCode

	def startup_burst(self):
		'''send startup burst'''
		burstbuffer = []
		i = 0
		while (i<512):
			burstbuffer.insert(i,0)
			i += 1
		SerialSensor.write(burstbuffer)

	def init_connection(self):
		'''initialize serial connection raspberry pi - chempro100i'''
		global SerialSensor
		print "[Chempro100i] Initializing Connection...\n"
		validbaud = 0
		validcommspeeds=[9600,19200,28800,38400,115200]
		ConnTries = 0

		SerialSensor = SerialInterface(self.device, self.baudrate)
		SerialSensor.flush()
		time.sleep(2)

		print "[Chempro100i] Sending Startup Burst..."
		self.startup_burst()

		while ((ConnTries < 5) and (not(validbaud))):
			print "[Chempro100i] Try to connect: %d \n" % ConnTries
			ConnTries += 1
			time.sleep(0.5)

			if(self.req_baudrate()==0):
				validbaud = 1
			
			if (ConnTries==5):
				return 0 
		return 1


	def measurement(self):
		WriteBufferMeasurement = [10,5,162,6,0,0,1,0,5,166]

		ReqLen = 6 #Request Message Baud Rate Length
		RespBytes = 138 #Response Message Length
		RepLen = 134 
		VarLen = 0
		self.ResponseMessage = []

		q = 0
		while (q<32):
			DataMeasurement.GasName[q] = 0
			q += 1


		SerialSensor.flush()
		SerialSensor.write(WriteBufferMeasurement)
		SerialSensor.set_timeout(1,0.1)
		self.ResponseMessage = SerialSensor.recv(138)
		Converted = [ord(i) for i in self.ResponseMessage]

		if (len(Converted)==138):
			ReadBuffer = struct.unpack("!BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",self.ResponseMessage)
			self.debug(ReadBuffer)
			SerDataHeader.FunctionCode = ReadBuffer[2]
			SerDataHeader.NumOfBytes = ReadBuffer[3]
			SerDataHeader.NumOfBytes16Hi = ReadBuffer[4]
			SerDataHeader.NumOfBytes16Lo = ReadBuffer[5]
			DataMeasurement.Valid = ReadBuffer[8]
			if (DataMeasurement.Valid == 1):
				DataMeasurement.ConcClassification = ReadBuffer[10]
				z = 0
				while (z<32):
					DataMeasurement.GasName[z] = ReadBuffer[24+z]
					z += 1

				DataMeasurement.StateHi = ReadBuffer[124]
				DataMeasurement.StateLo = ReadBuffer[125]

			else:
				RetCode = 4
				ResultText = "[Chempro100i] ERROR! Invalid Measurement!\n"


			if (not(self.IsOk_CRC16(ReadBuffer, 138))):
				ResultText = "[Chempro100i] ERROR! CRC error!\n"
				RetCode = 4 #Failure

			else:
				#Frame CRC, size and modbus header ok.
				msg_len = SerDataHeader.NumOfBytes
				if (msg_len == 0):
					#Using 16 bit length frame
					msg_len = (SerDataHeader.NumOfBytes16Hi<<8)|(SerDataHeader.NumOfBytes16Lo)
					ResultText = "[Chempro100i] ERROR! Reply Message not match.\n"

				if (SerDataHeader.FunctionCode==162):
					if (((RespBytes != RepLen + 4) or not(VarLen)or(RespBytes < PayloadLen - 4))):
						#We didnt get bytes number in the frame header
						ResultText = "[Chempro100i] ERROR! Premature end of frame!\n"
						RetCode = 4 #Failure

					if (VarLen or (msg_len==RepLen)):
						#Frame lenght is correct.
						ResultText = "[Chempro100i] VALID MEASUREMENT!\n"
						RetCode = 0 #DRR_OKE

					else:
						ResultText = "[Chempro100i] ERROR! Response length mismatch.\n"
						RetCode = 4 #Failure

				else:
					ResultText = "[Chempro100i] ERROR! Invalid Measurement\n"
					RetCode = 4 #Failure

		else:
			ResultText = "[Chempro100i] ERROR! Invalid Measurement\n"
			RetCode = 4 #Failure

		print ResultText
		return RetCode


class PixhawkMavlink(object):
	''' Class for Mavlink Communication with Pixhawk to get GPS data '''
	def __init__(self, device, baudrate):
		self.device = device
		self.baudrate = baudrate
		f = fifo()

	def init_connection(self):
		global mav, master
		mav = mavlink.MAVLink(f,srcSystem=2,srcComponent=1, use_native=False)
		master = mavutil.mavlink_connection(self.device, self.baudrate)
		print("[Pixhawk] Waiting for HEARTBEAT..")
		master.wait_heartbeat(master)
		master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

	def measurement(self):
		m = mav1.recv_match(type='GLOBAL_POSITION_INT', blocking = True)
		if m is not None:
			print (m)
			DataGPS.Latitude = m.lat
			DataGPS.Longitude = m.lon
			DataGPS.Altitude = m.alt
		else: 
			return 0




class DataLog (object):
	''' Class for Data Logging '''
	def __init__(self):
		self.GasID = 0 
		self.Hour = 0
		self.Min = 0
		self.Sec = 0
		self.Lat = 0 
		self.Lon = 0 
		self.Alt = 0
		self.State = 0 
		self.Conc = 0
		self.timestr = time.strftime("%Y-%m-%d_%H-%M-%S")

	
	def UpdateGas(self):
		print DataMeasurement.GasName
		self.GasNameStr = [DataMeasurement.GasName[0],DataMeasurement.GasName[1],DataMeasurement.GasName[2],DataMeasurement.GasName[3],DataMeasurement.GasName[4]]
		print ("%s")%self.GasNameStr
		a = ''.join("{0}".format(n) for n in self.GasNameStr)
		GasName2 = DataMeasurement.GasName[9]
		BufferGasName = [chr(i) for i in self.GasNameStr]
		self.GasNameStr = ''.join(BufferGasName)

		print "---------"
		print self.GasNameStr
		print "---------"
		if (self.GasNameStr == "Nerve"):
			self.GasID = 1
			self.GasNameStr = "Nerve"

		elif (self.GasNameStr == "Blist"):
			self.GasID = 2
			self.GasNameStr = "Blister"

		elif (self.GasNameStr == "Blood"):
			self.GasID = 3
			self.GasNameStr = "Blood"

		elif (self.GasNameStr == "Chemi"):
			if(GasName2=='H'):
				self.GasID = 4
				self.GasNameStr = "Chemical Hazard"

		elif (a == "65991051000"):
			self.GasID = 5
			self.GasNameStr = "Acid"

		elif (self.GasNameStr == "Toxic"):
			self.GasID = 6
			self.GasNameStr = "Toxic"

		elif (self.GasNameStr == "Cyano"):
			self.GasID = 7
			self.GasNameStr = "Cyanogen chloride"

		elif (self.GasNameStr == "Vesic"):
			self.GasID = 8
			self.GasNameStr = "Vesicant precursor"

		elif (self.GasNameStr == "Flamm"):
			self.GasID = 9
			self.GasNameStr = "Flammable"

		elif (self.GasNameStr == "Organ"):
			self.GasID = 10
			self.GasNameStr = "Organic Acid"

		elif (self.GasNameStr == "Chemi"):
			if (GasName2 == 'D'):
				self.GasID = 12
				self.GasNameStr = "Chemical Detected"

		elif (a == "84687300"):
			self.GasID = 13
			self.GasNameStr = "TDI"

		elif (self.GasNameStr == "Aceto"):
			self.GasID = 14
			self.GasNameStr = "Acetonitrile"

		elif (self.GasNameStr == "Inorg"):
			self.GasID = 15
			self.GasNameStr = "Inorganic Acid"

		elif (a == '86796700'):
			self.GasID = 16
			self.GasNameStr = "VOC"

		elif (self.GasNameStr == "Cyani"):
			self.GasID = 17
			self.GasNameStr = "Cyanide"

		elif (self.GasNameStr == "Oxidi"):
			self.GasID = 18
			self.GasNameStr = "Oxidizer"

		elif (a == '00000'):
			self.GasID = 19
			self.GasNameStr = "Air"

		elif (self.GasNameStr == "Choki"):
			self.GasID = 20
			self.GasNameStr = "Choking"

		elif (a == "84736700"):
			self.GasID = 21
			self.GasNameStr = "TIC"

		elif (self.GasNameStr == "TIC o"):
			self.GasID = 22
			self.GasNameStr = "TIC Oxidizer"

		elif (self.GasNameStr == "TIC h"):
			self.GasID = 23
			self.GasNameStr = "TIC Hydride"

		elif (self.GasNameStr == "TIC a"):
			self.GasID = 24
			self.GasNameStr = "TIC Acidic"

		elif (self.GasNameStr == "TIC o"):
			self.GasID = 25
			self.GasNameStr = "TIC Organic"

		elif (self.GasNameStr == "Chlor"):
			self.GasID = 26
			self.GasNameStr = "Chlorine"

		elif (self.GasNameStr == "Ammon"):
			self.GasID = 27
			self.GasNameStr = "Ammonia"

		elif (self.GasNameStr == "Hydro"):
			if(GasName2=='s'):
				self.GasID = 28
				self.GasNameStr = "Hydrogen Sulfide"

		elif (self.GasNameStr == "Sulfu"):
			self.GasID = 29
			self.GasNameStr = "Sulfure Dioxide"

		elif (self.GasNameStr == "Hydro"):
			if (GasName2=='c'):
				self.GasID = 30
				self.GasNameStr = "Hydrogen Cyanide"
		else:
			self.GasID = 31
			self.GasNameStr = "Unrecognize Gas"
		
		return self.GasID


	def Update(self):
		print ("================== Measurement Result =========================")

		self.GasID = self.UpdateGas()
		print ("Gas Name: %s (%d)\n")% (self.GasNameStr, self.GasID)

		self.Hour = time.strftime("%H")
		self.Min = time.strftime("%M")
		self.Sec = time.strftime("%S")
		print ("Time : %s:%s:%s\n") % (self.Hour,self.Min,self.Sec)

		self.Lon = DataGPS.Longitude
		self.Lat = DataGPS.Latitude
		self.Alt = DataGPS.Altitude
		print ("Lon: %d, Lat: %d, Alt: %d\n") % (self.Lon, self.Lat, self.Alt)

		self.Conc = DataMeasurement.ConcClassification
		if (self.Conc==0):
			self.ConcStr = "Very Low"
		elif (self.Conc==1):
			self.ConcStr = "Low"
		elif (self.Conc==2):
			self.ConcStr = "Medium"
		else:
			self.ConcStr = "High"
		print ("Concentration: %d (%s)") % (DataMeasurement.ConcClassification,self.ConcStr)

		self.State = (DataMeasurement.StateHi<<8) | (DataMeasurement.StateLo)


	def Save(self):
		row = [self.Lat,self.Lon,self.Alt,self.Hour,self.Min,self.Sec,self.GasID,self.State,self.Conc]
		with open('Missionlog/log_'+self.timestr+'.csv','a') as csvfile:
			csvwriter = csv.writer(csvfile,delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
			csvwriter.writerow(row)


	def Send(self):
		self.Hour = int(self.Hour)
		self.Min = int(self.Min)
		self.Sec = int(self.Sec)
		#master.mav.adsb_vehicle_send(self.Hour,self.Min,self.Sec,self.GasID,self.Lat,self.Lon,self.Alt,self.State,self.Conc)
		master.mav.adsb_vehicle_send(0,self.Lat,self.Lon,0,self.Alt,self.Hour,self.Min,self.Sec,'a',self.GasID,self.State,self.Conc,0)

	def Debug(self):
		data_debug = mav1.recv_match(type='CRN_SENSOR')
		if data_debug is not None:
			print("CRN_SENSOR %d %d %d %d %d %d %d %d %d" % (data_debug.hh,data_debug.mm,data_debug.ss,data_debug.gas_id,data_debug.lat,data_debug.lon,data_debug.alt,data_debug.state,data_debug.intensity))
		else: 
			print ("No Data READ")
			return 0



''' 
****************************************
			Main Program
**************************************** 
'''
Sensor = Chempro100i("/dev/ttyUSB0",38400)
GPS = PixhawkMavlink("/dev/ttyACM0",57600)
Log = DataLog()
Sensor.init_connection()
GPS.init_connection()

q = 0
while (q<32):
	DataMeasurement.GasName.insert(q,0)
	q += 1

time.sleep(5)
while True:
	measurement_valid = 0
	master.mav.heartbeat_send(12, 0, 0, 0, 0)
	while(measurement_valid==0):
		measurement_valid = Sensor.measurement()
	print type(measurement_valid)
	Log.Update()
	GPSvalid = 0
	while(GPSvalid==0):
		GPSvalid = GPS.measurement()
	Log.Save()
	Log.Send()
	Log.Debug()
	time.sleep(1)
			








