import socket
import sys
import math
# from mathutils import Matrix
import numpy as np
from numpy import matrix
import time
from xml.dom import minidom
import os.path
import signal
import xml.etree.ElementTree as ET

# Ini file handler
import configparser
debug=False

SCRIPT_ROOT = os.path.dirname(os.path.realpath(__file__))
configfile = SCRIPT_ROOT + '/' + 'Config.ini'

# Globals
port = 64444
ip = "127.0.0.1"


class ConfigIni:
    def __init__(self, filename):
        self.filename = filename
        self.config = configparser.ConfigParser()
        self.config.read(filename)

    # Assuming key is given by section.keyname
    def getIniValue(self, key):
        tokens = key.split(".")
        if (len(tokens) < 2):
            return ""
        return self.config[tokens[0]][tokens[1]]

    def getIniList(self, key):
        csv = self.getIniValue(key)
        if not csv:
            return []
        return csv.split(",")

    def getIniSection(self, section, option):
        return self.config.get(section, option)


class CrclClientSocket:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.stopconnecting = False
        # quit when find </CRCLStatus>
        self.End = '</CRCLStatus>'
        self.nextdata = ''

    def connect(self):
        try:
            if self.stopconnecting:
                return
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
        except:
            print('Failed to create socket. ')  # Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1])
            time.sleep(5)
            self.connect()

    def disconnect(self):
        self.sock.close()

    def syncsend(self, msg):
        if(debug):
            print(msg)
        sent = self.sock.send( msg.encode('utf-8'))
        if sent == 0:
            self.disconnect()
            self.connect()
            # raise RuntimeError("socket connection broken")      

    # http://code.activestate.com/recipes/408859/

    def syncreceive(self, end):
        # total_data=[];
        self.End = end  # '</CRCLStatus>'
        data = ''
        alldata = self.nextdata
        while True:
            data = self.sock.recv(8192)
            if data == 0:
                alldata = ''  # empty string
                return
            msg = data.decode()
            alldata = alldata + msg
            if self.End in alldata:
                alldata = alldata[:alldata.find(self.End)+len(self.End)]
                self.nextdata = msg[msg.find(self.End)  +len(self.End):]
                break
        return alldata.strip()  # ''.join(total_data)


def CrclActuateJoints(cmd, num, pos, vel, acc):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
  <CRCLCommand xsi:type="ActuateJointsType">
    <CommandID>{}</CommandID>
    <ActuateJoint>
      <JointNumber>{}</JointNumber>
      <JointPosition>{}</JointPosition>
      <JointDetails xsi:type="JointSpeedAccelType">
        <JointSpeed>{}</JointSpeed>
        <JointAccel>{}</JointAccel>
      </JointDetails>
    </ActuateJoint>
  </CRCLCommand>
</CRCLCommandInstance>'''.format(cmd, num, pos, vel, acc)


def CrclMoveTo(cmd, x, y, z, xi, xj, xk, zi, zj, zk):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
  <CRCLCommand xsi:type="MoveToType">
    <CommandID>{}</CommandID>
    <MoveStraight>true</MoveStraight>
    <EndPosition>
      <Point>
        <X>{}</X> <Y>{}</Y> <Z>{}</Z>
      </Point>
      <XAxis>
        <I>{}</I> <J>{}</J> <K>{}</K>
      </XAxis>
      <ZAxis>
        <I>{}</I> <J>{}</J> <K>{}</K>
      </ZAxis>
    </EndPosition>
  </CRCLCommand>
</CRCLCommandInstance>'''.format(cmd, x, y, z, xi, xj, xk, zi, zj, zk)


def CrclGetStatusCmd(cmd):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
  <CRCLCommand xsi:type="GetStatusType">
     <CommandID>{}</CommandID>
  </CRCLCommand>
</CRCLCommandInstance>
'''.format(cmd)


def CrclInitCanonCmd(cmd):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
  <CRCLCommand xsi:type="InitCanonType">
     <CommandID>{}</CommandID>
  </CRCLCommand>
</CRCLCommandInstance>
'''.format(cmd)


def CrclDwellCmd(cmd, dwell):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
    <CRCLCommand xsi:type="DwellType">
        <CommandID>{}</CommandID>
        <DwellTime>{}</DwellTime>
    </CRCLCommand>
</CRCLCommandInstance>
'''.format(cmd, dwell)


def CrclCloseToolCmd(cmd):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
    <CRCLCommand xsi:type="CloseToolChangerType">
        <CommandID>{}</CommandID>
    </CRCLCommand>
</CRCLCommandInstance>
'''.format(cmd)


def CrclOpenToolCmd(cmd):
    return '''<?xml version="1.0" encoding="UTF-8"?>
<CRCLCommandInstance
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="../xmlSchemas/CRCLCommandInstance.xsd">
    <CRCLCommand xsi:type="OpenToolChangerType">
        <CommandID>{}</CommandID>
    </CRCLCommand>
</CRCLCommandInstance>
'''.format(cmd)


def parseStatus(str):
    status = ''
    xmldoc = minidom.parseString(str)
    jointlist = xmldoc.getElementsByTagName('JointPosition')
    # assuming in order?
    for s in jointlist:
        status = status + (s.childNodes[0].nodeValue) + ":"
    x = xmldoc.getElementsByTagName("X")[0].childNodes[0].nodeValue
    y = xmldoc.getElementsByTagName("Y")[0].childNodes[0].nodeValue
    z = xmldoc.getElementsByTagName("Z")[0].childNodes[0].nodeValue
    xi = xmldoc.getElementsByTagName("XAxis")[0].getElementsByTagName("I")[0].childNodes[0].nodeValue
    xj = xmldoc.getElementsByTagName("XAxis")[0].getElementsByTagName("J")[0].childNodes[0].nodeValue
    xk = xmldoc.getElementsByTagName("XAxis")[0].getElementsByTagName("K")[0].childNodes[0].nodeValue
    zi = xmldoc.getElementsByTagName("ZAxis")[0].getElementsByTagName("I")[0].childNodes[0].nodeValue
    zj = xmldoc.getElementsByTagName("ZAxis")[0].getElementsByTagName("J")[0].childNodes[0].nodeValue
    zk = xmldoc.getElementsByTagName("ZAxis")[0].getElementsByTagName("K")[0].childNodes[0].nodeValue
    status = status + '\n' + '(' + x + "," + y + "," + z + '),' + '(' + xi + "," + xj + "," + xk + '),' + '(' + zi + "," + zj + "," + zk + ')' + '\n'
    return status


def parseInstances(msg):
    try:
        msg=msg.strip()
        str=''
        #msg=msg.encode('utf-8')
        #print(msg)
        #xmldoc = ET.fromstring(msg)
        xmldoc = minidom.parseString(msg.encode( 'utf-8'))
        instancelist = xmldoc.getElementsByTagName('ModelStatus')
        # assuming in order?
        for s in instancelist:
            str += s.getElementsByTagName("Name")[0].firstChild.nodeValue +","
            str+= s.getElementsByTagName("Pose")[0].getElementsByTagName("Point")[0].getElementsByTagName("X")[0].firstChild.nodeValue+","
            str+= s.getElementsByTagName("Pose")[0].getElementsByTagName("Point")[0].getElementsByTagName("Y")[0].firstChild.nodeValue + ","
            str+= s.getElementsByTagName("Pose")[0].getElementsByTagName("Point")[0].getElementsByTagName("Z")[0].firstChild.nodeValue + ","
            str+="\n"
        print(str)
    except Exception as e:
        print("exception parsing status Crcl XML", e)
    return str


def x_rot(rads):
    return np.matrix([
        [1, 0, 0],
        [0, np.cos(rads), -np.sin(rads)],
        [0, np.sin(rads), np.cos(rads)]])


def y_rot(rads):
    return np.matrix([
        [np.cos(rads), 0, np.sin(rads)],
        [0, 1, 0],
        [-np.sin(rads), 0, np.cos(rads)]])


def z_rot(rads):
    return np.matrix([
        [np.cos(rads), -np.sin(rads), 0],
        [np.sin(rads), np.cos(rads), 0],
        [0, 0, 1]])


def rot_max(yaw, pitch, roll):
    # m=np.matrix([[1, 0,0], [0,1,0], [0,0,1]])
    yaw = math.radians(yaw)
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    return z_rot(yaw) * y_rot(pitch) * x_rot(roll)


# for line in sys.stdin:

# line = 'r "/usr/local/michalos/nistfanuc_ws/src/nist_fanuc/doc/fanuclrmateprogram.xml"'
# line = 'g 0.465 0 0.695 -180.0 -90 0'
# j 1 1.7 0 0

#    line=sys.stdin.readline()
# def parseline(line):
#     global cmd
#     if not line.strip():
#         return
#
#     tokens = line.split()
#     if (len(tokens) == 0):
#         return
#     if tokens[0] == 'r':
#         print("r \"program path\"")
#         line = tokens[1].strip('\"')
#         line = line[0:].strip()
#     if not os.path.isfile(line):
#         print("Not a file name")
#         return
#     print(line)
#     try:
#         contents = open(line, 'r').read()
#     except IOError:
#         print("Bad file name", line)
#     # add try catch bad file
#     mysocket.syncsend(contents)
#
#     elif tokens[0] == 'g':
#         print("g x y z r p y - in degrees")
#         m = rot_max(float(tokens[6]), float(tokens[5]), float(tokens[4]))
#         xi = m.item(0, 0);
#         xj = m.item(1, 0);
#         xk = m.item(2, 0)
#         zi = m.item(0, 2);
#         zj = m.item(1, 2);
#         zk = m.item(2, 2)
#         print(xi, ":", xj, ":", xk)
#         print(zi, ":", zj, ":", zk)
#         s = CrclMoveTo(str(cmd), str(tokens[1]), str(tokens[2]), str(tokens[3]), str(xi), str(xj), str(xk), str(zi),
#                        str(zj), str(zk))
#         mysocket.syncsend(s)
#     elif tokens[0] == 'j':
#         print("j # pos vel acc - in degrees")
#         print(tokens[0], " ", tokens[1], " ", tokens[2])
#         # s=CrclActuateJoints(str(cmd), tokens[1], str(math.radians(float(tokens[2]))), 4.0, 1.0)
#         s = CrclActuateJoints(str(cmd), tokens[1], tokens[2], 4.0, 1.0)
#         mysocket.syncsend(s)
#     elif tokens[0] == 'o':
#         s = CrclOpenToolCmd(str(cmd))
#         mysocket.syncsend(s)
#     elif tokens[0] == 'c':
#         s = CrclCloseToolCmd(str(cmd))
#         mysocket.syncsend(s)
#     elif line[0:0] == 'd':
#         print("d n - dwell in seconds")
#         s = CrclDwellCmd(str(cmd), tokens[1])
#         mysocket.syncsend(s)
#     elif line[0:0] == 's':
#         print("s - status")
#         s = CrclGetStatusCmd(str(cmd))
#         mysocket.syncsend(s)
#         # wait for respose
#         resp = mysocket.syncreceive('</CRCLStatus>')
#         status = parseStatus(resp)
#         print(status)
#     else:
#         print("do not dispair, danger is not imminent")
#     cmd = cmd + 1




cmd = 1

# config=ConfigIni(configfile)
# ip=config.getIniValue("GLOBALS.ip")
# port=int(config.getIniValue("GLOBALS.port"))
# program=config.getIniList("GLOBALS.program")

# Connect to CRCL server
mysocket = CrclClientSocket(ip, port)
mysocket.connect()

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    mysocket.disconnect()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Run through program
# for x in program[:]:
#     str=config.getIniSection(x,"1")
#     print str
#     mysocket.syncsend(str)
#     time.sleep(10)

i = 0
str=CrclInitCanonCmd(i)
mysocket.syncsend(str)
i = i + 1
while True:
    str = CrclGetStatusCmd(i)
    mysocket.syncsend(str)
    i = i + 1
    time.sleep(5)
    str = mysocket.syncreceive('</CRCLStatus>')
    parseInstances(str)
    #print(str)

mysocket.disconnect()

# time.sleep(10)
# mywait=1
# mysocket = CrclClientSocket(ip, port)
# print 'Socket Created'
# mysocket.connect()
# print 'Socket connected'
# while 1:
# 	parseline("j 1 90 0 0")
# 	time.sleep(mywait)
# 	parseline("c");
# #	parseline("d 5")
# 	time.sleep(mywait+2)
# 	parseline("j 1 0.0 0 0")
# 	time.sleep(mywait)
# 	parseline("j 1 -90 0 0")
# 	time.sleep(mywait+2)
#  	parseline("o");
#  	time.sleep(mywait+2)
# mysocket.disconnect()
