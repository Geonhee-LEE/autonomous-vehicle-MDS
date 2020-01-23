#!/usr/bin/python -u
"""
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.
"""

import socket
import sys
import datetime
import serial
import base64
import time
import os
#import ssl
from optparse import OptionParser
from multiprocessing import Process, Value


version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=2 # How much the sleep time increases with each failed attempt
maxReconnect=2

sleepTime=1 # So the first one is 1 second
maxConnectTime=0

class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=None,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=True,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=None,
                 headerOutput=False,
                 ):

        maxReconnectTime=1200
        maxConnectTime=0
        
        self.buffer=buffer
        self.user=user
        self.out=out
        self.port=port
        self.caster=caster
        self.mountpoint=mountpoint
        self.setPosition(lat, lon)
        self.height=height
        self.verbose=verbose
        self.ssl=ssl
        self.host=host
        self.UDP_Port=UDP_Port
        self.V2=V2
        self.headerFile=headerFile
        self.headerOutput=headerOutput
        self.maxConnectTime=maxConnectTime
        self.IsConnect = False
        self.maxReconnectTime=maxReconnectTime
        self.maxReconnect=maxReconnect
        self.socket=None
        
        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket=None
        

    def init_for_ntrip(self):
        self.headerFile=sys.stderr
        self.out=sys.stdout
        #self.ser = serial.serial_for_url(self.ser_add,57600, timeout=0)

    def setPosition(self, lat, lon):
        self.flagN="N"
        self.flagE="E"
        if lon>180:
            lon=(lon-360)*-1
            self.flagE="W"
        elif (lon<0 and lon>= -180):
            lon=lon*-1
            self.flagE="W"
        elif lon<-180:
            lon=lon+360
            self.flagE="E"
        else:
            self.lon=lon
        if lat<0:
            lat=lat*-1
            self.flagN="S"
        self.lonDeg=int(lon)
        self.latDeg=int(lat)
        self.lonMin=(lon-self.lonDeg)*60
        self.latMin=(lat-self.latDeg)*60
        

    def getMountPointString(self):
        server = self.caster
        port =self.port
        
        mountpoint = self.mountpoint

        #'192.168.xxx.xxx',#Port,"#UserName","#Password",'#MP'

        pwd = self.user

        header =\
        "GET /{} HTTP/1.1\r\n".format(mountpoint) +\
        "Host \r\n".format(server) +\
        "Ntrip-Version: Ntrip/2.0\r\n" +\
        "User-Agent: NTRIP pyUblox/0.0\r\n" +\
        "Connection: close\r\n" +\
        "Authorization: Basic {}\r\n\r\n".format((base64.b64encode(pwd.encode('ascii'))).decode('ascii'))
        return header

        

    def getGGAString(self):
        now = datetime.datetime.utcnow()
        ggaString= "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % \
            (now.hour,now.minute,now.second,self.latDeg,self.latMin,self.flagN,self.lonDeg,self.lonMin,self.flagE,self.height)
        checksum = self.calcultateCheckSum(ggaString)
        if self.verbose:
            print ( "$%s*%s\r\n" % (ggaString, checksum))
        return "$%s*%s\r\n" % (ggaString, checksum)

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def connectToServer(self):
        reconnectTry=1
        sleepTime=1
        reconnectTime=0
        if self.maxConnectTime > 0 :
            EndConnect=datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry<=self.maxReconnect:
                found_header=False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry,self.maxReconnect))

                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                if self.ssl:
                    self.socket=ssl.wrap_socket(self.socket)

                error_indicator = self.socket.connect_ex((self.caster, self.port))
                if error_indicator==0:
                    sleepTime = 1
                    connectTime=datetime.datetime.now()

                    self.socket.settimeout(10)
                    
                    self.socket.send(self.getMountPointString().encode('ascii'))
                    while not found_header:
                        casterResponse=self.socket.recv(4096) #All the data
                        #print(casterResponse)
                        header_lines = casterResponse.split("\r\n".encode('ascii'))
                        if self.IsConnect == False:
                            
                            for line in header_lines:
                                if line=="":
                                    if not found_header:
                                        found_header=True
                                        if self.verbose:
                                            sys.stderr.write("End Of Header"+"\n")
                                else:
                                    if self.verbose:
                                        sys.stderr.write("Header: " + str(line) +"\n")
                                if self.headerOutput:
                                    self.headerFile.write(line+"\n")

                            for line in header_lines:
                                if line.find("SOURCETABLE".encode('ascii'))>=0:
                                    sys.stderr.write("Mount point does not exist")
                                    sys.exit(1)
                                elif line.find("401 Unauthorized".encode('ascii'))>=0:
                                    sys.stderr.write("Unauthorized request\n")
                                    sys.exit(1)
                                elif line.find("404 Not Found".encode('ascii'))>=0:
                                    sys.stderr.write("Mount Point does not exist\n")
                                    sys.exit(2)
                                elif line.find("ICY 200 OK".encode('ascii'))>=0:
                                    #Request was valid
                                    self.socket.sendall(self.getGGAString().encode('ascii'))
                                    self.IsConnect = True
                                    return
                                elif line.find("HTTP/1.0 200 OK".encode('ascii'))>=0:
                                    #Request was valid
                                    self.socket.sendall(self.getGGAString().encode('ascii'))
                                    self.IsConnect = True
                                    return
                                elif line.find("HTTP/1.1 200 OK".encode('ascii'))>=0:
                                    #Request was valid
                                    self.socket.sendall(self.getGGAString().encode('ascii'))
                                    self.IsConnect = True
                                    return

                    
                    if self.verbose:
                        sys.stderr.write('Closing Connection\n')
                    self.socket.close()
                    self.socket=None

                    if reconnectTry < self.maxReconnect :
                        sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= 2

                        if sleepTime>self.maxReconnectTime:
                            sleepTime=self.maxReconnectTime

                    reconnectTry += 1
                else:
                    self.socket=None
                    if self.verbose:
                        print ("Error indicator: ", error_indicator)

                    if reconnectTry < self.maxReconnect :
                        sys.stderr.write( "%s No Connection to NtripCaster.  Trying again in %i seconds\n" % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime *= 2
                        if sleepTime>self.maxReconnectTime:
                            sleepTime=self.maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()
            sys.exit()
            
    def readData(self):
        
        data = "Initial data"
        try:
            data=self.socket.recv(self.buffer)
            #self.out.write(str(data))
            if self.UDP_socket:
                self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))
#                    print datetime.datetime.now()-connectTime
            if maxConnectTime :
                if datetime.datetime.now() > connectTime+EndConnect:
                    if self.verbose:
                        sys.stderr.write("Connection Timed exceeded\n")
                    sys.exit(0)
        except socket.timeout:
            if self.verbose:
                sys.stderr.write('Connection TimedOut\n')
            data=False
        except socket.error:
            if self.verbose:
                sys.stderr.write('Connection Error\n')
            data=False

        self.IsConnect = False
        return data

    def update_RTK(self, q ,q_2):
        self.init_for_ntrip()
        self.connectToServer()
        print("RTK : {}".format(os.getpid()))
        while True:
            data = self.readData()
            #print >>sys.stderr, [ord(d) for d in data]
            sys.stdout.flush()
            q.put([data])
            
            if q_2.empty()==False:
                data_pos = q_2.get()
                lat = data_pos[0]
                lon = data_pos[1]
                self.setPosition(lat,lon)
            #print("setting lat: {}, lon: {}".format(lat,lon))

'''


    def getMountPointString(self):
        mountPointString = "GET %s HTTP/1.1\r\n" % (self.mountpoint)
#        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\n" % (self.mountpoint, useragent)
        if self.host or self.V2:
           hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
           mountPointString+=hostString
        if self.V2:
           mountPointString+="Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString+="User-Agent: {}\r\n".format(useragent)
        mountPointString+="Connection: close\r\n"
        mountPointString+="Authorization: Basic {}\r\n".format(self.user.decode("ascii"))
        mountPointString+="\r\n"
        if self.verbose:
           print (mountPointString)
        return mountPointString
'''
