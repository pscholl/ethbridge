from struct import Struct, pack

class OpenStruct:
        def __init__(self, **dic):
                self.__dict__.update(dic)
        def __getattr__(self, i):
                if i in self.__dict__:
                        return self.__dict__[i]
                else:
                        raise AttributeError, i
        def __setattr__(self,i,v):
                if i in self.__dict__:
                        self.__dict__[i] = v
                else:
                        self.__dict__.update({i:v})
                return v # i like cascates :)

CS_INTERFACE = 0x24

class CDC_Header(Struct, OpenStruct):
    def __init__(self,**kw):
        Struct.__init__(self, "<BBBH")
        OpenStruct.__init__(self, **kw)
        self.bDescriptorSubtype = 0x00
        self.bDescriptorType    = CS_INTERFACE

    def packself(self):
        return self.pack(self.size, self.bDescriptorType, self.bDescriptorSubtype,
                self.bcdCDC)

class CDC_Union(Struct, OpenStruct):
    def __init__(self,**kw):
        Struct.__init__(self, "<BBBBB")
        OpenStruct.__init__(self, **kw)
        self.bDescriptorSubtype = 0x06
        self.bDescriptorType    = CS_INTERFACE

    def packself(self):
        return self.pack(self.size, self.bDescriptorType, self.bDescriptorSubtype,
                self.bControlInterface, self.bSubordinateInterface0)

class CDC_ECM(Struct, OpenStruct):
    def __init__(self,**kw):
        Struct.__init__(self, "<BBBBLHHB")
        OpenStruct.__init__(self, **kw)
        self.bDescriptorSubtype = 0x0F
        self.bDescriptorType    = CS_INTERFACE

    def packself(self):
        return self.pack(self.size, self.bDescriptorType, self.bDescriptorSubtype,
                self.iMacAddress, self.bmEthernetStatistics, self.wMaxSegmentSize,
                self.wNumberMCFilters, self.bNumberPowerFilters)

CdcDevice = OpenStruct(
    bcdUSB             = 0x0110,
    bDeviceClass       = 0x00,
    bDeviceSubClass    = 0x00,
    bDeviceProtocol    = 0x00,
    bMaxPacketSize     = 64,
    idVendor           = 0x0b6a,
    idProduct          = 0x0a93,
    bcdDevice          = 0x0000,
    iManufacturer      = u"Maxim/Jennic/Teco",
    iProduct           = u"802.15.4 IPv6 ethernet bridge",
    iSerialNumber      = u"",
    #bNumConfigurations
    Configurations = [ OpenStruct(
        #wTotalLength
        #bNumInterfaces
        bConfigurationValue = 1,
        iConfiguration      = u"",
        bmAttributes        = 0x00,
        bMaxPower           = 50,
        Interfaces = [ OpenStruct(
            bInterfaceNumber    = 0,
            bAlternateSetting   = 0,
            #bNumEndpoints       = 2,
            bInterfaceClass     = 0x02,  # Commnucations - Master
            bInterfaceSubClass  = 0x06,  # Ethernet Networking
            bInterfaceProtocol  = 0x00,
            iInterface          = u"Control",
            Functions = [ CDC_Header(
                    bcdCDC                  = 0x0120),
                CDC_Union(
                    bControlInterface       = 0,
                    bSubordinateInterface0  = 1),
                CDC_ECM(
                    iMacAddress             = 1,    # XXX: this is a special index for the mac adress!!
                    bmEthernetStatistics    = 0x00,
                    wMaxSegmentSize         = 1300, # minimum mtu for ipv6 is 1280
                    wNumberMCFilters        = 0,
                    bNumberPowerFilters     = 0)
            ],
            Endpoints = [OpenStruct(
                bEndpointAddress = 0x83, # EP3-IN
                bmAttributes     = 0x03, # Interrupt
                wMaxPacketSize   = 64,
                bInterval        = 50),
            ]),
        OpenStruct(
            bInterfaceNumber    = 1,
            bAlternateSetting   = 0,     # (default interface)
            #bNumEndpoints       = 2,
            bInterfaceClass     = 0x0a,  # Commnunications - Data-Slave
            bInterfaceSubClass  = 0x00,  # Ethernet Networking
            bInterfaceProtocol  = 0x00,
            iInterface          = u"inactive",
            Endpoints           = []),
        OpenStruct(
            bInterfaceNumber    = 1,
            bAlternateSetting   = 1,
            #bNumEndpoints       = 2,
            bInterfaceClass     = 0x0a,  # Commnucations - Data-Slave
            bInterfaceSubClass  = 0x00,  # Ethernet Networking
            bInterfaceProtocol  = 0x00,
            iInterface          = u"active",
            Endpoints = [ OpenStruct(
                bEndpointAddress = 0x01,        # EP1-OUT
                bmAttributes     = 0x02,        # Bulk
                #wMaxPacketSize   = 0x800 + 64,  # 2x64 bytes
                wMaxPacketSize   = 64,  # 2x64 bytes
                bInterval        = 10), OpenStruct(
                bEndpointAddress = 0x82,        # EP2-IN
                bmAttributes     = 0x02,        # Bulk
                #wMaxPacketSize   = 0x800 + 64,  # 2x64 bytes
                wMaxPacketSize   = 64,  # 2x64 bytes
                bInterval        = 10)
            ])

        ])
    ])

DEVICE_DESCRIPTOR        = 0x01
CONFIGURATION_DESCRIPTOR = 0x02
STRING_DESCRIPTOR        = 0x03
INTERFACE_DESCRIPTOR     = 0x04
ENDPOINT_DESCRIPTOR      = 0x05

def generate(dd, name="usbdev"):
    sdd = Struct("<BBHBBBBHHHBBBB")
    scd = Struct("<BBHBBBBB")
    sid = Struct("<BBBBBBBBB")
    sed = Struct("<BBBBHB")

    # do some fancy stuff like string table generation and
    # configuration and interface numbering

    # Special Care must be taken for the string table since there two special
    # strings:
    #  stridx 0 -> language id, as per USB standard
    #  stridx 1 -> mac-address of the ethernet device per our firmware
    strtable = [u"\x0409", u"deadbeefdead"]
    def cstr(o):
        for a in [x for x in dir(o) if getattr(o,x).__class__==unicode]:
            s = getattr(o,a)
            if s not in strtable: strtable.append(s)
            setattr(o,a,strtable.index(s))

    dd.bNumConfigurations = 0
    cstr(dd)

    for c in dd.Configurations:
        dd.bNumConfigurations += 1
        c.wTotalLength         = scd.size
        c.bNumInterfaces       = 0
        cstr(c)

        for i in c.Interfaces:
            c.wTotalLength   += sid.size
            i.bNumEndpoints   = 0
            cstr(i)

            if i.bAlternateSetting==0:
                c.bNumInterfaces += 1

            if hasattr(i, 'Functions'):
                for f in i.Functions:
                    c.wTotalLength += f.size
                    cstr(f)

            for e in i.Endpoints:
                c.wTotalLength  += sed.size
                i.bNumEndpoints += 1
                cstr(e)

    # pack everything into buffers
    pdd = sdd.pack(sdd.size, DEVICE_DESCRIPTOR, dd.bcdUSB, dd.bDeviceClass,
            dd.bDeviceSubClass, dd.bDeviceProtocol, dd.bMaxPacketSize,
            dd.idVendor, dd.idProduct, dd.bcdDevice, dd.iManufacturer,
            dd.iProduct, dd.iSerialNumber, dd.bNumConfigurations)
    pcd = ""
    for c in dd.Configurations:
        pcd += scd.pack(scd.size, CONFIGURATION_DESCRIPTOR, c.wTotalLength,
                c.bNumInterfaces, c.bConfigurationValue, c.iConfiguration,
                c.bmAttributes, c.bMaxPower)
        for i in c.Interfaces:
            pcd += sid.pack(sid.size, INTERFACE_DESCRIPTOR, i.bInterfaceNumber,
                    i.bAlternateSetting, i.bNumEndpoints, i.bInterfaceClass,
                    i.bInterfaceSubClass, i.bInterfaceProtocol, i.iInterface)

            if hasattr(i, 'Functions'):
                for f in i.Functions:
                    pcd += f.packself()

            for e in i.Endpoints:
                pcd += sed.pack(sed.size, ENDPOINT_DESCRIPTOR, e.bEndpointAddress,
                        e.bmAttributes, e.wMaxPacketSize, e.bInterval)

    def strdata(s):
        d = s.encode('utf-16-le')
        if len(d)>64: raise Exception("string>64 is %i, %s"%(len(d), s))
        return [hex(ord(c)) for c in pack("<BB%is"%(len(d)), len(d)+2, STRING_DESCRIPTOR, d)]

    hdd = ", ".join([hex(ord(x)) for x in pdd])
    hcd = ", ".join([hex(ord(x)) for x in pcd])
    str = ", ".join([",".join(strdata(s))  for s in strtable])

    print """/* automatically generated by desc.py */
#ifndef __%s__
#include "usb_dev.h"

usbdev_t %s = {
/* Driver */                   { %s },
/* Default Status   */         { 0x00, 0x00, 0x01, 0x00 },
/* Device Descriptor */        { %s,
/* Configuration Descriptor */   %s,
/* String Table */               %s }
};

#endif"""%(name.upper(), name, "NULL",hdd,hcd,str)

generate(CdcDevice, "cdcdev")
