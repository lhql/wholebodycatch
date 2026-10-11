#!/usr/bin/python3
"""Overlay readable, payload-specific parameters on PX4's minimal BSON seed."""
import json
import struct
import sys
from pathlib import Path
source, config, destination = map(Path, sys.argv[1:])
b=source.read_bytes();i=4;values={}
while b[i]:
    typ=b[i];i+=1;j=b.index(b'\0',i);key=b[i:j].decode();i=j+1
    fmt={1:'d',16:'i',18:'q'}[typ];value=struct.unpack_from('<'+fmt,b,i)[0];i+=struct.calcsize(fmt)
    values[key]=(typ,value)
for key,value in json.loads(config.read_text()).items():values[key]=(16 if isinstance(value,int) else 1,value)
payload=b''
for key,(typ,value) in values.items():
    payload+=bytes([typ])+key.encode()+b'\0'+struct.pack('<'+{1:'d',16:'i',18:'q'}[typ],value)
destination.write_bytes(struct.pack('<i',len(payload)+5)+payload+b'\0')
