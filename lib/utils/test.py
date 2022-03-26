import pathlib
from ctypes import *

libname = str(pathlib.Path().absolute() / "crc16.so")
c_lib = CDLL(libname, mode=RTLD_GLOBAL)

c_lib.crc16.argtypes = [POINTER(c_ubyte), c_uint16]
c_lib.crc16.restype = c_uint16

data_block = (c_ubyte * 256)(1)

pointer = cast(addressof(data_block), POINTER(c_ubyte))

print(c_lib.crc16(pointer, 256))
