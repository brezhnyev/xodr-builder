import ctypes
import os
import sys

from certifi import contents

def process(xodrpath):
  # Load the shared library into ctypes
  libname = os.getcwd() + "/../../lib/libXodrBuilder.so"
  xodrLib = ctypes.cdll.LoadLibrary(libname)
  getXodrFun = xodrLib.getXodrBuilder
  getXodrFun.argtypes = [ctypes.POINTER(ctypes.c_char), ctypes.c_double]
  getXodrFun.restype = ctypes.POINTER(ctypes.c_double)
  xb = getXodrFun(xodrpath[0].encode('utf-8'), 1.0)
  
  getNofPfun = xodrLib.getNumberOfPoints
  getNofPfun.argtypes = [ctypes.c_void_p]
  getNofPfun.restype = ctypes.c_longlong
  pointsN = getNofPfun(xb)
  print(pointsN)

  getPointsFun = xodrLib.getAllPoints
  getPointsFun.argtypes = [ctypes.c_void_p]
  pn = pointsN*4
  getPointsFun.restype = ctypes.POINTER(ctypes.c_double * pn)
  points = getPointsFun(xb)
  for p,c in zip(points.contents, range(pn)):
    print(p)
    if (c > 16):
      break

if __name__ == "__main__":
  xodrpath = sys.argv[1:]
  process(xodrpath)
  