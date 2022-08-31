import ctypes
import os

from certifi import contents

if __name__ == "__main__":
  # Load the shared library into ctypes
  libname = os.getcwd() + "/lib/libXodrBuilder.so"
  xodrLib = ctypes.cdll.LoadLibrary(libname)
  getXodrFun = xodrLib.getXodrBuilder
  getXodrFun.argtypes = [ctypes.POINTER(ctypes.c_char), ctypes.c_double]
  getXodrFun.restype = ctypes.POINTER(ctypes.c_double)
  xodrpath = "/home/kbrezhnyev/CARLA_0.9.12/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr".encode('utf-8')
  xb = getXodrFun(xodrpath, 1.0)
  
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
  # Attention! May print a lot of stuff here!
  for i in points.contents: print(i)
  