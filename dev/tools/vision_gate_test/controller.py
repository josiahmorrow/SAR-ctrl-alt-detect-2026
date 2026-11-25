"""Minimal Webots `controller` stub so the real controller module can be
imported and exercised offline. Only the surface proposed_solution.py touches."""
import math

class _Motor:
    def __init__(self): self.velocity = 0.0
    def setPosition(self, p): pass
    def setVelocity(self, v): self.velocity = v

class _Encoder:
    def __init__(self): self.value = 0.0
    def enable(self, ts): pass
    def getValue(self): return self.value

class _Compass:
    def enable(self, ts): pass
    def getValues(self): return [0.0, 1.0, 0.0]      # heading 0 rad

class _Lidar:
    def __init__(self, n=360): self.n = n
    def enable(self, ts): pass
    def getHorizontalResolution(self): return self.n
    def getRangeImage(self): return [5.0] * self.n
    def getFov(self): return 2 * math.pi

class _Camera:
    """RGB camera returning a caller-supplied BGRA buffer."""
    def __init__(self, w=640, h=480):
        self.w, self.h, self.buf, self.fov = w, h, None, 1.047
    def enable(self, ts): pass
    def getWidth(self): return self.w
    def getHeight(self): return self.h
    def getFov(self): return self.fov
    def getImage(self): return self.buf

class _Range:
    """Depth camera returning a caller-supplied flat float list."""
    def __init__(self, w=640, h=480):
        self.w, self.h, self.buf, self.fov = w, h, None, 1.047
    def enable(self, ts): pass
    def getWidth(self): return self.w
    def getHeight(self): return self.h
    def getFov(self): return self.fov
    def getRangeImage(self): return self.buf

class _Emitter:
    def __init__(self): self.sent = []
    def send(self, payload): self.sent.append(payload)

class _Receiver:
    def enable(self, ts): pass
    def getQueueLength(self): return 0
    def getString(self): return ""
    def nextPacket(self): pass

class Robot:
    def __init__(self, name="robot1"):
        self._name, self._t = name, 0.0
        self.camera, self.depth = _Camera(), _Range()
        self.sup_emitter, self.squad_tx = _Emitter(), _Emitter()
        self._devices = {}
        for n in ("fl_wheel_joint", "fr_wheel_joint",
                  "rl_wheel_joint", "rr_wheel_joint"):
            self._devices[n] = _Motor()
        for n in ("front left wheel motor sensor", "front right wheel motor sensor",
                  "rear left wheel motor sensor", "rear right wheel motor sensor"):
            self._devices[n] = _Encoder()
        self._devices["imu compass"]            = _Compass()
        self._devices["laser"]                  = _Lidar()
        self._devices["camera rgb"]             = self.camera
        self._devices["camera depth"]           = self.depth
        self._devices["supervisor emitter"]     = self.sup_emitter
        self._devices["robot to robot emitter"] = self.squad_tx
        self._devices["robot to robot receiver"]= _Receiver()
    def getBasicTimeStep(self): return 32.0
    def getName(self): return self._name
    def getTime(self): return self._t
    def getWorldPath(self): return "/tmp/sartest/fake.wbt"
    def getDevice(self, name):
        if name not in self._devices: raise Exception("no device " + name)
        return self._devices[name]
    def step(self, ts):
        self._t += ts / 1000.0
        return 0
