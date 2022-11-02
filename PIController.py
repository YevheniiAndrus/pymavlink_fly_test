import time

class PIController:
    def __init__(self, kP, kI):
        self._kP = kP
        self._kI = kI
        self._last_time = None
        self._I = 0
        self._P = 0
        self._total = 0
        self._counter = 0
        self._target = 0
        self._current = 0
        self._complete = False
    
    def complete(self) -> bool:
        return self._complete


    def update(self, target, current):
        err = target - current
        if abs(err) <= 1.0:
            self._complete = True
        else:
            self._complete = False
            
        return self._kP * err
