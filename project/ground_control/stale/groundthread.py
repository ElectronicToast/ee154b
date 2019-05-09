import threading 
import ctypes 
from time import sleep
from datetime import datetime
import sys

class GroundThread(threading.Thread): 
    def __init__(self, t, *args): 
        threading.Thread.__init__(self, target=t, args=args)
        self.target = t

    def run(self): 
  
        # target function of the thread class 
        try: 
            self.target()
        finally: 
            print('ended') 
            sys.stdout.flush()

    def get_id(self): 
  
        # returns id of the respective thread 
        if hasattr(self, '_thread_id'): 
            return self._thread_id 
        for id, thread in threading._active.items(): 
            if thread is self: 
                return id
   
    def raise_exception(self): 
        thread_id = self.get_id() 
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 
              ctypes.py_object(SystemExit)) 
        if res > 1: 
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0) 
            print('Exception raise failure')

def test():
    last = datetime.now()
    while True:
        if (datetime.now() - last).seconds > 2:
            i.raise_exception()
            i.join()
            print('hi')
            sys.stdout.flush()

            # sleep(1)
            last = datetime.now()
            # i.start()


def inputfunc():
    while True:
        print(input(">> "))

global t
global i

t = GroundThread(test)
i = GroundThread(inputfunc)
t.start()
i.start()
# time.sleep(1)
# t.raise_exception()
# t.join()