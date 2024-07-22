import threading
import ctypes
 
class StoppableThread(threading.Thread):
    def stop(self):
        thread_id = threading.get_ident(self)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
              ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print('Exception raise failure')