#!/usr/bin/python2.7
#
# File: Main.py
# Author: Paul Buzaud
#
# Created:
#

import subprocess
import sys
import signal
import psutil

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    """Make sure that all the child process of the roscore are killed before his termination"""
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        process.send_signal(sig)

class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False
    def __init__(self):
        """Check if one roscore node have already been created"""
        if Roscore.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        Roscore.__initialized = True

    def run(self):
        """Run the roscore"""
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            sys.stderr.write('roscore could not be run')
            raise e

    def terminate(self):
        """Terminate the roscore process and his child"""
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False

