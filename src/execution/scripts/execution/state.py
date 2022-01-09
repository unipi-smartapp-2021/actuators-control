#!/usr/bin/env python

class VehicleState():
    def __init__(self, started = False, stopped = False):
        self.started = started
        self.stopped = stopped

    def is_started(self):
        return self.started

    def is_stopped(self):
        return self.stopped

    def set_started(self, started):
        self.started = started

    def set_stopped(self, stopped):
        self.stopped = stopped
