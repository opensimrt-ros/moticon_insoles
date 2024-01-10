#!/usr/bin/env python3

print(f"Loaded {__file__}")

from abc import ABC, abstractmethod, abstractproperty

class InsoleDataGetter(ABC):
    """Abstract class to interface between reading from file or sensor"""
    
    start_time = None
    start_frame = [None, None]

    @abstractmethod
    def set_start_time(self):
        pass

    @abstractproperty
    def ok(self):
        pass
    
    @abstractmethod
    def start_listening(self):
        pass
    @abstractmethod
    def get_data(self):
        pass
    @abstractmethod
    def close(self):
        pass

