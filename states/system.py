from abc import ABC, abstractmethod

class System():
    def __init__(self, parent):
        self.__parent = parent