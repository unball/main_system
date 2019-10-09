from abc import ABC, abstractmethod

class System():
    def __init__(self, parent):
        self.__parent = parent

    @property
    def parent(self):
        return self.__parent

    @property
    def gameThread(self):
        return self.__parent