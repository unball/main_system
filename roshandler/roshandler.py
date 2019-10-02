import subprocess
import time
import rosgraph
import gui.singleton

class Process():
    def __init__(self, command, aliveChecker = None):
        self.__command = command
        self.__process = None
        self.__aliveChecker = aliveChecker

    def run(self):
        # Não faz nada se o processo estiver rodando
        if self.__aliveChecker is not None:
            if self.__aliveChecker():
                return
        
        # Se não estiver definido cria um novo subprocesso
        if self.__process is None:
            self.__process = subprocess.Popen(self.__command.split(" "))
        
        # Se o processo terminou tenta abrir novamente
        while self.__process.poll() is not None:
            print("\"" + self.__command + "\" não está rodando, tentando abrir em 5 segundos...")
            self.__process = subprocess.Popen(command.split(" "))
            time.sleep(5)
    
    def terminate(self):
        if self.__process is None:
            return
        self.__process.terminate()

        # Wait until process really terminates
        while(self.__process.poll() is not None): time.sleep(1)

        
        self.__process = None

def roscorechecker():
    try:
        rosgraph.Master("/rostopic").getPid()
        return True
    except:
        return False

class RosHandler(metaclass=gui.singleton.Singleton):
    def __init__(self):
        self.__processes = {
            "radioSerial": Process("rosrun rosserial_arduino serial_node.py /dev/ttyUSB0 _baud:=115200"),
            "roscore": Process("roscore", aliveChecker=roscorechecker)
        }

    def runProcess(self, processKey):
        self.__processes[processKey].run()
    
    def terminateAll(self):
        for _,process in self.__processes.items():
            process.terminate()