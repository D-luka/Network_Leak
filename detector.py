import matplotlib.pyplot as plt
import serial
import numpy as np
import time
from sympy.abc import n


port = "/dev/ttyS0"
baudrate = 9600
timeout = 1


class FugaDados:
    def __init__(self, voltage=220, frequency=60, amps=10):
        self.voltage = voltage
        self.frequency = frequency
        self.amps = amps
        self.R = self.voltage / self.amps
        self.P = self.voltage * self.amps


class SensorCorrente:
    def __init__(self, sensitivity=0.1, offset=2.5, gain=5):
        self.sensitivity = sensitivity
        self.offset = offset
        self.gain = gain


class MicroControlador:
    def __init__(self, port="/dev/ttyS0", baudrate=9600, timeout=1):
        self.serial = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

    # Read port: ls /dev/tty*
    def abrir_port(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Port {self.port} Opened successfully.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")

    def read_dados(self):
        if self.serial and self.serial.is_open:
            try:
                data = self.serial.readline()
                return data
            except serial.SerialTimeoutException as e:
                print(f"Error ao ler dadops da porta serial: {e}")
                return None
        else:
            print("The serial port is not open or has not been initialized correctly")
            return None

    def fechar_port(self):
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
                print("Porta serial fechada com sucesso!")
            except serial.SerialException as e:
                print(f"Error ao fechar porta serial: {e}")
        else:
            print("Porta serial não está aberta ou não foi inicializada corretamente.")
        #self.serial.close()
        #print(f"Port {self.port} Closed successfully.")

    #def read_dados(self):
    #   dados = self.serial.readline()
    #   return dados


class Parametros:
    def __init__(self, f=60,  t=1, n=100, limit=0.1):
        self.f = f
        self.t = t
        self.n = n
        self.limit = limit
        self.t = self.calc_t()
        self.dt = self.calc_dt()

    def calc_t(self):
        return self.t / self.f

    def calc_dt(self):
        return self.t / self.n


def main():
    porta_serial = '/dev/ttyS0'
    baudrate = 9600
    timeout = 1

    micro = MicroControlador()
    micro.abrir_port()

    try:
        be = serial.Serial(porta_serial, baudrate, timeout=timeout)
    except serial.SerialException as e:
        print(f"Erro ao abrir a porta serial: {e}")
    fuga = FugaDados()
    print(f"Voltage: {fuga.voltage} V")
    print(f"Frequency: {fuga.frequency} Hz")
    print(f"Amps: {fuga.amps} A")
    print(f"Circuit Resistence: {fuga.R} Ohms ")
    print(f"Rated Power {fuga.P} Watts")

    sensor = SensorCorrente()
    print(f"Sensitivity: {sensor.sensitivity}")
    print(f"Offset: {sensor.offset}")
    print(f"Gain: {sensor.gain}")



    #dados = micro.read_dados()
    #print(f"Read data: {dados}")
    #micro.fechar_port()

    parametros = Parametros()

    #print(f"T: {parametros.t}")
    #print(f"dt: {parametros.dt}")

    #be = serial.Serial(port, baudrate, timeout=timeout)
    dados = []

    while True:
        try:
            number = input(" Enter number: ")
            line = micro.read_dados().decode('utf-8').strip()
            if line:
                dado = float(line)
                line.append(dado)
                if len(dados) == n:
                    dados = np.array(dados)
                    i = (dados - sensor.offset) / (sensor.sensitivity * sensor.gain)
                    p_med = np.mean(i * fuga.voltage)
                    dif = p_med - fuga.P
                    porctg = dif / fuga.P * 100

                    if abs(porctg) >= parametros.limit:
                        print(f"ATTENTION: Energy leakage detected! {dif:.2f} W ({porctg:.2f} %)")
                    else:
                        print(f"OK: Normal electrical network. {dif:.2f} W ({porctg:.2f} %)")
        finally:

                #Code to be executed when a keyboard interrupt is detected
                #print("\nKeyboard locked outage. Closing the program.")
                #micro.fechar_port()

                t = np.linspace(0, 2, 10)
                i = np.random.rand(10)
                plt.plot(t, i)
                plt.xlabel("Time (s)")
                plt.ylabel("Current (A)")
                plt.title("Electric current in the network")
                plt.show()
                dados = []
                time.sleep(1)


if __name__ == "__main__":
    main()
    #porta = '/dev/ttyS0'  # Substitua pela porta serial que você está usando
    #baudrate = 9600  # Substitua pela taxa de transmissão correta

    #micro_instancia = MicroControlador(porta, baudrate)
    #micro_instancia.abrir_port()

    #dados = micro_instancia.read_dados()
    #if dados is not None:
    #    try:
    #        line = dados.decode('utf-8').strip()
    #        print("Dados lidos da porta serial:", line)
    #    except UnicodeDecodeError as e:
    #        print("Erro ao decodificar os dados:", e)

    #micro_instancia.fechar_port()  # Não se esqueça de fechar a porta serial após o uso
