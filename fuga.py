import matplotlib.pyplot as plt
import serial
import numpy as np
import time


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

    def abrir_porta(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Porta {self.port} aberta com sucesso.")
        except serial.SerialException as e:
            print(f"Erro ao abrir porta serial: {e}")

    def ler_dados(self):
        if self.serial and self.serial.is_open:
            try:
                return self.serial.readline()
            except serial.SerialTimeoutException as e:
                print(f"Erro ao ler dados da porta serial: {e}")
        else:
            print("A porta serial não está aberta ou não foi inicializada corretamente.")
            return None

    def fechar_porta(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Porta serial fechada com sucesso.")
        else:
            print("A porta serial não está aberta ou não foi inicializada corretamente.")


class Parametros:
    def __init__(self, f=60, t=1, n=100, limit=0.1):
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
    micro.abrir_porta()

    fuga = FugaDados()
    print(f"Tensão: {fuga.voltage} V")
    print(f"Frequência: {fuga.frequency} Hz")
    print(f"Amperes: {fuga.amps} A")
    print(f"Resistência do Circuito: {fuga.R} Ohms")
    print(f"Potência Nominal: {fuga.P} Watts")

    sensor = SensorCorrente()
    print(f"Sensibilidade: {sensor.sensitivity}")
    print(f"Offset: {sensor.offset}")
    print(f"Ganho: {sensor.gain}")

    parametros = Parametros()

    while True:
        try:
            line = micro.ler_dados().decode('utf-8').strip()
            if line:
                dado = float(line)
                if len(dados) == parametros.n:
                    dados = np.array(dados)
                    i = (dados - sensor.offset) / (sensor.sensitivity * sensor.gain)
                    p_med = np.mean(i * fuga.voltage)
                    dif = p_med - fuga.P
                    porctg = dif / fuga.P * 100

                    if abs(porctg) >= parametros.limit:
                        print(f"ATENÇÃO: Vazamento de energia detectado! {dif:.2f} W ({porctg:.2f} %)")
                    else:
                        print(f"OK: Rede elétrica normal. {dif:.2f} W ({porctg:.2f} %)")
        except KeyboardInterrupt:
            print("\nInterrupção do teclado. Fechando o programa.")
            micro.fechar_porta()

            t = np.linspace(0, 2, 10)
            i = np.random.rand(10)
            plt.plot(t, i)
            plt.xlabel("Tempo (s)")
            plt.ylabel("Corrente (A)")
            plt.title("Corrente Elétrica na Rede")
            plt.show()
            time.sleep(1)
            break


if __name__ == "__main__":
    main()
