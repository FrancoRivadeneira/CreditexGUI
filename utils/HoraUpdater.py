from PyQt5.QtCore import QThread, pyqtSignal
import time

class HoraUpdater(QThread):
    _SIGNAL_data = pyqtSignal(str)  # Señal para actualizar la hora en la UI
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
    def run(self):
        """Ejecuta el hilo y emite la hora actual cada segundo."""
        while True:
            hora = time.strftime("%H:%M")  # Obtener la hora en formato HH:MM:SS
            # print(hora)  # Solo para depuración
            hora = f"Hora: {hora}"
            self._SIGNAL_data.emit(hora)  # Emite la nueva hora
            time.sleep(1)  # Espera 1 segundo antes de actualizar nuevamente

