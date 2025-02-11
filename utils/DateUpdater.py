from PyQt5.QtCore import QThread, pyqtSignal
import time

class DateUpdater(QThread):
    _SIGNAL_data = pyqtSignal(str)  # Señal para actualizar la hora en la UI
    def __init__(self,parent):
        super().__init__(None)
        self._FLAG_run=True
        self.falseParent=parent
    def run(self):
        """Ejecuta el hilo y emite la hora actual cada segundo."""
        while True:
            fecha = time.strftime("%d-%m-%Y")  # Obtener la hora en formato HH:MM:SS
            # print(hora)  # Solo para depuración
            fecha = f"Fecha: {fecha}"
            self._SIGNAL_data.emit(fecha)  # Emite la nueva hora
            time.sleep(3600)  # Espera 1 segundo antes de actualizar nuevamente

