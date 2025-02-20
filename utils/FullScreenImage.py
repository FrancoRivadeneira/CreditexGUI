import PyQt5.QtCore as QtCore
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
class FullScreenImage(QDialog):
    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Imagen en pantalla completa")
        self.setWindowState(Qt.WindowFullScreen)  # Pantalla completa

        layout = QVBoxLayout(self)
        label = QLabel(self)
        pixmap = QPixmap(image_path)

        if pixmap.isNull():
            print(f"No se pudo cargar la imagen: {image_path}")
            return

        label.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        label.setAlignment(Qt.AlignCenter)
        
        layout.addWidget(label)
        self.setLayout(layout)

        # Cerrar la ventana al hacer clic
        self.mousePressEvent = self.closeEvent