from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap


class FullScreenImage(QDialog):
    def __init__(self, image_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Imagen en pantalla completa")
        # Pantalla completa
        self.setWindowState(Qt.WindowState.WindowFullScreen)

        layout = QVBoxLayout(self)
        label = QLabel(self)
        pixmap = QPixmap(image_path)

        if pixmap.isNull():
            print(f"No se pudo cargar la imagen: {image_path}")
            return

        label.setPixmap(pixmap.scaled(self.size(),
                                      Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation))
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(label)
        self.setLayout(layout)

        # Cerrar la ventana al hacer clic
        self.mousePressEvent = self.closeEvent
