## Library for PyQt
from PyQt5.QtWidgets import *  ###
from PyQt5.QtCore import *     ###
from PyQt5.QtGui import *      ###
from pygame.locals import *    ###

class CustomDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Escribe algo")
        
        # Crear los widgets dentro del cuadro
        self.label = QLabel("Nombre del archivo (.bag):")
        self.line_edit = QLineEdit()

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        #self.enter_button = QPushButton("Enter")

        # Conectar el botón "Enter" para cerrar el cuadro
        #self.enter_button.clicked.connect(self.accept)

        # Conectar los botones
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

        # Diseñar el layout del cuadro
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.line_edit)
        #layout.addWidget(self.enter_button)
        layout.addWidget(self.button_box)
        self.setLayout(layout)
