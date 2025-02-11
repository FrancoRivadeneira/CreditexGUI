from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.uic import loadUi
from screens.MainUi import MainUi
import pandas as pd
import os
from PyQt5 import QtCore, QtGui, QtWidgets

class LoginUi (QtWidgets.QMainWindow):
    """Función Constructor"""

    def __init__(self):
        super(LoginUi, self).__init__()
        self.users_df = pd.read_csv('users.csv')
        # Cargamos interfaz principal
        loadUi("C:/Users/Franco Rivadeneira/Desktop/PlantasIndustrialesV2/screens/login.ui", self)
        self.username=''
        self.name=''
        self.last=''
        self.code=''
        self.password=''
        self.pic=''
        self.users_csv = 'users.csv'
        """ DEFINICION de FLAG """
        self._FLAG_socketConected = False
        self._FLAG_sshImuConnected = False
        self._FLAG_sshLidarConnected = False
        self._FLAG_recordVideos = False
        self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
       # Se abre maximizando la pantalla
        self.showNormal()
        self.showFullScreen()
        # Llamamos a la función para crear los frames desde el inicio
        self.initialize_user_frames()

    
        #self.btn_user1.clicked.connect(lambda: self.GoPassword("Franco", "iconos/load.jpg"))
        self.btn_addUsers.clicked.connect(lambda: self.GoAdd())
        self.btn_next.clicked.connect(lambda: self.GoPic())
        self.btn_return.clicked.connect(lambda: self.returnUsers())
        self.btn_cargarPic.clicked.connect(lambda: self.uploadPic())
        self.close_window_button.clicked.connect(lambda: self.close())
        self.btn_returnUsers.clicked.connect(lambda: self.GoUsers())
        self.btn_atras1.clicked.connect(lambda: self.GoUsers())
        self.btn_atras2.clicked.connect(lambda: self.GoAdd())
        self.btn_verificar.clicked.connect(lambda: self.check_password())
        self.editPassword.installEventFilter(self)


        

    def initialize_user_frames(self):
        for index, row in self.users_df.iterrows():
            username = row['username']
            image_path = row['photo']
            user_code = row['code']
            self.create_user_frame(username, image_path, user_code)
    def uploadPic(self):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getOpenFileName(self, "Seleccionar Imagen", "", "Images (*.png *.xpm *.jpg)", options=options)
        if fileName:
            
            save_path = os.path.join("user_images", f"{self.username}.jpg")
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            with open(fileName, 'rb') as fsrc, open(save_path, 'wb') as fdst:
                fdst.write(fsrc.read())
            QMessageBox.information(self, 'Éxito', f'Imagen guardada como {save_path}')
            self.pic=f'{save_path}'
            user_data = pd.DataFrame({
            'username': [self.username],
            'name': [self.name],
            'last': [self.last],
            'code': [self.code],
            'password': [self.password],
            'photo': [self.pic]
            })
            self.users_df = pd.concat([self.users_df, user_data], ignore_index=True)
            self.users_df.to_csv(self.users_csv, index=False)
            QMessageBox.information(self, 'Éxito', 'Usuario guardado en CSV')
            self.create_user_frame(self.username, save_path, self.code)
            self.textUsername.clear()
            self.textName.clear()
            self.textLast.clear()
            self.textCode.clear()
            self.textPassword.clear()
            self.username=''
            self.name=''
            self.last=''
            self.code=''
            self.password=''
            self.pic=''




    def eventFilter(self, source, event):
        if event.type() == QtCore.QEvent.KeyPress and source is self.editPassword:
            if event.key() == QtCore.Qt.Key_Return or event.key() == QtCore.Qt.Key_Enter:
                self.check_password()
                return True
        return super(LoginUi, self).eventFilter(source, event)
    def check_password(self):
        global USER,USERNAME,USERCODE,USERPICKPATH
        password = self.editPassword.text().strip()
        user = self.users_df[(self.users_df['password'] == password) & (self.users_df['username'] == USERNAME)]
        if not user.empty:
            # Load the user's photo
            USER = user.iloc[0]['username']
            USERNAME = user.iloc[0]['name']
            USERCODE = user.iloc[0]['code']
            USERPICKPATH = user.iloc[0]['photo']
            self.open_main_ui()
        else:
            QMessageBox.warning(self, 'Error', 'Contraseña incorrecta')
    def open_main_ui(self):
        self.main_ui = MainUi()
        self.main_ui.show()
        self.close()
    def on_text_changed(self):
        if self.editPassword.text().endswith('\n'):
            self.check_credentials()
    def create_user_frame(self, username, image_path, user_code):
        frame = QFrame()
        frame.setMinimumSize(260, 350)
        frame.setMaximumSize(260, 350)

        layout = QVBoxLayout()
        button = QPushButton()
        button.setMinimumSize(250, 250)
        button.setMaximumSize(250, 250)
        pixmap = QPixmap(image_path)
        pixmap = pixmap.scaled(250, 250, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        icon = QIcon(pixmap)
        button.setIcon(icon)
        button.setIconSize(QtCore.QSize(250, 250))

        label_name = QLabel(str(username))  # Convertir a str si es necesario
        label_code = QLabel(str(user_code))  # Convertir a str si es necesario
        
        font = QFont()
        font.setFamily('MS Shell Dlg 2')
        font.setPointSize(14)
        
        label_name.setFont(font)
        label_code.setFont(font)
        
        label_name.setStyleSheet("color: white; text-align: center;")
        label_code.setStyleSheet("color: white; text-align: center;")
        label_name.setAlignment(QtCore.Qt.AlignCenter)
        label_code.setAlignment(QtCore.Qt.AlignCenter)
        
        # Set maximum width for labels
        label_name.setWordWrap(True)
        label_code.setWordWrap(True)

        layout.addWidget(button)
        layout.addWidget(label_name)
        layout.addWidget(label_code)
        frame.setLayout(layout)
        button.clicked.connect(lambda checked, user=username, img=image_path: self.GoPassword(user, img))
        # Insert frame to the left of frame_create
        layout_usuarios = self.frame_usuarios.layout()
        index_create = layout_usuarios.indexOf(self.frame_create)
        layout_usuarios.insertWidget(index_create, frame)

    #Funciones para cambiar de pestaña

    def returnUsers(self):
        self.frame_login.setMaximumSize(10000, 100000)
        self.frame_password.setMaximumSize(0, 0)
        self.frame_new.setMaximumSize(0,0)
        self.frame_pic.setMaximumSize(0,0)
        self.editPassword.clear()
        
    def GoPic(self):
        self.username=self.textUsername.toPlainText()
        self.name=self.textName.toPlainText()
        self.last=self.textLast.toPlainText()
        self.code=self.textCode.toPlainText()
        self.password=self.textPassword.text()

        self.frame_login.setMaximumSize(0, 0)
        self.frame_password.setMaximumSize(0, 0)
        self.frame_new.setMaximumSize(0,0)
        self.frame_pic.setMaximumSize(10000,10000)
    def GoAdd(self):
        self.textUsername.setAlignment(Qt.AlignCenter)
        self.textName.setAlignment(Qt.AlignCenter)
        self.textLast.setAlignment(Qt.AlignCenter)
        self.textCode.setAlignment(Qt.AlignCenter)
        self.textPassword.setAlignment(Qt.AlignCenter)
        self.textPassword.setEchoMode(QtWidgets.QLineEdit.Password)


        self.frame_login.setMaximumSize(0, 0)
        self.frame_password.setMaximumSize(0, 0)
        self.frame_new.setMaximumSize(10000,10000)
        self.frame_pic.setMaximumSize(0,0)


    def GoPassword(self, username,image_path):
        global USERNAME
        USERNAME = username
        
        self.editPassword.setAlignment(Qt.AlignCenter)
        self.label_saludo.setText(f"Hola {USERNAME}")
        pixmap = QPixmap(image_path)
        pixmap = pixmap.scaled(450, 450, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        icon = QIcon(pixmap)
        self.btn_profile.setIcon(icon)
        self.btn_profile.setIconSize(QtCore.QSize(450, 450))

        # Distribucion espacio
        self.frame_login.setMaximumSize(0, 0)
        self.frame_password.setMaximumSize(100000, 100000)
        self.frame_new.setMaximumSize(0,0)
        self.frame_pic.setMaximumSize(0,0)
    def GoUsers(self):
        # Distribucion espacio
        self.editPassword.clear()
        self.frame_login.setMaximumSize(100000, 100000)
        self.frame_password.setMaximumSize(0, 0)
        self.frame_new.setMaximumSize(0,0)
        self.frame_pic.setMaximumSize(0,0)
        self.textUsername.clear()
        self.textName.clear()
        self.textLast.clear()
        self.textCode.clear()
        self.textPassword.clear()
    #################################################################

