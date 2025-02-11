import cv2

def extract_frames(video_path, output_folder):
    # Abre el video
    video_capture = cv2.VideoCapture(video_path)
    if not video_capture.isOpened():
        print("Error al abrir el video.")
        return
    
    # Obtiene la información del video
    fps = video_capture.get(cv2.CAP_PROP_FPS)
    total_frames = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = total_frames / fps

    # Crea el directorio de salida si no existe
    import os
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Itera sobre el video y guarda una imagen por cada segundo
    for second in range(int(duration)):
        # Salta al segundo deseado
        video_capture.set(cv2.CAP_PROP_POS_FRAMES, int(second * fps))
        success, frame = video_capture.read()
        if success:
            # Guarda la imagen
            output_path = os.path.join(output_folder, f"frame_{second}.jpg")
            cv2.imwrite(output_path, frame)
            print(f"Guardado frame {second}")
        else:
            print(f"No se pudo leer el frame {second}")

    # Libera el objeto de captura de video
    video_capture.release()
    print("Extracción de frames completa.")

# Ruta del video de entrada
video_path = "ArmTest.mp4"
# Carpeta de salida para los frames
output_folder = "DivVideo"

# Llama a la función para extraer los frames
extract_frames(video_path, output_folder)
