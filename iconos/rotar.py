from PIL import Image
import os

# Ruta de la imagen original
input_image_path = "iconos/pitch_nar.png"

# Carpeta donde se guardarán las imágenes rotadas
output_folder = "pitch_images"

# Crea la carpeta si no existe
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Abre la imagen
imagen = Image.open(input_image_path)

# Rota y guarda la imagen para cada ángulo de 0 a 359 grados
for angle in range(360):
    # Rota la imagen
    imagen_rotada = imagen.rotate(angle, expand=True)
    
    # Guarda la imagen rotada
    output_image_path = os.path.join(output_folder, f"pitch_nar_{angle}.png")
    imagen_rotada.save(output_image_path)

print("Imágenes rotadas y guardadas en la carpeta:", output_folder)