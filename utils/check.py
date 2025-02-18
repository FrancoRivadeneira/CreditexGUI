import pygame

# Inicializar pygame y joystick
pygame.init()
pygame.joystick.init()

# Detectar joystick
if pygame.joystick.get_count() == 0:
    print("⚠️ No se detectó ningún joystick. Conéctalo y vuelve a intentarlo.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"🎮 Joystick detectado: {joystick.get_name()}")
    print(f"🔢 Número de ejes: {joystick.get_numaxes()}")
    print(f"🎛️ Número de botones: {joystick.get_numbuttons()}")
    print(f"📍 Número de hats: {joystick.get_numhats()}")

    # Mapear los ejes
    print("\n📌 Mapeo de ejes:")
    for i in range(joystick.get_numaxes()):
        print(f"  - Eje {i}: {joystick.get_axis(i):.2f}")

    # Mapear los botones
    print("\n📌 Mapeo de botones:")
    for i in range(joystick.get_numbuttons()):
        print(f"  - Botón {i}: {joystick.get_button(i)}")

    # Mapear los hats (D-Pad)
    print("\n📌 Mapeo de hats (D-Pad):")
    for i in range(joystick.get_numhats()):
        print(f"  - Hat {i}: {joystick.get_hat(i)}")

# Bucle para leer eventos en tiempo real
running = True
print("\n🎮 Mueve los joysticks y presiona botones para ver los valores en tiempo real.")
print("Presiona ESC para salir.")

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False
        elif event.type == pygame.JOYAXISMOTION:
            print(f"📍 Eje {event.axis} movido: {event.value:.2f}")
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"🟢 Botón {event.button} presionado")
        elif event.type == pygame.JOYBUTTONUP:
            print(f"🔴 Botón {event.button} liberado")
        elif event.type == pygame.JOYHATMOTION:
            print(f"🎯 Hat {event.hat} movido: {event.value}")

pygame.quit()
