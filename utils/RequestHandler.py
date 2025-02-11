from http.server import BaseHTTPRequestHandler
import json

class RequestHandler(BaseHTTPRequestHandler):
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        # print(type(post_data))

        # Decodificar los datos JSON
        data = json.loads(post_data)
        data = json.loads(data)
        # Imprimir los datos recibidos
        print("Datos recibidos:", data)

        # Extraer las coordenadas del JSON y almacenarlas en la variable global
        global COORDINATES

        coordinates = []
        print(type(data))
        for item in data:
            # Verificar si todas las claves necesarias están presentes en el objeto
            if all(key in item for key in ('x1', 'y1', 'x2', 'y2')):
                # Agregar las coordenadas al diccionario 'coordinates'
                coordinates.append({
                    'x1': item['x1'],
                    'y1': item['y1'],
                    'x2': item['x2'],
                    'y2': item['y2']
                })
        COORDINATES = coordinates

        # Enviar una respuesta
        self._set_response()
        self.wfile.write(json.dumps(
            {'message': 'Datos recibidos correctamente'}).encode('utf-8'))
