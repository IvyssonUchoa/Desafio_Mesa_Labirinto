import serial
import json
import time
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

# Configurações de conexão
SERIAL_PORT = '/dev/ttyUSB0' # Para porta no Linux
#SERIAL_PORT = 'COM8' # Para porta no Windows

BAUD_RATE = 115200
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "SEU TOKEN"
INFLUX_ORG = "SUA ORG"
INFLUX_BUCKET = "SEU BUCKET"

def main():
    # Configura a conexão serial e cria um cliente InfluxDB
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = client.write_api(write_options=SYNCHRONOUS)

    print(f"Lendo dados de {SERIAL_PORT}...")

    while True:
        try:
            if ser.in_waiting > 0:
                # Lê os dados recebidos via porta serial
                line = ser.readline().decode('utf-8').strip()

                try:
                    # Decodifica o JSON e filtra os valores de pitch e roll
                    data = json.loads(line) 
                    pitch = float(data['pitch'])
                    roll = float(data['roll'])
                    
                    # Cria o ponto para o InfluxDB
                    point = Point("orientacao_mesa") \
                        .field("pitch", pitch) \
                        .field("roll", roll)
                    
                    # Envia os dados para o InfluxDB
                    write_api.write(bucket=INFLUX_BUCKET, org=INFLUX_ORG, record=point)
                    print(f"Enviado: Pitch={pitch}, Roll={roll}")
                    
                except json.JSONDecodeError:
                    print(f"Erro no JSON: {line}")
                    
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Erro: {e}")

if __name__ == "__main__":
    main()