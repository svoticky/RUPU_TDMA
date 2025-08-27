import serial
import time
import re
import csv
from pathlib import Path
from datetime import datetime

PUERTO = "COM11"       # Ej: "COM3" en Windows, "/dev/ttyUSB0" o "/dev/ttyACM0"
BAUD   = 115200
CSV_PATH = Path("datos_esp32_finales_atdma6.csv")

# Formato esperado:
# Datos[1] =  1525 ; 4535 ; 1526 ; S ; 14.667468 ; 14.000000 ; 0.016173 ; 2932
pat_linea = re.compile(r'^Datos\[(\d+)\]\s*=\s*(.*)$')

CAMPOS = ["ts", "idx", "enviados", "rx_m", "rx_v",
          "estado", "inicio", "vel_1", "vel_2", "curva", "delay"]

def parsear_linea(linea: str):
    m = pat_linea.match(linea.strip())
    if not m:
        return None

    idx = int(m.group(1))
    resto = m.group(2)
    partes = [p.strip() for p in resto.split(';')]
    if len(partes) != 10:
        return None

    try:
        ts = int(partes[0])
        enviados = int(partes[1])
        rx_m     = int(partes[2])
        rx_v     = int(partes[3])
        estado   = int(partes[4])
        inicio   = int(partes[5])
        vel_1    = float(partes[6].replace(',', '.'))
        vel_2    = float(partes[7].replace(',', '.'))
        curva    = float(partes[8].replace(',', '.'))
        delay    = int(partes[9])
    except ValueError:
        return None

    return {
        "ts": ts,
        "idx": idx,
        "enviados": enviados,
        "rx_m": rx_m,
        "rx_v": rx_v,
        "estado": estado,
        "inicio": inicio,
        "vel_1": vel_1,
        "vel_2": vel_2,
        "curva": curva,
        "delay": delay
    }

def asegurar_header(csv_path: Path):
    if not csv_path.exists():
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CAMPOS)
            writer.writeheader()

def main():
    asegurar_header(CSV_PATH)

    ser = serial.Serial(PUERTO, BAUD, timeout=1)
    time.sleep(2)

    with CSV_PATH.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CAMPOS)

        try:
            print(f"Grabando en {CSV_PATH.resolve()} — Ctrl+C para detener.")
            while True:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()
                if not linea:
                    continue

                reg = parsear_linea(linea)
                if reg is not None:
                    writer.writerow(reg)
                    f.flush()
                    print(reg)
        except KeyboardInterrupt:
            print("\nDetenido por el usuario.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()