import serial
import time

while True:
    print(f'***** Throughput Tester *****')
    device = input('0:UART 1:USB = ')
    chunk = input('ChunkSize (byte) = ')
    N = input('N = ')

    with serial.Serial('/dev/ttyACM0', 921600) as ser:
        ser.write((device + '\r\n').encode('utf-8'))
        device = ser.readline()
        ser.write((chunk + '\r\n').encode('utf-8'))
        chunk = ser.readline()
        ser.write((N + '\r\n').encode('utf-8'))
        N = ser.readline()
        print('Device = ' + device.strip().decode('utf-8'))
        print('ChunkSize = ' + chunk.strip().decode('utf-8') + ' byte')
        print('Repeat = ' + N.strip().decode('utf-8'))

        device = int(device.strip().decode('utf-8'))
        chunk = int(chunk.strip().decode('utf-8'))
        N = int(N.strip().decode('utf-8'))

        if device:
            with serial.Serial('/dev/ttyACM1') as data:
                for i in range(N):
                    data.read(chunk)
            result = ser.readline()
            print(result.strip().decode('utf-8'))
            print()
        else:
            for i in range(N):
                ser.read(chunk)
            result = ser.readline()
            print(result.strip().decode('utf-8'))
            print()
