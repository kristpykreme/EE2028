import socket

HOST = '192.168.43.182'
PORT = 2028

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind( (HOST, PORT) )
s.listen(5)

while True:
    # endpoint established
    clientsocket, address = s.accept()
    print(f"Connection from (address) has been established")

    while True:
        temp_data = clientsocket.recv(16)
        temp_data = float(temp_data)
        print(repr(temp_data))
