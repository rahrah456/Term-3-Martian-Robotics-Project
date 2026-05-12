import socket

UDP_IP = "172.20.10.2"  # Arduino IP (from Serial Monitor)
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))

# Send message
#sock.sendto(b"Die", (UDP_IP, UDP_PORT))
sock.sendto(b"Live", (UDP_IP, UDP_PORT))

# Receive response
data, addr = sock.recvfrom(1024)
print("Received:", data.decode())