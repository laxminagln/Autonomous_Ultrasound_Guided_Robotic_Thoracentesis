import socket
import cv2
import pickle
import struct

HOST = "0.0.0.0"
PORT = 5001

video_path = "/home/laxmi/GitHub/MSc-Dissertation/Videos/scan_rviz_preview.mp4"

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print("Jetson video server waiting...")

conn, addr = server_socket.accept()
print("Connected:", addr)

cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    data = pickle.dumps(frame)
    message = struct.pack("Q", len(data)) + data

    conn.sendall(message)

cap.release()
conn.close()
server_socket.close()