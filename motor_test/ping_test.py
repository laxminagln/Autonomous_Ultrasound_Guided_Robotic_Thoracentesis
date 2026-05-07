import socket
import subprocess
import os

HOST = "0.0.0.0"      # listen on all interfaces
PORT = 5000           # choose any free port

# Path to your motor control script on Jetson
MOTOR_SCRIPT = "/needle_pump_sequence.py"

def run_motor_script():
    if not os.path.exists(MOTOR_SCRIPT):
        return f"ERROR: Script not found: {MOTOR_SCRIPT}"

    try:
        result = subprocess.run(
            ["python3", MOTOR_SCRIPT],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            return f"SUCCESS\n{result.stdout}"
        else:
            return f"FAILED\n{result.stderr}"

    except Exception as e:
        return f"ERROR: {str(e)}"

def handle_command(command: str) -> str:
    command = command.strip()

    if command == "START_MOTOR":
        return run_motor_script()

    elif command == "PING":
        return "PONG"

    else:
        return f"UNKNOWN_COMMAND: {command}"

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind((HOST, PORT))
        server.listen(5)

        print(f"Jetson server listening on {HOST}:{PORT}")

        while True:
            conn, addr = server.accept()
            with conn:
                print(f"Connected by {addr}")

                data = conn.recv(1024)
                if not data:
                    continue

                command = data.decode("utf-8")
                print(f"Received: {command}")

                response = handle_command(command)
                conn.sendall(response.encode("utf-8"))

if __name__ == "__main__":
    main()