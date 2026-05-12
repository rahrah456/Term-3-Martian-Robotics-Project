import socket
import sys
import threading

PORT = 4210


def listen(sock):
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            print(f"\n[Arduino] {data.decode()}")
            print("> ", end="", flush=True)
        except:
            break


def main():
    if len(sys.argv) < 2:
        print("Usage: python dashboard.py <arduino_ip>")
        print("   or:  python dashboard.py <arduino_ip> <command>")
        sys.exit(1)

    arduino_ip = sys.argv[1]
    addr = (arduino_ip, PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(0.1)

    threading.Thread(target=listen, args=(sock,), daemon=True).start()

    if len(sys.argv) >= 3:
        cmd = " ".join(sys.argv[2:])
        sock.sendto(cmd.encode(), addr)
        print(f"Sent: {cmd}")
        try:
            data, _ = sock.recvfrom(1024)
            print(f"Reply: {data.decode()}")
        except:
            pass
        return

    print(f"Connected to {arduino_ip}:{PORT}")
    print("Commands: change_pid:<Kp>,<Ki>,<Kd>  |  stop  |  go  |  status  |  quit")
    print("Anything else is sent raw to the Arduino.")
    print()

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue
        if cmd == "quit":
            break

        sock.sendto(cmd.encode(), addr)


if __name__ == "__main__":
    main()
