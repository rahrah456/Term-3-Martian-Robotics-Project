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
        print()
        print("Commands:")
        print("  pid:line:<Kp>,<Ki>,<Kd>     set line-follow PID gains")
        print("  pid:speed:<Kp>,<Ki>,<Kd>    set speed-control PID gains")
        print("  pid:turn:<Kp>,<Ki>,<Kd>     set turn PID gains")
        print("  stop                        engage kill switch")
        print("  go                          disengage kill switch")
        print("  status                      request telemetry")
        print("  anything else               sent raw to Arduino")
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
