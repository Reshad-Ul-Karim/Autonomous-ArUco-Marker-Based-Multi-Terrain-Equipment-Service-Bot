import socket
import time
from pynput import keyboard

ESP_IP = "172.27.106.106"
PORT = 3333

# Send rate (Hz). Higher = more responsive; 30-60 is good.
TICK = 1 / 40.0

def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((ESP_IP, PORT))
    s.settimeout(None)
    return s

sock = None

def safe_connect_forever():
    global sock
    while True:
        try:
            sock = connect()
            print(f"✅ Connected to {ESP_IP}:{PORT}")
            try:
                sock.sendall(b"S")
            except Exception:
                pass
            return
        except Exception as e:
            print("⚠️ Connect failed, retrying...", e)
            time.sleep(1)

def send(ch: str):
    global sock
    if not sock:
        safe_connect_forever()
    try:
        sock.sendall(ch.encode("utf-8"))
    except Exception as e:
        print("⚠️ Disconnected, reconnecting...", e)
        try:
            sock.close()
        except Exception:
            pass
        sock = None
        safe_connect_forever()
        sock.sendall(ch.encode("utf-8"))

# ---- State (TRUE press/release) ----
pressed = set()
speed_key = "0"
last_cmd = None

def compute_cmd():
    # Swap forward/backward:
    # UP = backward ('B'), DOWN = forward ('F')
    up = keyboard.Key.up in pressed
    down = keyboard.Key.down in pressed
    left = keyboard.Key.left in pressed
    right = keyboard.Key.right in pressed

    throttle = None
    if up and not down:
        throttle = "B"   # swapped
    elif down and not up:
        throttle = "F"
    else:
        throttle = None

    steer = None
    if left and not right:
        steer = "L"
    elif right and not left:
        steer = "R"
    else:
        steer = None

    # diagonals
    if throttle == "F" and steer == "L":
        return "G"  # forward_left
    if throttle == "F" and steer == "R":
        return "I"  # forward_right
    if throttle == "B" and steer == "L":
        return "H"  # backward_left
    if throttle == "B" and steer == "R":
        return "J"  # backward_right

    # steering only
    if throttle is None and steer == "L":
        return "L"
    if throttle is None and steer == "R":
        return "R"

    # throttle only
    if throttle == "F":
        return "F"
    if throttle == "B":
        return "B"

    return "S"

def apply_cmd():
    global last_cmd
    cmd = compute_cmd()
    if cmd != last_cmd:
        send(cmd)
        last_cmd = cmd
        # print("cmd:", cmd)

def on_press(key):
    global speed_key

    # Speed keys
    if hasattr(key, "char") and key.char:
        ch = key.char.lower()
        if ch in "0123456789q":
            speed_key = ch
            send(ch)
            print("speed:", ch)
            return
        if ch == "x":
            speed_key = "x"
            send("x")
            print("speed: x (0)")
            return

    # Stop
    if key == keyboard.Key.space:
        pressed.clear()
        send("S")
        return

    # Add key to pressed + apply
    pressed.add(key)
    apply_cmd()

def on_release(key):
    # Quit
    if key == keyboard.Key.esc:
        pressed.clear()
        send("S")
        try:
            sock.close()
        except Exception:
            pass
        print("✅ Exit")
        return False

    # Remove key + apply
    if key in pressed:
        pressed.remove(key)
        apply_cmd()

def main():
    safe_connect_forever()
    print("Controls:")
    print("  Hold ↑ = BACKWARD (swapped)")
    print("  Hold ↓ = FORWARD  (swapped)")
    print("  Hold ←/→ = steer, diagonals auto")
    print("  Space = stop, Esc = quit")
    print("  0-9/q = speed, x = true 0 speed (if ESP supports)")

    # Optional: keep-alive loop to resend current cmd at steady rate (helps some networks)
    # We'll run listener in main thread, and a lightweight tick loop in background style:
    # easiest: just rely on press/release events (works well).
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        # Also run a small keepalive loop while listener is active
        while listener.running:
            # resend current command at fixed rate (prevents any stale state)
            if last_cmd:
                try:
                    send(last_cmd)
                except Exception:
                    pass
            time.sleep(TICK)

if __name__ == "__main__":
    main()
