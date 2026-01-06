import time
import serial
from serial.tools import list_ports
from pynput import keyboard

BAUD = 115200
SEND_HZ = 30

STEP_DEG = 2
STEP_GRIP = 5

BASE_LIM  = (0, 180)
JOINT_LIM = (20, 160)
GRIP_LIM  = (0, 100)

HOME_BASE, HOME_JOINT, HOME_GRIP = 110, 160, 40
base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP

running = True
held = set()            # for letter keys (a,d,w,s,q,e)
held_special = set()    # for arrow keys

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def find_port():
    candidates = []
    for p in list_ports.comports():
        dev = (p.device or "").lower()
        desc = (p.description or "").lower()
        if ("usbserial" in dev) or ("slab" in dev) or ("wch" in dev) or ("cp210" in desc) or ("ch340" in desc):
            candidates.append(p.device)
    for c in candidates:
        if str(c).startswith("/dev/cu."):
            return c
    return candidates[0] if candidates else None

PORT = find_port()
if not PORT:
    raise RuntimeError("ESP32 port not found. Run: python -m serial.tools.list_ports")

print("Using port:", PORT)
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

def send_pose():
    global base, joint, grip
    base  = clamp(base,  *BASE_LIM)
    joint = clamp(joint, *JOINT_LIM)
    grip  = clamp(grip,  *GRIP_LIM)
    ser.write(f"{base},{joint},{grip}\n".encode())

def on_press(key):
    global running, base, joint, grip

    if key == keyboard.Key.esc:
        running = False
        return False

    # Arrow keys
    if key in (keyboard.Key.left, keyboard.Key.right, keyboard.Key.up, keyboard.Key.down):
        held_special.add(key)
        return

    try:
        k = key.char.lower()
        held.add(k)

        if k == 'r':
            base, joint, grip = HOME_BASE, HOME_JOINT, HOME_GRIP
            send_pose()
            print(f"[HOME] base={base} joint={joint} grip={grip}")

        if k == 'p':
            print(f"[POSE] base={base} joint={joint} grip={grip}")

    except Exception:
        pass

def on_release(key):
    # Arrow keys
    if key in held_special:
        held_special.discard(key)
        return
    try:
        held.discard(key.char.lower())
    except Exception:
        pass

print("Controls (hold keys):")
print("Arrow Left/Right = base -, +")
print("Arrow Up/Down    = joint +, -")
print("Q/E              = gripper open/close")
print("R = HOME | P = print pose | ESC = exit")

send_pose()

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

period = 1.0 / SEND_HZ
try:
    while running:
        moved = False

        # Arrow keys
        if keyboard.Key.left in held_special:  base -= STEP_DEG; moved = True
        if keyboard.Key.right in held_special: base += STEP_DEG; moved = True
        if keyboard.Key.up in held_special:    joint += STEP_DEG; moved = True
        if keyboard.Key.down in held_special:  joint -= STEP_DEG; moved = True

        # Optional WASD support still works
        if 'a' in held: base -= STEP_DEG; moved = True
        if 'd' in held: base += STEP_DEG; moved = True
        if 'w' in held: joint += STEP_DEG; moved = True
        if 's' in held: joint -= STEP_DEG; moved = True

        # Gripper
        if 'q' in held: grip -= STEP_GRIP; moved = True
        if 'e' in held: grip += STEP_GRIP; moved = True

        if moved:
            send_pose()

        time.sleep(period)

finally:
    ser.close()
    listener.stop()
    print("Closed serial.")
