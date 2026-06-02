import os
import time
import subprocess
import sys
import serial          # pip install pyserial
import serial.tools.list_ports

try:
    from music21 import converter, note, chord, tempo, stream
    import fitz
except ImportError:
    print("Error: Missing libraries.  pip install music21 pymupdf pyserial")
    sys.exit(1)


FILE = "deck-the-halls.pdf"

SERIAL_PORT = None
BAUD_RATE   = 115200


STEPS_PER_FRET = 200

MAX_FRET = 12

MOVE_SETTLE_S = 0.05


OPEN_MIDI = [67, 60, 64, 69]

def note_to_midi(n: note.Note) -> int:
    return int(n.pitch.midi)


def midi_to_fret(midi_pitch: int, string_idx: int) -> int | None:

    fret = midi_pitch - OPEN_MIDI[string_idx]
    if 0 <= fret <= MAX_FRET:
        return fret
    return None


def best_voicing_single(midi_pitch: int, current_frets: list[int]) -> tuple[int, int] | None:

    candidates = []
    for s in range(4):
        fret = midi_to_fret(midi_pitch, s)
        if fret is not None:
            travel = abs(fret - current_frets[s])
            candidates.append((travel, s, fret))

    if not candidates:
        return None

    candidates.sort()
    _, s, fret = candidates[0]
    return s, fret


def best_voicing_chord(midi_pitches: list[int],
                        current_frets: list[int]) -> dict[int, int]:

    used_strings = set()
    assignment: dict[int, int] = {}

    for midi in sorted(set(midi_pitches)):
        candidates = []
        for s in range(4):
            if s in used_strings:
                continue
            fret = midi_to_fret(midi, s)
            if fret is not None:
                travel = abs(fret - current_frets[s])
                candidates.append((travel, s, fret))

        if candidates:
            candidates.sort()
            _, s, fret = candidates[0]
            assignment[s] = fret
            used_strings.add(s)

    return assignment



def find_arduino_port() -> str:
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").lower()
        mfg  = (port.manufacturer or "").lower()
        if "arduino" in desc or "arduino" in mfg or "ch340" in desc or "cp210" in desc:
            return port.device
    ports = list(serial.tools.list_ports.comports())
    if ports:
        return ports[0].device
    raise RuntimeError("No serial port found.  Connect the Arduino and retry, "
                       "or set SERIAL_PORT manually.")


def open_serial(port: str | None = None, baud: int = BAUD_RATE) -> serial.Serial:
    port = port or find_arduino_port()
    print(f"[serial] Connecting to Arduino on {port} @ {baud} baud …")
    ser = serial.Serial(port, baud, timeout=2)
    deadline = time.time() + 5
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "READY":
            print("[serial] Arduino is ready.")
            return ser
        if line:
            print(f"[serial] Arduino says: {line}")
    print("[serial] Warning: did not receive READY within 5 s – continuing anyway.")
    return ser


# def send(ser: serial.Serial, command: str):
#     """Send a newline-terminated command and (optionally) print it."""
#     print(f"  → {command}")
#     ser.write((command + "\n").encode())

def send(ser, command: str):
    """Dry run: print the command instead of sending to Arduino."""
    print(f"  [DRY RUN] → {command}")



def get_xml() -> str:
    """PDF → PNG → oemer → MusicXML (unchanged from main.py)."""
    doc = fitz.open(FILE)
    img = FILE.replace(".pdf", ".png")
    doc.load_page(0).get_pixmap(dpi=300).save(img)
    subprocess.run(["oemer", img], check=True)
    return img.replace(".png", ".musicxml")


def play(xml: str, ser: serial.Serial):

    score  = converter.parse(xml).flatten().notesAndRests
    bpm    = 120.0

    current_frets = [0, 0, 0, 0]

    send(ser, "HOME")
    time.sleep(1.0)

    for item in score:
        if isinstance(item, tempo.MetronomeMark):
            bpm = float(item.number)
            print(f"[tempo] {bpm} BPM")
            continue

        duration_s = float(item.quarterLength) * (60.0 / bpm)

        if item.isRest:
            print(f"[rest]  {duration_s:.3f}s")
            time.sleep(duration_s)
            continue

        if isinstance(item, note.Note):
            midi = note_to_midi(item)
            result = best_voicing_single(midi, current_frets)
            if result is None:
                print(f"[skip]  {item.nameWithOctave} – out of range on all strings")
                time.sleep(duration_s)
                continue

            s, fret = result
            steps   = fret * STEPS_PER_FRET
            print(f"[note]  {item.nameWithOctave}  →  string {s}, fret {fret}, {steps} steps")

            send(ser, f"MOVE {s} {steps}")
            time.sleep(MOVE_SETTLE_S)
            send(ser, f"PLUCK {s}")
            current_frets[s] = fret

            time.sleep(max(0.0, duration_s - MOVE_SETTLE_S))

        elif isinstance(item, chord.Chord):
            midi_pitches = [int(n.pitch.midi) for n in item.notes]
            voicing      = best_voicing_chord(midi_pitches, current_frets)
            names        = [n.nameWithOctave for n in item.notes]
            print(f"[chord] {names}  →  {voicing}")

            for s, fret in voicing.items():
                steps = fret * STEPS_PER_FRET
                send(ser, f"MOVE {s} {steps}")
                current_frets[s] = fret

            time.sleep(MOVE_SETTLE_S)

            for s in voicing:
                send(ser, f"PLUCK {s}")

            time.sleep(max(0.0, duration_s - MOVE_SETTLE_S))



if __name__ == "__main__":
    xml_file = get_xml()
    #ser      = open_serial(SERIAL_PORT, BAUD_RATE)
    ser = None
    try:
        play(xml_file, ser)
    finally:
        send(ser, "HOME")
        ser.close()
        print("[done]  Carriages homed. Serial port closed.")
