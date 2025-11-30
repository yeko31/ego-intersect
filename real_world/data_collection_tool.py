import argparse
import socket, os, time
from os.path import join
import numpy as np
import cv2
import threading
import tkinter as tk
from tkinter import ttk

# ------------------- Config -------------------
# Base save directories
BASE_SAVE_DIR = r""
SEQ_SAVE_DIR = BASE_SAVE_DIR + "_seq"

os.makedirs(BASE_SAVE_DIR, exist_ok=True)
os.makedirs(SEQ_SAVE_DIR, exist_ok=True)

# Categories
CATEGORIES = [
    "left_intersection",
    "left_bend",
    "right_intersection",
    "right_bend",
    "t_intersection",
    "four_way_intersection",
    "dead_end",
    "straight"
]

# Adjustable save FPS
SAVE_FPS = 5   # frames per second for continuous saving
save_interval = 1.0 / SAVE_FPS

# ------------------- State -------------------
current_category = None
saving = False
saved_count = 0
sequence_mode = False
sequence_buffer = []
lock = threading.Lock()

# ------------------- Helpers -------------------
def save_image_opencv(img, category, name):
    """Save OpenCV image (BGR) into category folder for continuous saving."""
    folder = join(BASE_SAVE_DIR, category)
    os.makedirs(folder, exist_ok=True)
    path = join(folder, f"{name}.jpg")
    cv2.imwrite(path, img)


def get_next_sequence_index(category):
    """Find the next sequence number for a category by scanning the folder."""
    seq_folder = join(SEQ_SAVE_DIR, category)
    os.makedirs(seq_folder, exist_ok=True)

    existing = [d for d in os.listdir(seq_folder) if d.startswith("seq")]
    if not existing:
        return 1
    nums = []
    for d in existing:
        try:
            nums.append(int(d.replace("seq", "")))
        except ValueError:
            pass
    return max(nums) + 1


def save_sequence(category, frames, seq_index):
    """Save 16 frames into dataset_seq/<category>/seqX"""
    folder = join(SEQ_SAVE_DIR, category, f"seq{seq_index}")
    os.makedirs(folder, exist_ok=True)

    for i, frame in enumerate(frames):
        path = join(folder, f"frame_{i+1:02d}.jpg")
        cv2.imwrite(path, frame)


# ------------------- Receiver -------------------
def receive_frames(deck_ip, deck_port):
    global saving, current_category, saved_count, sequence_mode, sequence_buffer

    print(f"Connecting to socket on {deck_ip}:{deck_port}...")
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((deck_ip, deck_port))
    print("Socket connected")

    buffer_data = b""
    start_time = time.time()
    frame_count = 0
    last_save_time = 0

    while True:
        chunk = client_socket.recv(8192)
        if not chunk:
            break

        buffer_data += chunk
        start_idx = buffer_data.find(b'\xff\xd8')
        end_idx = buffer_data.find(b'\xff\xd9', start_idx + 2)

        while start_idx != -1 and end_idx != -1:
            frame_data = buffer_data[start_idx:end_idx+2]
            buffer_data = buffer_data[end_idx+2:]

            img_array = np.frombuffer(frame_data, np.uint8)
            decoded = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if decoded is not None:
                frame_count += 1
                fps = 1.0 / (time.time() - start_time)
                start_time = time.time()

                cv2.imshow("AI-deck Stream", decoded)

                with lock:
                    if saving and current_category is not None:
                        now = time.time()
                        if sequence_mode:
                            # Sequence mode (16 frames only)
                            sequence_buffer.append(decoded.copy())
                            if len(sequence_buffer) == 16:
                                seq_idx = get_next_sequence_index(current_category)
                                save_sequence(current_category, sequence_buffer, seq_idx)
                                sequence_buffer = []
                                saving = False   # auto-stop after one sequence
                                print(f"[SEQ] Saved {current_category}/seq{seq_idx} (16 frames). Waiting for next GUI command.")
                        else:
                            # Continuous saving mode
                            if now - last_save_time >= save_interval:
                                saved_count += 1
                                last_save_time = now
                                print(f"Saving: {current_category} (#{saved_count})")
                                save_image_opencv(decoded, current_category,
                                                  f"{current_category}_{frame_count:06d}")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    client_socket.close()
                    cv2.destroyAllWindows()
                    return

            start_idx = buffer_data.find(b'\xff\xd8')
            end_idx = buffer_data.find(b'\xff\xd9', start_idx + 2)

    client_socket.close()
    cv2.destroyAllWindows()


# ------------------- GUI -------------------
def gui_thread():
    global saving, current_category, saved_count, sequence_mode, sequence_buffer

    root = tk.Tk()
    root.title("Image Categorizer")

    status_var = tk.StringVar(value="Not Saving")

    def start_saving(cat):
        """Start continuous saving mode"""
        global saving, current_category, saved_count, sequence_mode
        with lock:
            saving = True
            sequence_mode = False
            current_category = cat
            saved_count = 0
        status_var.set(f"Saving (continuous): {cat} (0 images)")

    def start_sequence(cat):
        """Start sequence mode (16 frames only, auto-stop)"""
        global saving, current_category, sequence_mode, sequence_buffer
        with lock:
            saving = True
            sequence_mode = True
            current_category = cat
            sequence_buffer = []
        status_var.set(f"Capturing sequence: {cat} (16 frames)")

    def stop_saving():
        global saving, current_category, sequence_mode
        with lock:
            saving = False
            sequence_mode = False
            current_category = None
        status_var.set("Not Saving")

    def update_status():
        """Periodically update saved count in GUI."""
        with lock:
            if saving and current_category is not None:
                if sequence_mode:
                    status_var.set(f"Capturing {current_category} sequence ({len(sequence_buffer)}/16 frames)")
                else:
                    status_var.set(f"Saving: {current_category} ({saved_count} images @ {SAVE_FPS} FPS)")
        root.after(500, update_status)

    # Buttons for categories (continuous mode)
    ttk.Label(root, text="Continuous Capture").pack(pady=5)
    for cat in CATEGORIES:
        btn = ttk.Button(root, text=f"Start {cat.replace('_',' ').title()}",
                         command=lambda c=cat: start_saving(c))
        btn.pack(fill="x", padx=10, pady=2)

    # Buttons for sequence mode
    ttk.Label(root, text="Sequence Capture (16 frames)").pack(pady=5)
    for cat in CATEGORIES:
        btn = ttk.Button(root, text=f"Seq16 {cat.replace('_',' ').title()}",
                         command=lambda c=cat: start_sequence(c))
        btn.pack(fill="x", padx=10, pady=2)

    # Stop button
    stop_btn = ttk.Button(root, text="Stop", command=stop_saving)
    stop_btn.pack(fill="x", padx=10, pady=5)

    ttk.Label(root, textvariable=status_var).pack(pady=5)

    # periodic update
    update_status()

    root.mainloop()


# ------------------- Main -------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AI-deck JPEG streamer with categorization GUI")
    parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
    parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
    args = parser.parse_args()

    # Run GUI in separate thread
    t = threading.Thread(target=gui_thread, daemon=True)
    t.start()

    # Run receiver (main thread)
    receive_frames(args.n, args.p)
