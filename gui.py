#!/usr/bin/env python3

import json
import os
from pathlib import Path
import rclpy
from rclpy.node import Node
import signal
from std_msgs.msg import String
import sys
import tkinter as tk
import speech_recognition as sr
import yaml
import threading

CONSOLE_LOG_DIR = Path(os.getenv("CODEBOTLER_CONSOLE_LOG_DIR", "/tmp/codebotler_console_logs"))
CONSOLE_LOG_FILE = CONSOLE_LOG_DIR / "gui.log"


def enable_console_log():
    CONSOLE_LOG_DIR.mkdir(parents=True, exist_ok=True)
    CONSOLE_LOG_FILE.write_text("")

    original_stdout_fd = os.dup(1)
    read_fd, write_fd = os.pipe()
    os.dup2(write_fd, 1)
    os.dup2(write_fd, 2)
    os.close(write_fd)

    try:
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    except Exception:
        pass

    def pump_console():
        with open(CONSOLE_LOG_FILE, "ab", buffering=0) as log:
            while True:
                try:
                    data = os.read(read_fd, 4096)
                except OSError:
                    break
                if not data:
                    break
                log.write(data)
                try:
                    os.write(original_stdout_fd, data)
                except OSError:
                    pass

    threading.Thread(target=pump_console, name="gui_console_log", daemon=True).start()


enable_console_log()


def listen_for_yes_or_no():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say yes or no!")
        while True:
            audio = r.listen(source)
            try:
                text = r.recognize_google(audio)
                print(f"You said: {text}")
                if "yes" in text.lower():
                    return True

                elif "no" in text.lower():
                    return False
            except sr.UnknownValueError:
                print("Sorry, could not understand audio.")
            except sr.RequestError as e:
                print(f"Speech recognition request failed: {e}")


class MyGUI(Node):
    def __init__(self, master):
        super().__init__('gui_interface')
        with open(Path(__file__).resolve().parent / 'data.yaml', 'r') as f:
            self.DATA = yaml.safe_load(f)
        self.master = master
        master.title("My GUI")
        master.attributes('-fullscreen', True)  # set full screen mode

        self.button_frame = None

        self.label = tk.Label(master, text="Cobot :)", font=("Helvetica", 180))
        self.label.pack(anchor=tk.CENTER, expand=True)

        self.robot_say_sub = self.create_subscription(String, self.DATA['ROBOT_SAY_TOPIC'], self.message_cb, 10)
        self.robot_ask_sub = self.create_subscription(String, self.DATA['ROBOT_ASK_TOPIC'], self.ask_cb, 10)
        self.human_response_pub = self.create_publisher(String, self.DATA['HUMAN_RESPONSE_TOPIC'], 10)

        self.button_list = []
        self.shutting_down = False

    def run_on_gui_thread(self, callback, *args):
        if self.shutting_down:
            return
        try:
            self.master.after(0, lambda: callback(*args))
        except tk.TclError:
            pass

    def update_label(self, text):
        if self.shutting_down:
            return
        self.label.config(text=f"{text}", font=("Helvetica", 80),
                          wraplength=int(self.master.winfo_screenwidth() * 0.8), justify="center")

    def reset_label(self):
        if self.shutting_down:
            return
        self.label.config(text="Cobot :)", font=("Helvetica", 180))
        self.label.pack(anchor=tk.CENTER, expand=True)

    def clear_buttons(self):
        if self.shutting_down:
            return
        for button in self.button_list:
            button.destroy()
        self.button_list.clear()
        if self.button_frame is not None:
            self.button_frame.destroy()
            self.button_frame = None

    def message_cb(self, msg):
        print("message:", msg.data)
        if msg.data:
            self.run_on_gui_thread(self.update_label, msg.data)
        else:
            self.run_on_gui_thread(self.reset_label)

    def on_button_click(self, option):
        print("answered {}".format(option))
        self.clear_buttons()
        self.human_response_pub.publish(String(data=option))
        self.reset_label()

    def ask_cb(self, msg):
        if not msg.data:
            self.run_on_gui_thread(self.clear_buttons)
            self.run_on_gui_thread(self.reset_label)
            return

        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as e:
            print(f"Could not parse ask request JSON: {e}")
            return

        question = request.get("question", "")
        options = request.get("options", [])
        if not isinstance(options, list):
            options = []

        self.run_on_gui_thread(self.show_question, question, options)

    def show_question(self, question, options):
        if self.shutting_down:
            return
        self.update_label(question)
        self.clear_buttons()

        self.button_frame = tk.Frame(self.master)
        self.button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        for option in options:
            button = tk.Button(
                self.button_frame,
                text=option,
                font=("Helvetica", 45),
                wraplength=int(self.master.winfo_screenwidth() * 0.25),
                command=lambda key=option: self.on_button_click(key),
            )
            button.config(width=10, height=10, pady=10)
            button.pack(side="left", fill="x", expand=True)

            self.button_list.append(button)


def main(args=None):
    rclpy.init(args=args)

    print("start gui")
    root = tk.Tk()
    my_gui = MyGUI(root)
    shutdown_event = threading.Event()

    def force_exit_after_delay():
        threading.Event().wait(3.0)
        os._exit(0)

    def close_window():
        if my_gui.shutting_down:
            return
        my_gui.shutting_down = True
        print("Shutting down gui")
        try:
            root.attributes('-fullscreen', False)
        except tk.TclError:
            pass
        try:
            root.withdraw()
        except tk.TclError:
            pass
        try:
            root.quit()
        except tk.TclError:
            pass
        try:
            root.destroy()
        except tk.TclError:
            pass

    def request_shutdown(signum=None, frame=None):
        first_request = not shutdown_event.is_set()
        shutdown_event.set()
        if first_request:
            threading.Thread(target=force_exit_after_delay, name="gui_force_exit", daemon=True).start()
        try:
            root.after(0, close_window)
        except tk.TclError:
            close_window()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    root.protocol("WM_DELETE_WINDOW", request_shutdown)
    root.bind("<Escape>", lambda event: request_shutdown())

    def spin_ros():
        while rclpy.ok() and not shutdown_event.is_set():
            try:
                rclpy.spin_once(my_gui, timeout_sec=0.1)
            except Exception as e:
                if not shutdown_event.is_set():
                    print(f"ROS GUI spin failed: {e}")
                break

    ros_thread = threading.Thread(target=spin_ros, name="gui_ros_spin", daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        request_shutdown()
    finally:
        shutdown_event.set()
        my_gui.shutting_down = True
        try:
            root.attributes('-fullscreen', False)
        except tk.TclError:
            pass
        try:
            root.withdraw()
        except tk.TclError:
            pass
        try:
            root.destroy()
        except tk.TclError:
            pass
        try:
            my_gui.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        ros_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
