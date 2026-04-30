#!/usr/bin/env python3

import json
import subprocess
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
import speech_recognition as sr
import yaml
import threading


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

        self.label = tk.Label(master, text="Jackal :)", font=("Helvetica", 180))
        self.label.pack(anchor=tk.CENTER, expand=True)

        self.robot_say_sub = self.create_subscription(String, self.DATA['ROBOT_SAY_TOPIC'], self.message_cb, 10)
        self.robot_ask_sub = self.create_subscription(String, self.DATA['ROBOT_ASK_TOPIC'], self.ask_cb, 10)
        self.human_response_pub = self.create_publisher(String, self.DATA['HUMAN_RESPONSE_TOPIC'], 10)

        self.options = []  # hardcode
        self.button_list = []

    def speak(self, text):
        espeak = subprocess.Popen(
            ["/usr/bin/espeak", "--stdout", "-s", "105", "-p", "75", text],
            stdout=subprocess.PIPE,
        )
        aplay = subprocess.Popen(
            ["/usr/bin/aplay", "-D", "sysdefault:CARD=P20"],
            stdin=espeak.stdout,
        )
        espeak.wait()
        aplay.wait()

    def update_label(self, text, speak=False):
        self.label.config(text=f"{text}", font=("Helvetica", 80),
                          wraplength=int(self.master.winfo_screenwidth() * 0.8), justify="center")
        if speak:
            threading.Thread(target=self.speak, args=[text], daemon=True).start()

    def message_cb(self, msg):
        print("message:", msg.data)
        self.update_label(msg.data)
        self.label.config(text="Jackal :)", font=("Helvetica", 180))
        self.label.pack(anchor=tk.CENTER, expand=True)

    def on_button_click(self, option):
        # call another function here
        print("answered {}".format(option))
        for b in self.button_list:
            b.destroy()
        if self.button_frame is not None:
            self.button_frame.destroy()
            self.button_frame = None
        self.human_response_pub.publish(String(data=option))
        self.label.config(text="Jackal :)", font=("Helvetica", 180))
        self.label.pack(anchor=tk.CENTER, expand=True)

    def ask_cb(self, msg):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as e:
            print(f"Could not parse ask request JSON: {e}")
            return

        question = request.get("question", "")
        options = request.get("options", [])
        if not isinstance(options, list):
            options = []

        self.update_label(question, speak=True)

        if self.button_frame is not None:
            self.button_frame.destroy()

        self.button_frame = tk.Frame(self.master)  # Create the frame here
        self.button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # create a button with text "Click me!"
        self.button_list.clear()
        for option in options:
            if "starting location" in option:
                continue
            shown_text = option.replace(' ', '\'s\n')
            if "office" in option:
                option = option.replace(' ', '\'s ')
            button = tk.Button(self.button_frame, text=shown_text, font=("Helvetica", 45), command=lambda key=option: self.on_button_click(key))
            button.config(width=10, height=10, pady=10)
            button.pack(side="left", fill="x", expand=True)

            self.button_list.append(button)


def main(args=None):
    rclpy.init(args=args)
    
    print("start gui")
    root = tk.Tk()
    my_gui = MyGUI(root)
    
    def spin_ros():
        while rclpy.ok():
            rclpy.spin_once(my_gui, timeout_sec=0.1)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        my_gui.destroy_node()
        rclpy.shutdown()
        raise


if __name__ == '__main__':
    main()
