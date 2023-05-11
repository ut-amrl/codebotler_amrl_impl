#!/usr/bin/env python

import rospy 
from std_msgs.msg import String

import tkinter as tk
from gtts import gTTS
import os
import speech_recognition as sr

def listen_for_yes_or_no():
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say yes or no!")
        while True:
            audio = r.listen(source)
            try:
                text = r.recognize_google(audio)
                print(f"You said: {text}")
                if  "yes" in text.lower():
                    return True
                    
                elif "no" in text.lower():
                    return False
            except sr.UnknownValueError:
                print("Sorry, could not understand audio.")
            except sr.RequestError as e:
                print(f"Speech recognition request failed: {e}")


class MyGUI:
    def __init__(self, master):
        self.master = master
        master.title("My GUI")
        master.attributes('-fullscreen', True)  # set full screen mode

        # create a frame to hold the buttons
        button_frame = tk.Frame(master)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # create two hidden buttons
        self.left_button = tk.Button(button_frame, text="Yes!", command=self.response_yes, width=15, height=15, bg="#ff0000")
        self.left_button.pack_forget()

        self.right_button = tk.Button(button_frame, text="No!", command=self.response_no, width=15, height=15, bg="#ff0000")
        self.right_button.pack_forget()

        self.label = tk.Label(master, text="hello", font=("Helvetica", 20))
        self.label.pack()

        rospy.init_node('gui_interface', disable_signals=True)
        self.robot_say_sub = rospy.Subscriber('robot_say', String, self.message_cb)
        self.robot_ask_sub = rospy.Subscriber('robot_ask', String, self.ask_cb)
        self.human_response_pub = rospy.Publisher('human_response', String, queue_size=10)

        self.options = [] # hardcode

    def update_label(self, text):
        # Create a gTTS object and specify the language
        tts = gTTS(text=text, lang='en')
        current_path = os.path.abspath(__file__)
        file_path = os.path.join(os.path.dirname(current_path), 'audio', 'hello.mp3')

        # Save the audio file
        tts.save(file_path)

        # Play the audio file
        os.system('mpg321 {}'.format(file_path))
        self.label.config(text=f"Robot says: \"{text}\"", font=("Helvetica", 20))

    def message_cb(self, msg):
        print("message:", msg.data)
        self.update_label(msg.data)
        
    
    def ask_cb(self, msg):
        msg = eval(msg.data)
        question = msg[-1]
        self.update_label(question)

        self.options = msg[:-1]
        yes_ans = listen_for_yes_or_no()
        if yes_ans: 
            self.human_response_pub.publish(self.options[0])
        else:
            self.human_response_pub.publish(self.options[1])

        self.left_button.pack(side=tk.LEFT, padx=80, pady=40)
        self.right_button.pack(side=tk.RIGHT, padx=80, pady=40)

    def response_yes(self):
        print("answered {}".format(self.options[0]))
        self.human_response_pub.publish(self.options[0])
        self.left_button.pack_forget()
        self.right_button.pack_forget()

    def response_no(self):
        print("answered {}".format(self.options[1]))
        self.human_response_pub.publish(self.options[1])
        self.left_button.pack_forget()
        self.right_button.pack_forget()


if __name__ == '__main__':
    print("start gui")
    root = tk.Tk()
    my_gui = MyGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.signal_shutdown("User shutdown")
        raise
