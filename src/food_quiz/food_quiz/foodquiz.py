# ============================================================
# MALAYSIAN FOOD HERITAGE QUIZ
# ============================================================
#
# Purpose:
# Educational software module for a Cultural Heritage Service
# Robot competition project.
#
# This quiz teaches visitors about Malaysian food heritage
# through images, questions and cultural facts.
#
# Future Robot Integration:
# - LCD Touch Screen
# - Speaker (Voice Narration)
# - Microphone (Speech Recognition)
# - AI Generated Questions
# - Multi-language Support
#
# Author: ANAQI Robotics Team
# ============================================================

import tkinter as tk
from PIL import Image, ImageTk
import random
import os
import math
import platform
import shutil
import subprocess
import tempfile
import wave
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


# ============================================================
# UI THEME COLORS
# ============================================================

BG_COLOR = "#F3EBDD"       # Warm woven cloth
CARD_COLOR = "#FFFDF7"     # Clean ivory card
PANEL_COLOR = "#F8F0E3"    # Soft songket panel
ACCENT = "#6F3F2A"         # Traditional wood brown
ACCENT_DARK = "#4E2C1F"    # Deep carved wood
GREEN = "#1F5E4A"          # Malay traditional green
GOLD = "#C79B3B"           # Songket gold
TEXT = "#2B241F"           # Dark readable text
MUTED_TEXT = "#6D5A4A"     # Secondary text


# ============================================================
# QUESTION DATABASE
# ============================================================

questions = [
    {
        "question": "Which state is famous for Nasi Kerabu?",
        "image": "nasi_kerabu.jpg",
        "choices": [
            "Kelantan",
            "Johor",
            "Sabah",
            "Selangor"
        ],
        "answer": 0,
        "fact": "Nasi Kerabu is a traditional dish from Kelantan and is known for its blue rice made using bunga telang."
    },
    {
        "question": "What sauce is commonly served with Satay?",
        "image": "satay.jpg",
        "choices": [
            "Tomato Sauce",
            "Peanut Sauce",
            "Cheese Sauce",
            "Chili Jam"
        ],
        "answer": 1,
        "fact": "Satay is one of Malaysia's most famous foods and is traditionally served with peanut sauce."
    },
    {
        "question": "Which drink is known as Malaysia's 'pulled tea'?",
        "image": "tehtarik.jpg",
        "choices": [
            "Kopi O",
            "Sirap",
            "Teh Tarik",
            "Milo"
        ],
        "answer": 2,
        "fact": "Teh Tarik means 'pulled tea' and is made by repeatedly pouring tea between containers."
    },
    {
        "question": "What is the green ingredient in Cendol?",
        "image": "cendol.jpg",
        "choices": [
            "Green Jelly",
            "Spinach",
            "Apple",
            "Avocado"
        ],
        "answer": 0,
        "fact": "Cendol contains green jelly noodles made from rice flour and pandan."
    }
]

# Randomize questions every time quiz starts
random.shuffle(questions)


# ============================================================
# SOUND PLAYER
# ============================================================

class SoundPlayer:

    def __init__(self, root):
        self.root = root
        self.sound_dir = tempfile.mkdtemp(prefix="foodquiz_sounds_")
        self.correct_sound = self.create_tone("correct.wav", [523, 659, 784], 0.10)
        self.wrong_sound = self.create_tone("wrong.wav", [220, 165], 0.16)
        self.finish_sound = self.create_tone("finish.wav", [392, 523, 659, 784], 0.09)
        self.speech_process = None

    def create_tone(self, filename, frequencies, duration):
        path = os.path.join(self.sound_dir, filename)
        sample_rate = 44100
        amplitude = 12000

        with wave.open(path, "w") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)

            for frequency in frequencies:
                frames = int(sample_rate * duration)
                for n in range(frames):
                    fade = min(1, n / 500, (frames - n) / 500)
                    value = int(amplitude * fade * math.sin(2 * math.pi * frequency * n / sample_rate))
                    wav_file.writeframesraw(value.to_bytes(2, "little", signed=True))

        return path

    def play(self, sound_path):
        system_name = platform.system()

        if system_name == "Windows":
            try:
                import winsound
                winsound.PlaySound(sound_path, winsound.SND_FILENAME | winsound.SND_ASYNC)
                return
            except RuntimeError:
                pass

        for player in ("aplay", "paplay", "afplay"):
            if shutil.which(player):
                subprocess.Popen(
                    [player, sound_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                return

        self.root.bell()

    def speak(self, text):
        system_name = platform.system()
        self.stop_speech()

        if system_name == "Windows":
            powershell = shutil.which("powershell") or shutil.which("powershell.exe")
            if powershell:
                safe_text = text.replace("'", "''")
                speech_command = (
                    "Add-Type -AssemblyName System.Speech; "
                    "$speaker = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                    "$speaker.Rate = -1; "
                    f"$speaker.Speak('{safe_text}')"
                )
                self.speech_process = subprocess.Popen(
                    [powershell, "-NoProfile", "-WindowStyle", "Hidden", "-Command", speech_command],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                return

        speech_players = (
            ("spd-say", [text]),
            ("espeak-ng", [text]),
            ("espeak", [text]),
            ("flite", ["-t", text])
        )

        for player, args in speech_players:
            if shutil.which(player):
                self.speech_process = subprocess.Popen(
                    [player, *args],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                return

    def stop_speech(self):
        if self.speech_process and self.speech_process.poll() is None:
            self.speech_process.terminate()
        self.speech_process = None


# ============================================================
# MAIN APPLICATION CLASS
# ============================================================

class QuizApp:

    def __init__(self, root):

        # Store root window
        self.root = root

        # Configure main window
        self.root.title("Malaysian Food Heritage Quiz")
        self.root.geometry("980x680")
        self.root.minsize(860, 620)
        self.root.configure(bg=BG_COLOR)

        # Quiz variables
        self.score = 0
        self.q_index = 0
        self.answer_locked = False
        self.sound_player = SoundPlayer(root)

        # ====================================================
        # TITLE SECTION
        # ====================================================

        self.title_label = tk.Label(
            root,
            text="Malaysian Food Heritage Quiz",
            font=("Segoe UI", 25, "bold"),
            bg=BG_COLOR,
            fg=ACCENT_DARK
        )
        self.title_label.pack(pady=(18, 2))

        self.subtitle_label = tk.Label(
            root,
            text="Discover traditional flavours from across Malaysia",
            font=("Segoe UI", 11),
            bg=BG_COLOR,
            fg=MUTED_TEXT
        )
        self.subtitle_label.pack(pady=(0, 10))

        # ====================================================
        # MAIN CARD
        # ====================================================

        self.main_card = tk.Frame(
            root,
            bg=CARD_COLOR,
            padx=20,
            pady=18,
            highlightbackground=GOLD,
            highlightthickness=2
        )

        self.main_card.pack(
            fill="both",
            expand=True,
            padx=28,
            pady=(0, 22)
        )

        self.main_card.grid_columnconfigure(0, weight=3, minsize=360)
        self.main_card.grid_columnconfigure(1, weight=4, minsize=420)
        self.main_card.grid_rowconfigure(1, weight=1)

        # ====================================================
        # PROGRESS INDICATOR
        # ====================================================

        self.progress_label = tk.Label(
            self.main_card,
            text="Question 1 / 4",
            font=("Segoe UI", 11, "bold"),
            bg=CARD_COLOR,
            fg=GREEN
        )

        self.progress_label.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))

        # ====================================================
        # IMAGE DISPLAY
        # ====================================================

        self.image_panel = tk.Frame(
            self.main_card,
            width=360,
            height=320,
            bg=PANEL_COLOR,
            highlightbackground="#E4D3B6",
            highlightthickness=1
        )
        self.image_panel.grid(row=1, column=0, sticky="nsew", padx=(0, 18))
        self.image_panel.grid_propagate(False)
        self.image_panel.grid_rowconfigure(0, weight=1)
        self.image_panel.grid_columnconfigure(0, weight=1)

        self.image_label = tk.Label(
            self.image_panel,
            bg=PANEL_COLOR
        )

        self.image_label.grid(row=0, column=0, padx=14, pady=14)

        self.content_panel = tk.Frame(
            self.main_card,
            bg=CARD_COLOR
        )
        self.content_panel.grid(row=1, column=1, sticky="nsew")
        self.content_panel.grid_columnconfigure(0, weight=1)

        # ====================================================
        # QUESTION LABEL
        # ====================================================

        self.question_label = tk.Label(
            self.content_panel,
            text="",
            font=("Segoe UI", 20, "bold"),
            wraplength=430,
            bg=CARD_COLOR,
            fg=TEXT,
            justify="left",
            anchor="w"
        )

        self.question_label.grid(row=0, column=0, sticky="ew", pady=(8, 18))

        # ====================================================
        # BUTTON CONTAINER
        # ====================================================

        self.button_frame = tk.Frame(
            self.content_panel,
            bg=CARD_COLOR
        )

        self.button_frame.grid(row=1, column=0, sticky="ew")
        self.button_frame.grid_columnconfigure(0, weight=1)
        self.button_frame.grid_columnconfigure(1, weight=1)

        self.buttons = []

        # Create 4 answer buttons
        for i in range(4):

            btn = tk.Button(
                self.button_frame,
                text="",
                font=("Segoe UI", 13, "bold"),
                width=18,
                height=2,
                bg=GREEN,
                fg="white",
                activebackground=GOLD,
                activeforeground=TEXT,
                disabledforeground=MUTED_TEXT,
                relief="flat",
                bd=0,
                cursor="hand2",
                wraplength=180,
                command=lambda idx=i: self.check_answer(idx)
            )

            btn.grid(row=i // 2, column=i % 2, sticky="ew", padx=6, pady=6)

            # Hover effect
            btn.bind("<Enter>", self.on_enter)
            btn.bind("<Leave>", self.on_leave)

            self.buttons.append(btn)

        # ====================================================
        # SCORE LABEL
        # ====================================================

        self.score_label = tk.Label(
            self.content_panel,
            text="Score: 0",
            font=("Segoe UI", 14, "bold"),
            bg=CARD_COLOR,
            fg=ACCENT
        )

        self.score_label.grid(row=2, column=0, sticky="w", pady=(18, 0))

        # ====================================================
        # FEEDBACK PANEL
        # ====================================================

        self.feedback_frame = tk.Frame(
            self.content_panel,
            bg=PANEL_COLOR,
            padx=14,
            pady=12,
            highlightbackground="#E4D3B6",
            highlightthickness=1
        )
        self.feedback_frame.grid(row=3, column=0, sticky="ew", pady=(14, 0))
        self.feedback_frame.grid_columnconfigure(0, weight=1)

        self.feedback_label = tk.Label(
            self.feedback_frame,
            text="Choose an answer to reveal the heritage fact.",
            font=("Segoe UI", 12, "bold"),
            bg=PANEL_COLOR,
            fg=ACCENT_DARK,
            anchor="w",
            justify="left",
            wraplength=410
        )
        self.feedback_label.grid(row=0, column=0, sticky="ew")

        self.fact_label = tk.Label(
            self.feedback_frame,
            text="",
            font=("Segoe UI", 10),
            bg=PANEL_COLOR,
            fg=MUTED_TEXT,
            anchor="w",
            justify="left",
            wraplength=410
        )
        self.fact_label.grid(row=1, column=0, sticky="ew", pady=(6, 0))

        self.next_button = tk.Button(
            self.content_panel,
            text="Next Question",
            font=("Segoe UI", 12, "bold"),
            bg=ACCENT,
            fg="white",
            activebackground=GOLD,
            activeforeground=TEXT,
            relief="flat",
            bd=0,
            cursor="hand2",
            command=self.next_question
        )
        self.next_button.grid(row=4, column=0, sticky="ew", pady=(12, 0))
        self.next_button.grid_remove()

        # Load first question
        self.load_question()

    # ========================================================
    # BUTTON HOVER EFFECTS
    # ========================================================

    def on_enter(self, event):
        if event.widget["state"] == "disabled":
            return
        event.widget["bg"] = GOLD

    def on_leave(self, event):
        if event.widget["state"] == "disabled":
            return
        event.widget["bg"] = GREEN

    # ========================================================
    # LOAD QUESTION
    # ========================================================

    def load_question(self):

        # If no more questions, show results
        if self.q_index >= len(questions):
            self.show_result()
            return

        # Current question
        q = questions[self.q_index]
        self.answer_locked = False
        self.next_button.grid_remove()
        self.feedback_label.config(
            text="Choose an answer to reveal the heritage fact.",
            fg=ACCENT_DARK
        )
        self.fact_label.config(text="")

        # Update progress text
        self.progress_label.config(
            text=f"Question {self.q_index + 1} / {len(questions)}"
        )

        # ----------------------------------------------------
        # IMAGE LOADING
        # ----------------------------------------------------

        share_dir = get_package_share_directory('food_quiz')

        image_path = os.path.join(
            share_dir,
            'images',
            q['image']
        )

        if os.path.exists(image_path):

            img = Image.open(image_path)

            # Keep photos inside the fixed panel so answer buttons stay visible.
            img.thumbnail((330, 290))

            self.photo = ImageTk.PhotoImage(img)

            self.image_label.config(
                image=self.photo,
                text=""
            )

        else:

            self.image_label.config(
                image="",
                text=f"Missing image\n{q['image']}",
                font=("Segoe UI", 18),
                fg=ACCENT
            )

        # Update question text
        self.question_label.config(
            text=q["question"]
        )

        # Update button choices
        for i in range(4):
            self.buttons[i].config(
                text=q["choices"][i],
                bg=GREEN,
                fg="white",
                state="normal"
            )

        choices_text = ". ".join(
            f"Choice {i + 1}: {choice}"
            for i, choice in enumerate(q["choices"])
        )
        self.sound_player.speak(f"Question {self.q_index + 1}. {q['question']} {choices_text}.")

    # ========================================================
    # CHECK ANSWER
    # ========================================================

    def check_answer(self, selected):

        if self.answer_locked:
            return

        self.answer_locked = True
        q = questions[self.q_index]
        correct_index = q["answer"]
        correct_answer = q["choices"][correct_index]

        # Correct answer
        if selected == correct_index:

            self.score += 1
            self.feedback_label.config(text="Correct answer", fg=GREEN)
            self.sound_player.play(self.sound_player.correct_sound)
            self.sound_player.speak(f"Correct. {q['fact']}")

        else:
            self.feedback_label.config(
                text=f"Not quite. Correct answer: {correct_answer}",
                fg=ACCENT
            )
            self.sound_player.play(self.sound_player.wrong_sound)
            self.sound_player.speak(f"Not quite. The correct answer is {correct_answer}. {q['fact']}")

        self.fact_label.config(text=f"Heritage fact: {q['fact']}")

        for i, button in enumerate(self.buttons):
            if i == correct_index:
                button.config(bg=GOLD, fg=TEXT, disabledforeground=TEXT)
            elif i == selected:
                button.config(bg=ACCENT, fg="white", disabledforeground="white")
            else:
                button.config(bg="#D8C7AD", fg=MUTED_TEXT, disabledforeground=MUTED_TEXT)
            button.config(state="disabled")

        # Update score
        self.score_label.config(
            text=f"Score: {self.score}"
        )

        self.next_button.grid()

    # ========================================================
    # NEXT QUESTION
    # ========================================================

    def next_question(self):

        self.q_index += 1

        self.load_question()

    # ========================================================
    # FINAL RESULTS
    # ========================================================

    def show_result(self):

        percentage = (self.score / len(questions)) * 100

        if percentage >= 90:
            rank = "Malaysian Food Master"
        elif percentage >= 70:
            rank = "Heritage Expert"
        elif percentage >= 50:
            rank = "Food Explorer"
        else:
            rank = "Heritage Beginner"

        self.sound_player.play(self.sound_player.finish_sound)
        self.progress_label.config(text="Quiz Complete")
        self.image_label.config(
            image="",
            text="Malaysia's Food Heritage",
            font=("Segoe UI", 20, "bold"),
            fg=ACCENT_DARK
        )
        self.question_label.config(text="Congratulations!")
        self.button_frame.grid_remove()
        self.score_label.config(text=f"Final Score: {self.score}/{len(questions)}")
        self.feedback_label.config(text=f"Achievement: {rank}", fg=GREEN)
        self.fact_label.config(
            text="You have successfully explored traditional Malaysian food heritage."
        )
        self.next_button.config(text="Play Again", command=self.restart_quiz)
        self.next_button.grid()
        self.sound_player.speak(
            f"Quiz complete. Your final score is {self.score} out of {len(questions)}. "
            f"Achievement: {rank}."
        )

    # ========================================================
    # RESTART QUIZ
    # ========================================================

    def restart_quiz(self):

        random.shuffle(questions)
        self.score = 0
        self.q_index = 0
        self.score_label.config(text="Score: 0")
        self.button_frame.grid()
        self.next_button.config(text="Next Question", command=self.next_question)
        self.load_question()


# ============================================================
# START PROGRAM
# ============================================================
class HeritageQuizNode(Node):

    def __init__(self):
        super().__init__('food_quiz')

        self.get_logger().info(
            "Malaysian Food Heritage Quiz Started"
        )
        
def main(args=None):

    rclpy.init(args=args)

    node = HeritageQuizNode()

    root = tk.Tk()
    app = QuizApp(root)

    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
