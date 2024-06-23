
import tkinter as tk

class StoplightGUI:
    def __init__(self, update_callback):
        self.root = tk.Tk()
        self.root.title("Stoplight Timer")

        self.canvas = tk.Canvas(self.root, width=200, height=200)
        self.canvas.pack()

        self.stoplight = self.canvas.create_oval(50, 50, 150, 150, fill='red')

        self.timer_text = tk.StringVar()
        self.timer_label = tk.Label(self.root, textvariable=self.timer_text)
        self.timer_label.pack()

        self.timer_text.set("Time: 0.0 sec")

        self.update_callback = update_callback
        self.root.after(100, self.check_ros) 

    def check_ros(self):
        self.update_callback()
        self.root.after(100, self.check_ros)

    def update_display(self, stop, left_time):
        if stop:
            self.canvas.itemconfig(self.stoplight, fill='red')
        else:
            self.canvas.itemconfig(self.stoplight, fill='green')

        self.timer_text.set(f"Time: {left_time:.1f} sec")

    def run(self):
        self.root.mainloop()

