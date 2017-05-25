from tkinter import *
import serial
import sys
import glob
import subprocess
import os

def Serial_Transfer(value):
    port = serial.Serial(7, baudrate=9600, timeout=5.0)
    port.write(value)
    
class application(Frame):

    def __init__(self, master):
        Frame.__init__(self, master)
        self.grid()
        self.control()

    def control(self):
        Settings = Label(self, text='Control')
        Settings.grid(row=0, column=0, columnspan=1, rowspan=3, sticky='NW', padx=0, pady=5, ipadx=0, ipady=5)

        column2 = Label(self)
        column2.grid(row=4, column=0, columnspan=1, sticky='NW', padx=0, pady=0, ipadx=0, ipady=0)

        self.scale0 = Scale(Settings, label="Duty Cycle", from_=100, to=0)
        self.scale0.pack(side=LEFT)
        self.scale0.set(50)

        self.button4 = Button(column2, text="Send to Controller", command=self.export, bg="grey")
        self.button4.pack(side=TOP, anchor=N, ipady=2, ipadx=7, pady=3)

    def export(self):
        duty_cycle = str(self.scale0.get())
        Serial_Transfer(duty_cycle)		

if __name__ == "__main__":
	root=Tk()
	root.title("Throttle")
	root.geometry("150x200")
	app=application(root)
	root.mainloop()
