import tkinter as tk
from tkinter import scrolledtext
import serial
import threading

def read_serial():
    while ser.is_open:
        try:
            data = ser.readline().decode('utf-8').strip()
            if data:
                output_text.insert(tk.END, "接收到数据：" + data + '\n')
                output_text.see(tk.END)
        except:
            pass

def send_data():
    data = input_entry.get()
    if ser.is_open:
        # 将输入映射为指令
        if data.isdigit():
            command_to_send = f"#000P{data}T1000!"
        else:
            command_to_send = data

        # 发送指令
        ser.write((command_to_send + '\n').encode('utf-8'))
        output_text.insert(tk.END, "已发送指令：" + command_to_send + '\n')
        output_text.see(tk.END)

         #自动发送“#000PRAD!”
        ser.write("#000PRAD!\n".encode('utf-8'))
        output_text.insert(tk.END, "已发送指令：#000PRAD!\n")
        output_text.see(tk.END)
    else:
        output_text.insert(tk.END, "串口未打开\n")
        output_text.see(tk.END)

def open_serial():
    try:
        ser.port = port_entry.get()
        ser.baudrate = int(baudrate_entry.get())
        ser.open()
        output_text.insert(tk.END, "串口已打开\n")
        output_text.see(tk.END)
        threading.Thread(target=read_serial, daemon=True).start()
    except Exception as e:
        output_text.insert(tk.END, "无法打开串口：" + str(e) + '\n')
        output_text.see(tk.END)

def close_serial():
    if ser.is_open:
        ser.close()
        output_text.insert(tk.END, "串口已关闭\n")
        output_text.see(tk.END)

# 配置串口
ser = serial.Serial()

# 创建主窗口
window = tk.Tk()
window.title("串口通信")

# 串口配置区域
config_frame = tk.Frame(window)
config_frame.pack(pady=5)

tk.Label(config_frame, text="串口号：").grid(row=0, column=0)
port_entry = tk.Entry(config_frame)
port_entry.grid(row=0, column=1)
port_entry.insert(0, 'COM9')  # 默认串口号，可根据需要修改

tk.Label(config_frame, text="波特率：").grid(row=1, column=0)
baudrate_entry = tk.Entry(config_frame)
baudrate_entry.grid(row=1, column=1)
baudrate_entry.insert(0, '115200')  # 默认波特率，可根据需要修改

open_button = tk.Button(config_frame, text="打开串口", command=open_serial)
open_button.grid(row=2, column=0, pady=5)

close_button = tk.Button(config_frame, text="关闭串口", command=close_serial)
close_button.grid(row=2, column=1)

# 数据发送区域
send_frame = tk.Frame(window)
send_frame.pack(pady=5)

tk.Label(send_frame, text="输入发送的数据：").grid(row=0, column=0)
input_entry = tk.Entry(send_frame, width=30)
input_entry.grid(row=0, column=1)

send_button = tk.Button(send_frame, text="发送", command=send_data)
send_button.grid(row=0, column=2, padx=5)

# 输出显示区域
output_text = scrolledtext.ScrolledText(window, width=50, height=15)
output_text.pack(pady=5)

window.mainloop()