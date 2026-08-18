# Seengreat Thermal Camera MLX90640 demo codes
# Author(s):Andy Li from Seengreat

import os
import time,threading
import math
import tkinter as tk
from ctypes import *
import numpy as np
from PIL import ImageDraw, Image, ImageFont, ImageTk, ImageFilter

try:
    import rclpy
    from std_msgs.msg import Float32
except ImportError:
    rclpy = None
    Float32 = None

DEV_ADDR  = 0x33
IMG_W     = 24
IMG_H     = 32
DISP_W    = 400
DISP_H    = 320
REFRESH_RATE  = 16  #define refresh rate to 64HZ

mlx = cdll.LoadLibrary('./lib/libmlx90640.so')

img_src = Image.new("RGB", (IMG_W, IMG_H), "RED")
pixels = img_src.load()
img_show = Image.new("RGB", (DISP_W, DISP_H), "BLACK")
image_show = Image.new("RGB", (DISP_W, DISP_H), "BLACK")
font = ImageFont.truetype("./lib/MiSans-Light.ttf",size=20)

window = tk.Tk()
window.title('SEENGREAT')
window.geometry("400x320+0+0")

I2C_DEVICE = "/dev/i2c-1"
thermal_ros_node = None
thermal_max_publisher = None

def publish_max_temperature(value):
    if thermal_max_publisher is not None and math.isfinite(value):
        thermal_max_publisher.publish(Float32(data=float(value)))


def check_i2c_access():
    if not os.path.exists(I2C_DEVICE):
        return False, f"{I2C_DEVICE} 장치가 없습니다. I2C 활성화를 확인하세요."
    if not os.access(I2C_DEVICE, os.R_OK | os.W_OK):
        return False, (
            f"{I2C_DEVICE} 접근 권한이 없습니다. "
            "seeno04 사용자를 i2c 그룹에 추가한 뒤 재로그인하세요."
        )
    return True, ""

def Temp_To_RGB(x, y, v): 
    # Heatmap code borrowed from: http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
    NUM_COLORS = 7  # (1) black, (2) blue, (3) cyan, (4) green, (5) yellow, (6) red, (7) white
    color = [[0,0,0], [0,0,1], [0,1,0], [1,1,0], [1,0,0], [1,0,1], [1,1,1]]
    d_color = []
    idx1 = 0
    idx2 = 0
    fractBetween = 0.0
    vmin = 2.0
    vmax = 55.0
    float(v)
    v = (v-vmin)/(vmax-vmin)
    if(v <= 0): # accounts for an input <=0
        idx1=idx2=0
    elif(v >= 1): # accounts for an input >=0
        idx1=idx2=6
    else:   
        v *= (NUM_COLORS-1)
        idx1 = math.floor(float(v))
        idx2 = idx1+1
        fractBetween = v - float(idx1)
    d_color.append(int((((color[idx2][0] - color[idx1][0]) * fractBetween) + color[idx1][0]) * 255.0)) #red
    d_color.append(int((((color[idx2][1] - color[idx1][1]) * fractBetween) + color[idx1][1]) * 255.0)) #green
    d_color.append(int((((color[idx2][2] - color[idx1][2]) * fractBetween) + color[idx1][2]) * 255.0)) #bule
    pixels[x,y] = tuple(d_color)
    
# mlx90640 will output 32*24 temperature array with chess mode
def Update_image():
    global image_show,img_show,window,label
    data_valid = True
    mlx.Mlx90640_Init(DEV_ADDR, REFRESH_RATE)  # mlx90640 init and set the refresh rate as 16 HZ
    mlx90640To=(c_float*768)()
    p_mlx90640To=pointer(mlx90640To)
    image_show = ImageTk.PhotoImage(img_show)    
    label = tk.Label(window,image=image_show)
    while True:
        minTemp = 100.0
        maxTemp = 0.0        
        mlx.Get_temp_val(DEV_ADDR,p_mlx90640To)
        data_valid = True
        for i in range(768):
            if mlx90640To[i] != mlx90640To[i]: # find NaN value
                data_valid = False
                break
        if data_valid:
            for i in range(768):  # get the minimum and maximum temperature
                if(minTemp > mlx90640To[i]):
                    minTemp = mlx90640To[i]
                if(maxTemp < mlx90640To[i]):
                    maxTemp = mlx90640To[i]
            publish_max_temperature(maxTemp)
            for y in range(IMG_W):  # set pixel color
                for x in range(IMG_H):
                    val = mlx90640To[IMG_H * (IMG_W-1-y) + x]
                    Temp_To_RGB(y, x, val) # update image pixels colors
            img_temp = img_src.filter(ImageFilter.BoxBlur(1.5))  # image blur

            # 카메라를 반시계 방향 90도 설치했으므로
            # 화면은 시계 방향 90도 회전
            img_temp = img_temp.rotate(-90, expand=True)
            # 현재 화면 방향을 기준으로 180도 뒤집기
            img_temp = img_temp.rotate(180, expand=True)

            img_show = img_temp.resize((DISP_W, DISP_H))
            draw = ImageDraw.Draw(img_show)        
            show_text = str.format("Min:%d%s"%(int(minTemp),"°C"))
            draw.text((10,DISP_H-30),show_text,fill=(0,0,255),font=font)# draw minimum temperature
        
            show_text = str.format("Max:%d%s"%(int(maxTemp),"°C"))
            draw.text((DISP_W-100,DISP_H-30),show_text,fill=(0,0,255),font=font) # draw maximum temperature
            
            image_show = ImageTk.PhotoImage(img_show)
            label.configure(image = image_show)
            label.image = image_show # keep a refrence to eliminate screen flicker
            label.place(x=0,y=0)

if __name__ == '__main__':
    print("mlx90640 for python demo")
    if rclpy is not None:
        try:
            rclpy.init(args=None)
            thermal_ros_node = rclpy.create_node("mlx90640_thermal_viewer")
            thermal_max_publisher = thermal_ros_node.create_publisher(
                Float32, "/thermal/max_temperature_c", 10
            )
        except Exception as exc:
            print(f"MLX90640 ROS publisher disabled: {exc}", flush=True)
            thermal_ros_node = None
            thermal_max_publisher = None
    else:
        print("MLX90640 ROS publisher disabled: rclpy unavailable", flush=True)
    i2c_ok, i2c_error = check_i2c_access()
    if i2c_ok:
        thread = threading.Thread(target=Update_image, daemon=True)
        thread.start()
    else:
        print(f"MLX90640 ERROR: {i2c_error}", flush=True)
        error_label = tk.Label(
            window,
            text="MLX90640 연결 오류\n\n" + i2c_error,
            fg="white",
            bg="#111827",
            font=("Sans", 13),
            justify="center",
            wraplength=360,
        )
        error_label.pack(fill="both", expand=True)
    try:
        window.mainloop()
    finally:
        if thermal_ros_node is not None:
            thermal_ros_node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


