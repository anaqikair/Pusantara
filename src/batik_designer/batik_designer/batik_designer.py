#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageDraw, ImageTk
from datetime import datetime
import os, math

class BatikStudio:
    def __init__(self, root):
        self.root = root
        self.root.title("PUSANTARA Batik Studio")
        self.root.geometry("1400x850")

        self.w, self.h = 1000, 700
        self.color = "#000000"
        self.brush = 4
        self.symmetry = "none"
        self.brush_style = "normal"
        self.selected_stamp = None
        self.last = None

        self.image = Image.new("RGB",(self.w,self.h),"white")
        self.draw = ImageDraw.Draw(self.image)

        self.build_ui()

    def build_ui(self):
        top = tk.Label(self.root,text="PUSANTARA BATIK STUDIO",font=("Arial",20,"bold"))
        top.pack(pady=5)

        main = tk.Frame(self.root)
        main.pack(fill="both",expand=True)

        self.canvas = tk.Canvas(main,width=self.w,height=self.h,bg="white")
        self.canvas.pack(side="left",padx=10,pady=10)

        self.canvas.bind("<Button-1>", self.click_canvas)
        self.canvas.bind("<B1-Motion>", self.paint)

        side = tk.Frame(main,width=300)
        side.pack(side="right",fill="y")

        tk.Label(side,text="Colors").pack()
        colors=["#000000","#d62828","#1d4ed8","#16a34a","#9333ea","#f59e0b"]
        row=tk.Frame(side); row.pack()
        for c in colors:
            tk.Button(row,bg=c,width=3,command=lambda cc=c:self.set_color(cc)).pack(side="left")

        tk.Label(side,text="Brush Size").pack()
        s=tk.Scale(side,from_=1,to=20,orient="horizontal",command=self.set_brush)
        s.set(4); s.pack(fill="x")

        ttk.Label(side,text="Brush Style").pack()
        for st in ["normal","dotted","calligraphy"]:
            ttk.Button(side,text=st.title(),command=lambda x=st:self.set_style(x)).pack(fill="x")

        ttk.Label(side,text="Symmetry").pack()
        for st in ["none","horizontal","vertical","four"]:
            ttk.Button(side,text=st.title(),command=lambda x=st:self.set_sym(x)).pack(fill="x")

        ttk.Label(side,text="Motifs").pack(pady=5)
        motifs=["Flower","Lotus","Leaf","Bunga Raya","Awan Larat","Pucuk Rebung"]
        for m in motifs:
            ttk.Button(side,text=m,command=lambda x=m:self.select_stamp(x)).pack(fill="x")

        ttk.Button(side,text="Inspire Me",command=self.inspire).pack(fill="x",pady=4)
        ttk.Button(side,text="Gallery",command=self.gallery).pack(fill="x")
        ttk.Button(side,text="Clear",command=self.clear).pack(fill="x")
        ttk.Button(side,text="Save",command=self.save).pack(fill="x")

    def set_color(self,c): self.color=c
    def set_brush(self,v): self.brush=int(v)
    def set_sym(self,s): self.symmetry=s
    def set_style(self,s): self.brush_style=s
    def select_stamp(self,s): self.selected_stamp=s

    def click_canvas(self,e):
        if self.selected_stamp:
            self.place_stamp(e.x,e.y,self.selected_stamp)
            return
        self.last=(e.x,e.y)

    def line(self,x1,y1,x2,y2):
        width=self.brush
        if self.brush_style=="calligraphy":
            width*=2
        self.canvas.create_line(x1,y1,x2,y2,fill=self.color,width=width,smooth=True)
        self.draw.line((x1,y1,x2,y2),fill=self.color,width=width)

    def paint(self,e):
        if not self.last: return
        x1,y1=self.last
        x2,y2=e.x,e.y

        if self.brush_style=="dotted":
            self.canvas.create_oval(x2,y2,x2+self.brush,y2+self.brush,fill=self.color,outline=self.color)
            self.draw.ellipse((x2,y2,x2+self.brush,y2+self.brush),fill=self.color)
        else:
            self.line(x1,y1,x2,y2)

        if self.symmetry in ["horizontal","four"]:
            self.line(self.w-x1,y1,self.w-x2,y2)
        if self.symmetry in ["vertical","four"]:
            self.line(x1,self.h-y1,x2,self.h-y2)
        if self.symmetry=="four":
            self.line(self.w-x1,self.h-y1,self.w-x2,self.h-y2)

        self.last=(x2,y2)

    def place_stamp(self,x,y,name):
        if name=="Flower": self.flower(x,y)
        elif name=="Lotus": self.lotus(x,y)
        elif name=="Leaf": self.leaf(x,y)
        elif name=="Bunga Raya": self.bungaraya(x,y)
        elif name=="Awan Larat": self.awan(x,y)
        elif name=="Pucuk Rebung": self.rebung(x,y)

    def flower(self,x,y):
        for dx,dy in [(-30,0),(30,0),(0,-30),(0,30),(0,0)]:
            self.canvas.create_oval(x+dx-15,y+dy-15,x+dx+15,y+dy+15,outline=self.color,width=2)

    def lotus(self,x,y):
        for i in range(8):
            a=i*math.pi/4
            self.canvas.create_oval(x+math.cos(a)*25-12,y+math.sin(a)*25-20,
                                    x+math.cos(a)*25+12,y+math.sin(a)*25+20,
                                    outline=self.color,width=2)

    def leaf(self,x,y):
        self.canvas.create_oval(x-40,y-15,x+40,y+15,outline=self.color,width=2)
        self.canvas.create_line(x-40,y,x+40,y,fill=self.color,width=2)

    def bungaraya(self,x,y):
        for i in range(5):
            a=i*2*math.pi/5
            self.canvas.create_oval(x+math.cos(a)*25-18,y+math.sin(a)*25-18,
                                    x+math.cos(a)*25+18,y+math.sin(a)*25+18,
                                    outline=self.color,width=2)

    def awan(self,x,y):
        for i in range(5):
            self.canvas.create_arc(x+i*25,y, x+50+i*25,y+40,start=0,extent=180,style=tk.ARC,width=2)

    def rebung(self,x,y):
        for i in range(5):
            self.canvas.create_polygon(x+i*25,y+50,x+12+i*25,y,x+25+i*25,y+50,
                                       outline=self.color,fill="",width=2)

    def inspire(self):
        self.awan(200,150)
        self.rebung(300,300)
        self.flower(600,250)

    def clear(self):
        self.canvas.delete("all")
        self.image = Image.new("RGB",(self.w,self.h),"white")
        self.draw = ImageDraw.Draw(self.image)

    def save(self):
        d = os.path.expanduser("~/batik_designs")
        os.makedirs(d, exist_ok=True)

        f = os.path.join(
            d,
            "batik_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".png"
        )

        self.canvas.postscript(file=f + ".ps")

        messagebox.showinfo(
            "Saved",
            f"Design saved.\nLocation:\n{f}"
        )

    def gallery(self):
        d = os.path.expanduser("~/batik_designs")
        os.makedirs(d, exist_ok=True)

        files = "\n".join(
            sorted(os.listdir(d))[-20:]
        ) or "No saved designs found."

        messagebox.showinfo(
            "Gallery",
            files
        )


def main():
    root = tk.Tk()
    app = BatikStudio(root)
    root.mainloop()


if __name__ == "__main__":
    main()
