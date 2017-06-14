import cairo
import random
from random import randint
from random import uniform
from math import pi as M_PI
import numpy


width = 1920.0
height = 1080.0

# setup a place to draw
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 1920, 1080)
ctx = cairo.Context (surface)
ctx.set_antialias(cairo.ANTIALIAS_SUBPIXEL) #maybe doesn't work?
 
def drawRect(x,y,w,h):
    ctx.rectangle(randint(0,width), randint(0,height), randint(0,100), randint(0,100))
    ctx.stroke() 

def drawEllipse(x,y,w,h):
    ctx.save()
    ctx.translate(x + w/2.0,y + h /2.0)
    ctx.rotate(numpy.deg2rad(randint(0,360)))
    ctx.scale(w/2.0,h/2.0)
    ctx.arc(0.0,0.0,1.0,0.0,2*M_PI)
    ctx.restore()
    ctx.stroke() 

for x in range(5000):
    a = random.uniform(0,1)
    ctx.set_source_rgba(1,0,0,a)
    #drawRect(randint(0,width), randint(0,height), randint(0,100), randint(0,100))
    drawEllipse(uniform(0.001,width),uniform(0.001,height),uniform(0.001,100),uniform(0.001,100))

# finish up
surface.write_to_png('tst.png') # write to file

