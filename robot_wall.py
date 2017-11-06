import turtle
import random
from time import sleep
from math import sin
from math import cos

#teta orientation
teta = [0]
#velocity
v = 10

#aux_vars
flag_init = [0]

#position
pos = []


def home(x,y):
    flag_init[0] = 0
    teta[0] = random.randint(0,359) # RANDOM THE TETA
    pos[:] = []	
    turtle.hideturtle()
    turtle.clear()
    turtle.pu()
    turtle.color("blue")
    turtle.goto(0,0)
    turtle.write("Start")
    turtle.title("Circle")
    turtle.onscreenclick(start)
    turtle.mainloop()

def init():
    turtle.clear()
    turtle.pu()
    turtle.speed(0)
    turtle.pensize(20)
    turtle.color("grey")
    turtle.goto(-220,220)
    turtle.pd()
    turtle.goto(220,220)
    turtle.goto(220,-220)
    turtle.goto(-220,-220)
    turtle.goto(-220,220)
    turtle.pu()
    turtle.goto(0,0)

def start(x,y):
    turtle.onscreenclick(None)

    init()

    
    while x > -210 and x < 210 and y > -210 and y <210:
        turtle.listen()
        turtle.setx(turtle.position()[0] + v*cos(teta[0]))
        turtle.sety(turtle.position()[1] + v*sin(teta[0]))
        move()
        x = turtle.xcor()
        y = turtle.ycor()        

    wallhit()



def move():
    turtle.pensize(1)
    turtle.color("blue")
    turtle.pu()
    turtle.shape("circle")
    turtle.stamp()
    x = turtle.xcor()
    y = turtle.ycor()
    if flag_init[0] > 0:     
        turtle.clearstamps(1)
        pos.insert(0,[round(x),round(y)])
        pos.pop(-1)
    else:
        pos.insert(0,[round(x),round(y)])       
        flag_init[0] += 1
    
def wallhit():
    turtle.onscreenclick(None)
    turtle.speed(0)
    turtle.pu()
    turtle.goto(0,150)
    turtle.color("red")
    turtle.write("WALL HIT!",align="center", font=(10))
    turtle.onscreenclick(home)
    turtle.mainloop()
    
if __name__ == '__main__':
    home(0,0)
