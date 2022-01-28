from mpl_toolkits.mplot3d import axes3d
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib import animation
i=1
numberstr=''
number=0
numlist=[]

#获取数据data_orca.txt
with open("data_orca.txt", "r") as f:
    for line in f.readlines():
        # line = line.strip()
        # print(i)
        # print(line)
        # print(len(line))
        for word in line:
            if word !='\t' and word !='\n' and word != ' ':
                numberstr=numberstr+word
            else:
                if numberstr == '':
                    continue
                else:
                    number=float(numberstr)
                    numberstr = ''
                    numlist.append(number)
        i=i+1
print(len(numlist))
# print(numlist)
x1=[]
y1=[]
z1=[]
x2=[]
y2=[]
z2=[]
x3=[]
y3=[]
z3=[]
x4=[]
y4=[]
z4=[]
for index,val in enumerate(numlist):
    flag = index % 16
    # print(index,val,flag)
    if flag == 0:
        x1.append(val)
    if flag == 1:
        y1.append(val)
    if flag == 2:
        z1.append(val)

    if flag == 4:
        x2.append(val)
    if flag == 5:
        y2.append(val)
    if flag == 6:
        z2.append(val)

    if flag == 8:
        x3.append(val)
    if flag == 9:
        y3.append(val)
    if flag == 10:
        z3.append(val)
    if flag == 12:
        x4.append(val)
    if flag == 13:
        y4.append(val)
    if flag == 14:
        z4.append(val)

#绘图(尝试)
print(x1)
print(y1)
print(z1)
# new a figure and set it into 3d
# fig1 = plt.figure()
# ax = fig1.gca(projection='3d')
#
# # set figure information
# ax.set_title("3D_Curve")
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
#
# # draw the figure, the color is r = read
# figure1 = ax.plot(x1, y1, z1)
# figure2 = ax.plot(x2, y2, z2)
# figure3 = ax.plot(x3, y3, z3)
# figure4 = ax.plot(x4, y4, z4)
# plt.show()
#
# #2D
# plt.plot(x1,y1)
# plt.plot(x2,y2)
# plt.plot(x3,y3)
# plt.plot(x4,y4)
# plt.show()
fig, ax = plt.subplots()

l = ax.plot(x1, y1)
l2 = ax.plot(x2, y2)
l3 = ax.plot(x3, y3)
l4 = ax.plot(x4, y4)
dot1, = ax.plot([], [], 'ro')
dot2, = ax.plot([], [], 'ro')
dot3, = ax.plot([], [], 'ro')
dot4, = ax.plot([], [], 'ro')
def init():
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    return l

#generate
def gen_dot1():
    for index,val in enumerate(x1):
        newdot = [val, y1[index]]
        yield newdot
def gen_dot2():
    for index,val in enumerate(x2):
        newdot = [val, y2[index]]
        yield newdot
def gen_dot3():
    for index,val in enumerate(x3):
        newdot = [val, y3[index]]
        yield newdot
def gen_dot4():
    for index,val in enumerate(x4):
        newdot = [val, y4[index]]
        yield newdot

#update
def update_dot1(newd):
    dot1.set_data(newd[0], newd[1])
    return dot1,
def update_dot2(newd):
    dot2.set_data(newd[0], newd[1])
    return dot2,
def update_dot3(newd):
    dot3.set_data(newd[0], newd[1])
    return dot3,
def update_dot4(newd):
    dot4.set_data(newd[0], newd[1])
    return dot4,


# ani = animation.FuncAnimation(fig, update_dot, frames=gen_dot, interval=100, init_func=init)
ani1 = animation.FuncAnimation(fig, update_dot1, frames=gen_dot1, interval=100)
ani2 = animation.FuncAnimation(fig, update_dot2, frames=gen_dot2, interval=100)
ani3 = animation.FuncAnimation(fig, update_dot3, frames=gen_dot3, interval=100)
ani4 = animation.FuncAnimation(fig, update_dot4, frames=gen_dot4, interval=100)
ani1.save('sin_dot.gif', writer='imagemagick', fps=30)

plt.show()





