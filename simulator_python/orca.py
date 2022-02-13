from mpl_toolkits.mplot3d import axes3d
import numpy as np
import os
import matplotlib.pyplot as plt
from matplotlib import animation
i=1
agent=8
numberstr=''
number=0
numlist=[]

#获取数据data_orca.txt
with open("data_orca8.txt", "r") as f:
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
x5=[]
y5=[]
z5=[]
x6=[]
y6=[]
z6=[]
x7=[]
y7=[]
z7=[]
x8=[]
y8=[]
z8=[]

for index,val in enumerate(numlist):
    flag = index % (agent*4)
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

    if flag == 16:
        x5.append(val)
    if flag == 17:
        y5.append(val)
    if flag == 18:
        z5.append(val)

    if flag == 20:
        x6.append(val)
    if flag == 21:
        y6.append(val)
    if flag == 22:
        z6.append(val)
    if flag == 24:
        x7.append(val)
    if flag == 25:
        y7.append(val)
    if flag == 26:
        z7.append(val)
    if flag == 28:
        x8.append(val)
    if flag == 29:
        y8.append(val)
    if flag == 30:
        z8.append(val)
#绘图(尝试)
print("hello")
print(len(x1),len(x2),len(x3),len(x4),len(x5))
print("hello")
# print(x1)
# print(y1)
# print(z1)
nametxt1='uav1-position_agent'+str(agent)+'.txt'
nametxt2='uav2-position_agent'+str(agent)+'.txt'
nametxt3='uav3-position_agent'+str(agent)+'.txt'
nametxt4='uav4-position_agent'+str(agent)+'.txt'
nametxt5='uav5-position_agent'+str(agent)+'.txt'
nametxt6='uav6-position_agent'+str(agent)+'.txt'
nametxt7='uav7-position_agent'+str(agent)+'.txt'
nametxt8='uav8-position_agent'+str(agent)+'.txt'
uav1_pos = open(nametxt1,mode='w')
uav2_pos = open(nametxt2,mode='w')
uav3_pos = open(nametxt3,mode='w')
uav4_pos = open(nametxt4,mode='w')
uav5_pos = open(nametxt5,mode='w')
uav6_pos = open(nametxt6,mode='w')
uav7_pos = open(nametxt7,mode='w')
uav8_pos = open(nametxt8,mode='w')

for index,val in enumerate(x8):
    content1 = 'path.push_back(Eigen::Vector3d(' + str(x1[index]) + ',' + str(y1[index]) + ',' + str(z1[index]) + '));'
    content2 = 'path.push_back(Eigen::Vector3d(' + str(x2[index]) + ',' + str(y2[index]) + ',' + str(z2[index]) + '));'
    content3 = 'path.push_back(Eigen::Vector3d(' + str(x3[index]) + ',' + str(y3[index]) + ',' + str(z3[index]) + '));'
    content4 = 'path.push_back(Eigen::Vector3d(' + str(x4[index]) + ',' + str(y4[index]) + ',' + str(z4[index]) + '));'
    content5 = 'path.push_back(Eigen::Vector3d(' + str(x5[index]) + ',' + str(y5[index]) + ',' + str(z5[index]) + '));'
    content6 = 'path.push_back(Eigen::Vector3d(' + str(x6[index]) + ',' + str(y6[index]) + ',' + str(z6[index]) + '));'
    content7 = 'path.push_back(Eigen::Vector3d(' + str(x7[index]) + ',' + str(y7[index]) + ',' + str(z7[index]) + '));'
    content8 = 'path.push_back(Eigen::Vector3d(' + str(x8[index]) + ',' + str(y8[index]) + ',' + str(z8[index]) + '));'
    uav1_pos.write(content1)
    uav2_pos.write(content2)
    uav3_pos.write(content3)
    uav4_pos.write(content4)
    uav5_pos.write(content5)
    uav6_pos.write(content6)
    uav7_pos.write(content7)
    uav8_pos.write(content8)
    uav1_pos.write('\n')
    uav2_pos.write('\n')
    uav3_pos.write('\n')
    uav4_pos.write('\n')
    uav5_pos.write('\n')
    uav6_pos.write('\n')
    uav7_pos.write('\n')
    uav8_pos.write('\n')
uav1_pos.close()
uav2_pos.close()
uav3_pos.close()
uav4_pos.close()
uav5_pos.close()
uav6_pos.close()
uav7_pos.close()
uav8_pos.close()
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
l5 = ax.plot(x5, y5)
l6 = ax.plot(x6, y6)
l7 = ax.plot(x7, y7)
l8 = ax.plot(x8, y8)
dot1, = ax.plot([], [], 'ro')
dot2, = ax.plot([], [], 'ro')
dot3, = ax.plot([], [], 'ro')
dot4, = ax.plot([], [], 'ro')
dot5, = ax.plot([], [], 'ro')
dot6, = ax.plot([], [], 'ro')
dot7, = ax.plot([], [], 'ro')
dot8, = ax.plot([], [], 'ro')
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
def gen_dot5():
    for index,val in enumerate(x5):
        newdot = [val, y5[index]]
        yield newdot
def gen_dot6():
    for index,val in enumerate(x6):
        newdot = [val, y6[index]]
        yield newdot
def gen_dot7():
    for index,val in enumerate(x7):
        newdot = [val, y7[index]]
        yield newdot
def gen_dot8():
    for index,val in enumerate(x8):
        newdot = [val, y8[index]]
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
def update_dot5(newd):
    dot5.set_data(newd[0], newd[1])
    return dot5,
def update_dot6(newd):
    dot6.set_data(newd[0], newd[1])
    return dot6,
def update_dot7(newd):
    dot7.set_data(newd[0], newd[1])
    return dot7,
def update_dot8(newd):
    dot8.set_data(newd[0], newd[1])
    return dot8,

# ani = animation.FuncAnimation(fig, update_dot, frames=gen_dot, interval=100, init_func=init)
ani1 = animation.FuncAnimation(fig, update_dot1, frames=gen_dot1, interval=100)
ani2 = animation.FuncAnimation(fig, update_dot2, frames=gen_dot2, interval=100)
ani3 = animation.FuncAnimation(fig, update_dot3, frames=gen_dot3, interval=100)
ani4 = animation.FuncAnimation(fig, update_dot4, frames=gen_dot4, interval=100)
ani5 = animation.FuncAnimation(fig, update_dot5, frames=gen_dot5, interval=100)
ani6 = animation.FuncAnimation(fig, update_dot6, frames=gen_dot6, interval=100)
ani7 = animation.FuncAnimation(fig, update_dot7, frames=gen_dot7, interval=100)
ani8 = animation.FuncAnimation(fig, update_dot8, frames=gen_dot8, interval=100)
ani1.save('sin_dot.gif', writer='imagemagick', fps=30)

plt.show()





