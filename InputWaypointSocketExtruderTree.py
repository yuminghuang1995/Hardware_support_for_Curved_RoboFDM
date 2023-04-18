# This example shows how to create a program that involves moving axes
# You should use a target to hold the information of the external axes
# The additional joint values will be used to move the external axes in synchronization with the robot
import math
# -*- coding:utf-8 -*-
import os
import glob

#这两个不用要关掉
from numpy import * #导入numpy的库函数
import numpy as np  #这个方式使用numpy的函数时，需要以np.开头。

from operator import itemgetter, attrgetter


from robolink import *    # RoboDK API
from robodk import *      # Robot toolbox
RDK = Robolink()
HOME =RDK.Item('Home')
# get the first robot of Items robot
ROBOT = RDK.Item('', ITEM_TYPE_ROBOT)
if not ROBOT.Valid():
    raise Exception("Robot not selected or available")

# get the parent/master robot. 
# For example, if we selected a synchronized external axis this will return the robot that drives this external axis 
ROBOT = ROBOT.getLink(ITEM_TYPE_ROBOT) #ITEM_TYPE_ROBOT == 2
print("Using robot: " + ROBOT.Name())

FRAME = ROBOT.getLink(ITEM_TYPE_FRAME) # get the linked/active reference frame
TOOL = ROBOT.getLink(ITEM_TYPE_TOOL) # get the linked/active reference tool
# Retrieve the tool pose
tool_pose = TOOL.PoseTool()
# Get the reference pose
frame_pose = ROBOT.PoseFrame()
#print(frame_pose)

# 读取txt,设置文件对象
# txtfile = open("C:/Visualstudio/EBDualGraph/DataSet/waypoint/waypoint.txt", "r")

# 合并多个txt
# 获取目标文件夹的路径
path = "C:/Users/yumin/Desktop/roboDK/waypoint_robot"
# path = "C:/Visualstudio/EBDualGraph/DataSet/results"
# 获取目标文件夹中的文件名称列表
#filenames = os.listdir(path)

#按数字大小排序（否则会排成二进制）
txt_routine = glob.glob(path + "/*.txt")
filenamesini = [x.split("\\")[-1] for x in txt_routine]
filenamesthe = []
filenames = []
for x in filenamesini:
    if 'S' in x:
        tmpxs = x.split("S.")
        x = int(tmpxs[0]), "S"
        filenamesthe.append(x)
    else:
        tmpx = x.split(".")
        x = int(tmpx[0]), " "
        filenamesthe.append(x)
#filenames = [x.split(".")[0] for x in filenames ]
filenamesthe.sort(key = itemgetter(0))
#filenames.sort()
for x in filenamesthe:
    if x[1] == "S":
        filenames.append(str(x[0])+"S.txt")
    else:
        filenames.append(str(x[0])+".txt")

#filenames = [str(x)+".txt" for x in filenames]


# 更改层数，手动变更！！！-------------------------变更！！！-------------------------
start_layer = 1484
end_layer = 1541
# 更改层数，变更！！！-------------------------变更！！！-------------------------
robot_speed = 15.0

mergeLayer = path + "/merge/mergeLayer" + str(start_layer) + str(end_layer) + ".txt"
# 打开当前目录下的文件
file = open(mergeLayer, 'w+', encoding="utf-8")
file.seek(0) #定位到第一行
file.truncate() #清空文件
# 向文件中写入字符, 先遍历文件名
filecounter = 0

for filename in filenames:
    #根据层数读取文件
    if filecounter >= start_layer  and filecounter <= end_layer:
        filepath = path + '/'
        filepath = filepath + filename
        print(filepath)
        # 遍历单个文件，读取行数
        linecounter = 0
        for line in open(filepath, encoding="utf-8"):
            # if linecounter == 0:
            #     #将第一行根据空格分隔开不同的字段
            #     kv = line.split(' ')
            #     #修改第一行抬刀
            #     line = line.replace(kv[7], '1')
            file.writelines(line)
            linecounter += 1
        #file.write('\n')
    filecounter += 1
#关闭文件
file.close()

# Load txtdata as a list of poses, including links to reference and tool frames
def load_targets(strfile):
    data = []
    
    with open(strfile) as f:
        lines = f.readlines() # 读取文本中所有内容，并保存在一个列表中，列表中每一个元素对应一行数据
    counter = 0
    for line in lines:
        line = line.split() #将每一行根据空格分隔开不同的字段
        for i in range(len(line)): #读取每一行长度 
            line[i] = float(line[i])

        # # 读取x y z nx ny nz, y轴朝上
        # x = line[0]
        # # y = -z
        # y = -line[2]
        # # z = oriy
        # z = line[1]
        # #nx = nx
        # nx = line[3]
        # # ny = -nz
        # ny = -line[5]
        # # nz = ny
        # nz = line[4]

        # 读取x y z nx ny nz, z轴朝上，绕z轴逆时针旋转90度
        _coor = np.mat([[line[0]], [line[1]], [line[2]], [1.0]])
        rot_z = np.mat([[cos(-pi/2), -sin(-pi/2),0,0], [sin(-pi/2), cos(-pi/2),0,0], [0,0,1,0], [0,0,0,1]])
        rotcoor = rot_z * _coor 
        x = rotcoor[0]
        y = rotcoor[1]
        z = rotcoor[2]
        n = np.mat([[line[3]], [line[4]], [line[5]], [1.0]])
        rotn = rot_z * n
        nx = rotn[0]
        ny = rotn[1]
        nz = rotn[2]

        # # 读取x y z nx ny nz, z轴朝上（主要使用）
        # x = line[0]
        # y = line[1]
        # z = line[2]
        # nx = line[3]
        # ny = line[4]
        # nz = line[5]

        #print(line)
        # 转换n为r
        rx = -math.degrees(math.asin(ny / math.sqrt(ny**2 + nz**2)))
        ry = math.degrees(math.asin(nx / math.sqrt(nx**2 + ny**2 + nz**2)))
        rz = 0
        # 第一个Jointmove，其他linearmove
        # if counter == 0:
        #     g = 0
        # else:
        #     g = 1

        #挤出信息
        e = line[6]     
        # 计数器，主要给g用
        counter += 1
        #抬刀控制
        h = line[7]

        #喷头Index信息
        g = line[8]

        axissync = float(x), float(y), float(z), float(rx), float(ry), float(rz), float(e), float(h), float(g)
        data.append(axissync)
    #print(data)

    poses = []
    extaxes = []
    idxs = []
    for i in range(0, len(data)):
        x, y, z, rx, ry, rz, e, h, g = data[i][0:9]
        # e挤出信息, h控制抬刀, g控制喷头选择
        extaxis = e, h, g
        #六轴机械臂UR5e的位姿信息
        poses.append(transl(x, y, z) * rotx(rx * pi/180) * roty(ry * pi/180) * rotz(rz * pi/180))
        extaxes.append(extaxis)
        #序列号
        idxs.append(i)
    #print(poses)
    #print(extaxes)
    return poses, extaxes, idxs

#使用此函数进行多轴联动
def load_targets_GUI(strfile):
    poses, extaxes, idxs = load_targets(strfile)
    program_name = getFileName(strfile)
    program_name = program_name.replace('-', '_').replace(' ', '_')
    #如果已经有了这个名字的program，删除该program
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    if program.Valid():
        program.Delete()
    #新建program，初始化各个参数
    program = RDK.AddProgram(program_name, ROBOT)
    # Turn off rendering (faster)
    RDK.Render(False)
    # Speed up by not showing the instruction:
    program.ShowInstructions(False)
    # Remember the speed so that we don't set it with every instruction
    #current_speed = None
    # Very important: Make sure we set the reference frame and tool frame
    program.setFrame(FRAME)
    program.setTool(TOOL)

    #给机械臂设定速度，根据打印的层手动控制，同时变更示教器上的全局变量-------------------------------------！！！
    program.setSpeed(robot_speed)
    #给机械臂设定速度，根据打印的层手动控制，同时变更示教器上的全局变量-------------------------------------！！！

    #圆弧插补
    #program.setRounding(5)
    joint = []
    last_pose = poses[0]
    last_extaxis0 = 0.0
    last_extaxis2 = 0.0
    #counter = 0
    #创建target目标点
    for pose, extaxis, idx in zip(poses, extaxes, idxs):
        name = '%s-%i' % (program_name, idx)
        # # 将F给机械臂的速度，避免重复给相同
        #speed = extaxis[1]
        #if speed != current_speed:
            #program.setSpeed(speed)
            #current_speed = speed

        # # 根据层数手动调整机械臂的速度(使用buffer传输数据无法实现此操作)
        # if filecounter == 31:
        #     program.setSpeed(10)

        # # 如果已经有重名的target，删除重名target
        # target = RDK.Item(name, ITEM_TYPE_TARGET)
        # if target.Valid():
        #     target.Delete()
        # # 创建新的target
        # target = RDK.AddTarget(name, FRAME, ROBOT)
        # target.setPose(pose)
        # # print(pose)

        # 将target坐标转换为6轴机械臂的joint值
        jointi = ROBOT.SolveIK(pose, None, tool_pose, frame_pose)
        #输出的joint
        joint.append(jointi)
        #print(jointi)
        
        try:
            #抬刀，回抽
            if extaxis[1] == 1:

                #'Extruder' + str(extaxis[2]) + "(" + math.round(str(last_extaxis-2.0),3) +")"
                
                program.RunInstruction('Extruder%s(%s)' % (str(round(last_extaxis2)), str("%.3f" % (last_extaxis0-2.0))), INSTRUCTION_COMMENT)
                program.MoveL(transl(0, 0, 70) * last_pose) 
                
                program.RunInstruction('Extruder%s(%s)' % (str(round(extaxis[2])), str("%.3f" % (last_extaxis0))), INSTRUCTION_COMMENT)
                program.MoveL(transl(0, 0, 70) * pose) 
            
            program.RunInstruction('Extruder%s(%s)' % (str(round(extaxis[2])), str("%.3f" %(extaxis[0]))), INSTRUCTION_COMMENT)
            program.MoveL(pose)
            

        except:
            print('Warning: %s can not be reached. It will not be added to the program' % name)

        last_pose = pose
        last_extaxis0 = extaxis[0]
        last_extaxis2 = extaxis[2]
        #counter = counter + 1

    # #回抽
    # program.RunInstruction('Extruder%s(%s)' % (str(round(0)), str("%.3f" % (last_extaxis0))), INSTRUCTION_COMMENT)
    # program.MoveL(transl(0, 0, 35) * last_pose)
    program.MoveL(transl(0, 0, 30) * last_pose)
    program.MoveL(HOME)
    #program.ShowTargets(False)    
    # # If desired, show the program instruction at the end
    #program.ShowInstructions(True)
    return joint

MAKE_GUI_PROGRAM = True

if MAKE_GUI_PROGRAM:
    #主程序
    joint = load_targets_GUI(mergeLayer)
    #导出joint信息
    f = open('C:/Users/yumin/Desktop/roboDK/test_file_joints5.txt','w')
    for jointi in joint:
        s = str(jointi).replace('[','').replace(']','')#去除[],这两行按数据不同，可以选择
        
        s = s.replace("'",'').replace(',','').replace('\n',' ') +'\n' #去除单引号，逗号，每行末尾追加换行符
        f.write(s)
    f.close()

else:
    print('No program running')