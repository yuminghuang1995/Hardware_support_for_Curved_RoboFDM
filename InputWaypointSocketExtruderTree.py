# This example shows how to create a program that involves moving axes
# You should use a target to hold the information of the external axes
# The additional joint values will be used to move the external axes in synchronization with the robot
import math
# -*- coding:utf-8 -*-
import os
import glob

from numpy import * 
import numpy as np 

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

path = "../roboDK/waypoint_robot"

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

start_layer = 1484
end_layer = 1541
robot_speed = 15.0

mergeLayer = path + "/merge/mergeLayer" + str(start_layer) + str(end_layer) + ".txt"

file = open(mergeLayer, 'w+', encoding="utf-8")
file.seek(0) 
file.truncate() 
filecounter = 0

for filename in filenames:
    if filecounter >= start_layer  and filecounter <= end_layer:
        filepath = path + '/'
        filepath = filepath + filename
        print(filepath)
        linecounter = 0
        for line in open(filepath, encoding="utf-8"):
            # if linecounter == 0:
            #     kv = line.split(' ')
            #     line = line.replace(kv[7], '1')
            file.writelines(line)
            linecounter += 1
        #file.write('\n')
    filecounter += 1
file.close()

# Load txtdata as a list of poses, including links to reference and tool frames
def load_targets(strfile):
    data = []
    
    with open(strfile) as f:
        lines = f.readlines() 
    counter = 0
    for line in lines:
        line = line.split() 
        for i in range(len(line)):
            line[i] = float(line[i])

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

        # x = line[0]
        # y = line[1]
        # z = line[2]
        # nx = line[3]
        # ny = line[4]
        # nz = line[5]

        rx = -math.degrees(math.asin(ny / math.sqrt(ny**2 + nz**2)))
        ry = math.degrees(math.asin(nx / math.sqrt(nx**2 + ny**2 + nz**2)))
        rz = 0

        e = line[6]     
        counter += 1
        h = line[7]
        g = line[8]

        axissync = float(x), float(y), float(z), float(rx), float(ry), float(rz), float(e), float(h), float(g)
        data.append(axissync)
    #print(data)

    poses = []
    extaxes = []
    idxs = []
    for i in range(0, len(data)):
        x, y, z, rx, ry, rz, e, h, g = data[i][0:9]
        extaxis = e, h, g
        poses.append(transl(x, y, z) * rotx(rx * pi/180) * roty(ry * pi/180) * rotz(rz * pi/180))
        extaxes.append(extaxis)
        idxs.append(i)
    #print(poses)
    #print(extaxes)
    return poses, extaxes, idxs

def load_targets_GUI(strfile):
    poses, extaxes, idxs = load_targets(strfile)
    program_name = getFileName(strfile)
    program_name = program_name.replace('-', '_').replace(' ', '_')
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    if program.Valid():
        program.Delete()
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

    program.setSpeed(robot_speed)

    #program.setRounding(5)
    joint = []
    last_pose = poses[0]
    last_extaxis0 = 0.0
    last_extaxis2 = 0.0
    for pose, extaxis, idx in zip(poses, extaxes, idxs):
        name = '%s-%i' % (program_name, idx)
        #speed = extaxis[1]
        #if speed != current_speed:
            #program.setSpeed(speed)
            #current_speed = speed
        # if filecounter == 31:
        #     program.setSpeed(10)

        # target = RDK.Item(name, ITEM_TYPE_TARGET)
        # if target.Valid():
        #     target.Delete()
        # target = RDK.AddTarget(name, FRAME, ROBOT)
        # target.setPose(pose)
        # # print(pose)

        jointi = ROBOT.SolveIK(pose, None, tool_pose, frame_pose)
        joint.append(jointi)
        #print(jointi)
        
        try:
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
    joint = load_targets_GUI(mergeLayer)
    f = open('C:/Users/yumin/Desktop/roboDK/test_file_joints5.txt','w')
    for jointi in joint:
        s = str(jointi).replace('[','').replace(']','')
        
        s = s.replace("'",'').replace(',','').replace('\n',' ') +'\n'
        f.write(s)
    f.close()

else:
    print('No program running')
