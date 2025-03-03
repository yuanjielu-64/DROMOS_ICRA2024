import os
import argparse
import pandas as pd
import random

def RunGenerateInstancePerScene(goalPoints):
    path = 'data/Instances/' + args.scene + args.agent + 'NrGrids' + str(args.grid)
    if not os.path.exists(path):
        os.makedirs(path)

    i = 0
    while (i <= args.instances):
        fname = path + '/' + str(i) + '.txt'
        generated = False
        count = 0

        with open('data/ParamsScene' + args.scene + 'ForCar.txt') as file:
            f = file.readlines()
            f[7] = "   ObstaclesPolygonsFile data/Obstacles/" + args.scene + "_" + str(args.grid) + "/" + str(i) + ".txt\n"

        with open('data/ParamsScene' + args.scene + 'ForCar_tmp.txt', 'w') as file:
            file.writelines(f)

        while generated == False and count < 2:
            if os.path.exists(fname):
                os.remove(fname)

            cmd = './bin/Runner GenerateInstances ' + 'data/ParamsScene' + args.scene + 'ForCar_tmp.txt ' + \
                  'GenerateInstancesNrGrids ' + str(args.grid) + ' GenerateInstancesWriteToFile ' + \
                  fname + ' VelocityScaleConversion ' + str(args.velScaleConv) + ' GoalPoints ' + goalPoints

            os.system(cmd)
            if os.path.exists(fname):
                file = open(fname, 'r').readlines()
                if len(file) > 1:
                    generated = True

            count += 1
            if generated == False:
                print('...trying again for <%s> <count = %d>\n', fname, count)
        
        if generated == False:
            print('...cannot generate instance with cmd <%s>\n')
        else:
            i += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Maze',
                        help="the scene of training data, 'Random' (default), Curves or Maze")
    parser.add_argument('--grid', type=int, default=3,
                        help="")
    parser.add_argument('--instances', type=int, default=400,
                        help="the number of instances")
    parser.add_argument('--velScaleConv', type=int, default=1)
    parser.add_argument('--agent', type=str, default="Car",
                        help="the type of agent, car (default), snake or airship")
    args = parser.parse_args()

    goalPoints = "data/goalPosition/" + args.scene + "Points_" + str(args.grid) + ".txt"

    RunGenerateInstancePerScene(goalPoints)