import os
import argparse

def RunGenerateDistances():
    path = 'data/Instances/' + args.scene + args.agent + 'NrGoals' + str(args.goal)
    if not os.path.exists(path + '_distance'):
        os.makedirs(path + '_distance')
    for i in range(args.instances):
        fname = path + '/' + str(i) + '.txt'
        if os.path.exists(fname):
            cmd = './bin/Runner RunDistance data/ParamsScene' + args.scene + 'ForCar.txt ParamsExtraFile ' + fname + ' OutputFile ' + path + '_distance/' + str(i) + '_distances.txt'
            os.system(cmd)
        
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Curves',
                        help="the scene of training data, 'Random' (default), Curves or Maze")
    parser.add_argument('--goal', type=int, default=20,
                        help="the number of goals")
    parser.add_argument('--instances', type=int, default=100,
                        help="the number of instances")
    parser.add_argument('--velScaleConv', type=int, default=1)
    parser.add_argument('--agent', type=str, default="Car",
                        help="the type of agent, car (default), snake or airship")
    args = parser.parse_args()

    RunGenerateDistances()
