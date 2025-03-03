import os
import argparse

def GenerateResults():
    path = 'data/Instances/' + args.scene + args.agent + 'NrGrids' + str(args.grid)
    if not os.path.exists(path):
        return None

    if os.path.exists('data/Results/' + args.planner + args.scene + 'For' + args.agent + '_' + str(
                args.grid) + '.txt'):
        os.remove('data/Results/' + args.planner + args.scene + 'For' + args.agent + '_' + str(
                args.grid) + '.txt')

    i = 0
    while (i <= args.instances):

        with open('data/ParamsScene' + args.scene + 'ForCar.txt') as file:
            f = file.readlines()
            f[7] = "   ObstaclesPolygonsFile data/Obstacles/" + args.scene + "_" + str(args.grid) + "/" + str(i) + ".txt\n"

        with open('data/ParamsScene' + args.scene + 'ForCar_tmp.txt', 'w') as file:
            file.writelines(f)

        fname = path + "/" + str(i) + ".txt"
        generated = False
        count = 0
        while generated == False and count <= 0:
            cmd = './bin/Runner RunPlanner data/ParamsScene' + args.scene + 'ForCar_tmp.txt ParamsExtraFile ' + fname + ' UseMP ' + args.planner + ' UseMap ' + args.scene + ' PlannerStatsFile data/Results/' + args.planner + args.scene + 'For' + args.agent + '_' + str(
                args.grid) + '.txt'
            a = os.system(cmd)
            if a != 0:
                count = count + 1
                print('...trying again for <%s> <count = %d>\n', fname, count)
            else:
                generated = True

        if generated == False:
            print('...cannot generate instance with cmd <%s>\n')
        
        i += 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Curves',
                        help="the scene of training data, 'Random' (default), Curves or Maze")
    parser.add_argument('--planner', type=str, default='RRT',
                        help="the planner algorithms")
    parser.add_argument('--grid', type=int, default=3,
                        help="")
    parser.add_argument('--instances', type=int, default=400,
                        help="")            
    parser.add_argument('--agent', type=str, default="Car",
                        help="the type of agent, car (default), snake or airship")
    args = parser.parse_args()

    GenerateResults()