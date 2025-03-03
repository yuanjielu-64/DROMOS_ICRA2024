import numpy as np
import pandas as pd
import elkai
import argparse

def obstcale_distance(nodes_coord, dis):
    node = np.zeros([len(nodes_coord) + 1, len(nodes_coord) + 1])
    m = 0
    for j in range(0, len(nodes_coord)):
        for k in range(j + 1, len(nodes_coord)):
            node[j][k] = dis[m]
            m += 1
    node += node.T
    return node

def main(configs):
    len_train = int(configs.NumInstance)

    dis_train = []

    for i in range(0, len_train):
        df = np.loadtxt("data/Instances/SceneCurvesForCarNrGoals" + str(configs.NumGoals) + "_" + str(i) +"_distances.txt")
        if len(df) != 0:
            dis_train.append(df)
    dis_train = pd.DataFrame(dis_train)
    dis_train.to_csv("data/motion_data/train.csv", index = False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", default='Random', help="random, Maze or Curves")
    parser.add_argument("--NumGoals", type=int, default= 20, help="the number of goals, like 0~N")
    parser.add_argument("--NumInstance", type=int, default= 20, help="the number of instances")
    configs = parser.parse_args()
    main(configs)

