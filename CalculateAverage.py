import pandas as pd
import argparse

def GetResults():

    for i in ['20', '30', '40', '50']:
        df = pd.read_csv("data/Results/" + args.scene + "/" + args.planner + args.scene + "For" + args.agent + "_" + i + args.UsePrediction + ".txt", sep = " ", usecols=[0, 1, 2, 3], header= None, nrows= 200)
        df.columns = ['success', 'solutionTime', 'distance', 'time']
        df.sort_values(by=['solutionTime'], inplace = True)
        #df.drop(df[df['success'] == 0].index, inplace = True)
        print("The goal is :", i)
        print("Q3:")
        print(df.solutionTime.quantile([0.25,0.5,0.75]))

        print("Median:")
        print(df.solutionTime.median())

        lens = int(0.25 * len(df))
        df = df.iloc[lens: -lens]
        averageTime = df['solutionTime'].mean()
        print("mean:")
        print(df['solutionTime'].mean())

        print("")
        print("")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, default='Random',
                        help="the scene of training data, 'Random' (default), Curves or Maze")
    parser.add_argument('--planner', type=str, default='RRT',
                        help="the planner algorithms")
    parser.add_argument('--goals', type=int, default=10,
                        help="the number of goals")
    parser.add_argument('--UsePrediction', type=str, default="false",
                        help="whether to use prediction")            
    parser.add_argument('--instances', type=int, default=1000,
                        help="")            
    parser.add_argument('--agent', type=str, default="Car",
                        help="the type of agent, car (default), snake or airship")
    args = parser.parse_args()

    GetResults()