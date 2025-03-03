* COMPILE
cmake -DCMAKE_BUILD_TYPE="Release"
make

* GENERATE PROBLEM INSTANCES

step 1: install octave or matlab

step 2: edit RunGenerateInstances.m. Right now it is configured to generate problem instances for 5 goals, 10 goals, or 15 goals. For each number of goals, it will generate 12 problem instances. You can change goals and nrInstances to whatever numbers you like. For example, nrInstances = 10000 would generate 10000 problem instances.

There are three scenes: Random, Maze, and Curves. The problem instances will be written to data/Instances/ folder. For example, data/Instances/SceneRandomForCarNrGoals5_4.txt is the problem instance with index 4 for the Random scene with 5 goals.

step 3: 
octave --eval RunGenerateInstances

* VISUALIZE THE PROBLEM INSTANCES

If you would like to visualize each problem, run the following:

./bin/Runner GRunMP data/ParamsSceneRandomForCar.txt ParamsExtraFile data/Instances/SceneRandomForCarNrGoals5_4.txt

You can replace Random with Maze or Curves and use whatever NrGoals you have from the instances you generated.

* GENERATE DISTANCES

octave --eval "RunGenerateInstances('Random', nrGoals, nrInstances)"

where 
=> nrGoals is the number of goals (it could be 5, 10, or 15 depending on which number you are using
=> nrInstances is the number of instances

You can replace Random with Maze or Curves.
The script will run the program for all the instances from index 0 to index (nrInstances - 1). For each file, it will write the results in the corresponding distance file. For example, for 
data/Instances/SceneRandomForCarNrGoals5_4.txt it will write the results in 
data/Instances/SceneRandomForCarNrGoals5_4_distances.txt

To test it, you can run it with nrInstances = 1 (so it will just run one instance).

Each distance file has one line:
first it has the x, y coordinates for the initial point, and then the x,y coordinates for each of the goals. After that it has the pairwise distances.


