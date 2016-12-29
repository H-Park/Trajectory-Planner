State-Space Trajectory Planner

Given N dimensional sequential waypoints, this code will develop a n dimensional path that yields
a smooth transition between states. 

## C

Declare a 2d array of your original way points

```bash
    int num_points = 5;
    int dim_points = 2;
    float** points = malloc(num_points * sizeof(float*));
    for (int i = 0; i < num_points; i++) {
      points[i] = malloc(dim_points * sizeof(float));
    }
    
    //populate points with values here....
```

then the maximum time you want this path to be completed in, as well as your motor controller's frequency

```bash
    float totalTime = 15;
    float timeStep = .1;
```

Call smoothPath, which returns a 2d array (float) of your path, that starts at [1][0].
)[0][0] stores the length of the pointer, which includes the first slot.

```bash
    float** sPath = smoothPath(num_points, dim_points, points, totalTime, timeStep);
```

to read the values

```bash
        for(int i = 1; i < sPath[0][0]; i++) {
            for(int j = 0; j < dim_points; j++) {
            //do something with sPath[i][j]
            }
        }
```

## Java

Declare a 2d array of your original way points

```bash
        double[][] waypoints = new double[][]{
                {1, 2},
                {2, 7},
                {4, 7},
                {6, 9},
                {10, 11}
        };
```


then the maximum time you want this path to be completed in, as well as your motor controller's frequency

```bash
    double totalTime = 15;
    double timeStep = .1;
```

Construct a new PathPlanner class, set optimization parameters (optional), then call .calculate to generate the 
smoothPath.

```bash
        final PathPlanner path = new PathPlanner(waypoints);

        path.setPathAlpha(0.7);
        path.setPathBeta(0.3);
        path.setPathTolerance(0.0000001);
        path.calculate(totalTime, timeStep);
```

The smooth path will be stored in 

```bash
    path.smoothPath
```
