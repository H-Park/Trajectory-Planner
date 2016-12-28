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
