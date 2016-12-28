/**
 * pseudo code
 *
 * 1. Reduce input waypoints to only essential (direction changing) node points
 * 2. Calculate how many total datapoints we need to satisfy the controller for "playback"
 * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number
 *    of datapoints, and which follows the waypoint path.
 * numWayPoints - number of waypoints
 * dimWayPoint - dimensions of the waypoints
 * maxTtotalTimeime - the max time allowed completing the path
 * timeStep - the frequency of the motor controller
 */

#include <stdlib.h>
#include <stdio.h>


/**
 * This method calculates the optimal parameters for determining what amount of nodes to inject into the path
 * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
 * results for the final smooth path.
 * Big O: Constant Time
 */
int* injectionCounter2Steps(float numNodeOnlyPoints, float maxTimeToComplete, float timeStep)
{
    int first = 0;
    int second = 0;
    int third = 0;

    float oldPointsTotal = 0;

    int* ret = malloc(3*sizeof(int));

    float totalPoints = maxTimeToComplete / timeStep;

    if (totalPoints < 100)
    {
        float pointsFirst;
        float pointsTotal;


        for (int i = 4; i <= 6; i++) {
            for (int j = 1; j <= 8; j++) {
                pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                pointsTotal = (j * (pointsFirst - 1) + pointsFirst);

                if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal)
                {
                    first = i;
                    second = j;
                    oldPointsTotal = pointsTotal;
                }
            }
        }
    } else {
        float pointsFirst;
        float pointsSecond;
        float pointsTotal;

        for (int i = 1; i <= 5; i++) {
            for (int j = 1; j <= 8; j++) {
                for (int k = 1; k < 8; k++) {
                    pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                    pointsSecond = (j * (pointsFirst - 1) + pointsFirst);
                    pointsTotal = (k * (pointsSecond - 1) + pointsSecond);

                    if (pointsTotal <= totalPoints) {
                        first = i;
                        second = j;
                        third = k;
                    }
                }
            }
        }
    }

    ret[0] = first;
    ret[1] = second;
    ret[2] = third;

    return ret;
}

/**
 * Method upsamples the Path by linear injection. The result providing more waypoints along the path.
 * BigO: Order N * injection#
 */
float** inject(int numWayPoints, int dimWayPoint, float** orig, int numToInject) {
    //create extended n Dimensional array to hold additional points
    int newNumWayPoints = numWayPoints + ((numToInject) * (numWayPoints- 1));
    float** morePoints = malloc(newNumWayPoints * sizeof(float*));
    for (int i = 0; i < newNumWayPoints; i++) {
      morePoints[i] = malloc(dimWayPoint * sizeof(float));
    }

    int index = 0;

    //loop through original array
    for (int i = 0; i < numWayPoints - 1; i++) {
        //copy first

        for(int j = 0; j < dimWayPoint; j++) {
            morePoints[index][j] = orig[i][j];
        }

        index++;

        for (int j = 1; j < numToInject + 1; j++) {
            for (int k = 0; k < dimWayPoint; k++) {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][k] = j * ((orig[i + 1][k] - orig[i][k]) / (numToInject + 1)) + orig[i][k];
            }
            index++;
        }
    }

    //copy last
    for(int j = 0; j < dimWayPoint; j++) {
        morePoints[index][j] = orig[numWayPoints-1][j];
    }

    return morePoints;
}

/**
 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
 * converge. If this happens, try increasing the tolerance level.
 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
 */
float** smoother(int numWayPoints, int dimWayPoint, float** path, float weight_data, float weight_smooth, float tolerance) {
    //copy array
    float**  newPath = malloc(numWayPoints * sizeof(float*));
    for (int i = 0; i < numWayPoints; i++) {
        newPath[i] = malloc(dimWayPoint * sizeof(float));
    }

    for(int i = 0; i < numWayPoints; i++) {
        for(int j = 0; j < dimWayPoint; j++) {
            newPath[i][j] = path[i][j];
        }
    }

    double change = tolerance;
    while (change >= tolerance) {
        change = 0.0;
        for (int i = 1; i < numWayPoints - 1; i++) {
            for (int j = 0; j < dimWayPoint; j++) {
                double aux = newPath[i][j];
                newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                change += abs(aux - newPath[i][j]);
            }
        }
    }

    return newPath;
}


float** smoothPath(int numWayPoints, int dimWayPoint, float** wayPoints, float totalTime, float timeStep)
{
    //the path we are going to return
    float** path;

    //important values used for smoothing
    float pathAlpha = 0.7;
    float pathBeta = 0.3;
    float pathTolerance = 0.0000001;

    int numPoints = numWayPoints;
    int* injections = injectionCounter2Steps(numWayPoints, totalTime, timeStep);
    //iteratively inject and smooth the path
    for (int i = 0; i < 3; i++) {
        if (i == 0) {
            path = inject(numPoints, dimWayPoint, wayPoints, injections[0]);

            //update the number of points in our path before proceeding
            numPoints = numPoints + ((injections[i]) * (numPoints- 1));

            path = smoother(numPoints, dimWayPoint, path, pathAlpha, pathBeta, pathTolerance);
        } else {
            path = inject(numPoints, dimWayPoint, path, injections[i]);

            //update the number of points in our path before proceeding
            numPoints = numPoints + ((injections[i]) * (numPoints- 1));

            path = smoother(numPoints, dimWayPoint, path, pathAlpha, pathBeta, pathTolerance);
        }
    }
    //shift all the points down 1 and store the length in [0][0], including
    //the added point.
    float** finalPath = malloc((numPoints + 1) * sizeof(float*));
    for(int i = 0; i < numPoints + 1; i++) {
      finalPath[i] = malloc(dimWayPoint * sizeof(float));
    }

    finalPath[0][0] = numPoints + 1;

    for(int i = 0; i < numPoints; i++) {
        for(int j = 0; j < dimWayPoint; j++) {
            finalPath[i+1][j] = path[i][j];
        }
    }

    //clean up memory
    for (int i = 0; i < numPoints; i++) {
      free(path[i]);
    }
    free(path);

    return finalPath;
}

void test()
{
    int num_points = 5;
    int dim_points = 2;
    float** points = malloc(num_points * sizeof(float*));
    for (int i = 0; i < num_points; i++) {
      points[i] = malloc(dim_points * sizeof(float));
    }

    points[0][0] = 1;
    points[0][1] = 2;
    points[1][0] = 2;
    points[1][1] = 7;
    points[2][0] = 4;
    points[2][1] = 7;
    points[3][0] = 6;
    points[3][1] = 9;
    points[4][0] = 10;
    points[4][1] = 11;

    float totalTime = 15;
    float timeStep = .1;

    float** sPath;
    sPath = smoothPath(num_points, dim_points, points, totalTime, timeStep);

    for(int i = 1; i < sPath[0][0]; i++) {
        for(int j = 0; j < dim_points; j++) {
            printf("%f ", sPath[i][j]);
        }
        printf("\n");
    }
    //clean up memory
    for (int i = 0; i < num_points; i++) {
      free(points[i]);
    }
    free(points);

    for (int i = 0; i < sPath[9][0]; i++) {
      free(sPath[i]);
    }
    free(sPath);
}

int main()
{
    test();
    return 0;
}
