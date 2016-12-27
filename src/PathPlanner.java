/**
 * This Class provides many useful algorithms for Robot Path Planning. It uses optimization techniques and knowledge
 * of Robot Motion in order to calculate smooth path trajectories, if given only discrete waypoints. The Benefit of these optimization
 * algorithms are very efficient path planning that can be used to Navigate in Real-time.
 * <p>
 * This Class uses a method of Gradient Decent, and other optimization techniques to produce smooth Velocity profiles
 * for every wheel of a 4 wheeled swerve drive.
 * <p>
 * This Class does not attempt to calculate quintic or cubic splines for best fitting a curve. It is for this reason, the algorithm can be ran
 * on embedded devices with very quick computation times.
 * <p>
 * The output of this function are independent velocity profiles for the each wheel of a 4 wheeled swerve drivetrain. The velocity
 * profiles start and end with 0 velocity and maintain smooth transitions throughout the path.
 */
public class PathPlanner {
    //The waypoint only path
    private static double[][] origPath;

    //Tuninmg parameters for tuning the path generated
    double pathAlpha;
    double pathBeta;
    double pathTolerance;

    //Tuning paramters for the velocities generated
    double velocityAlpha;
    double velocityBeta;
    double velocityTolerance;

    //The origPath with injected points to allow for smooth transitions
    private double[][] smoothPath;

    /**
     * Constructor, takes a Path of Way Points defined as a double array of column vectors representing robot value
     * such as x y coordinates, or an arm angle
     * <p>
     * For example: here is a properly formated waypoint array
     * <p>
     * double[][] waypointPath = new double[][]{
     * {1, 1, 0},
     * {5, 1, 45},
     * {9, 12, 90},
     * {12, 9, 135},
     * {15,6, 180},
     * {15, 4, 225}
     * };
     * This path goes from {1,1,0} -> {5,1,45} -> {9,12,90} -> {12,9, 135} -> {15,6,180} -> {15,4,225}
     * The units of these coordinates are position units assumed by the user (i.e inch, foot, meters)
     * The units do not have to match between elements.
     *
     * @param path
     */
    public PathPlanner(double[][] path) {
        origPath = ArrayCopy(path);

        //default values
        pathAlpha = 0.7;
        pathBeta = 0.3;
        pathTolerance = 0.0000001;

        velocityAlpha = 0.1;
        velocityBeta = 0.3;
        velocityTolerance = 0.0000001;
    }

    /**
     * Prints Cartesian Coordinates to the System Output as Column Vectors in the Form X	Y
     *
     * @param path
     */
    public static void print(double[][] path) {
        for(int i = 0; i < path.length; i++) {
            for (int j = 0; j < path[i].length; j++) {
                System.out.print(path[i][j] + "\t");
            }
            System.out.print("\n");
        }
    }

    /**
     * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
     * <p>
     * BigO: Order N x M
     *
     * @param arr
     * @return
     */
    public static double[][] ArrayCopy(double[][] arr) {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            System.arraycopy(arr[i], 0, temp[i], 0, arr[i].length);
        }

        return temp;
    }

    //main program
    public static void main(String[] args) {
        //create n dimensional waypoint path
        double[][] waypoints = new double[][]{
                {2, 2, 0, 1},
                {2, 7, 90, 2},
                {2, 12, 180, 3},
                {2, 17, 270, 4},
                {2, 22, 360, 5},
                {7, 22, 450, 6},
                {12, 22, 540,7},
                {17, 22, 630, 6},
                {22, 22, 720, 5},
                {22, 17, 810, 4},
                {22, 12, 900, 3},
                {22, 7, 990, 2},
                {22, 2, 1080, 1}
        };

        double totalTime = 15; //seconds
        double timeStep = .1; //period of control loop on Rio, seconds

        final PathPlanner path = new PathPlanner(waypoints);

        path.setPathAlpha(0.7);
        path.setPathBeta(0.3);
        path.setPathTolerance(0.0000001);
        path.calculate(totalTime, timeStep);

        path.print(path.smoothPath);
    }

    /**
     * Method upsamples the Path by linear injection. The result providing more waypoints along the path.
     * <p>
     * BigO: Order N * injection#
     *
     * @param orig
     * @param numToInject
     * @return
     */
    public double[][] inject(double[][] orig, int numToInject) {
        //create extended 2 Dimensional array to hold additional points
        double[][] morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][orig[0].length];

        int index = 0;

        //loop through original array
        for (int i = 0; i < orig.length - 1; i++) {
            //copy first
            System.arraycopy(orig[i], 0, morePoints[index], 0, orig[i].length);
            index++;

            for (int j = 1; j < numToInject + 1; j++) {
                for (int k = 0; k < orig[i].length; k++) {
                    //calculate intermediate x points between j and j+1 original points
                    morePoints[index][k] = j * ((orig[i + 1][k] - orig[i][k]) / (numToInject + 1)) + orig[i][k];
                }
                index++;
            }
        }

        //copy first
        System.arraycopy(orig[orig.length - 1], 0, morePoints[index], 0, morePoints[index - 1].length);

        return morePoints;
    }

    /**
     * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
     * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
     * converge. If this happens, try increasing the tolerance level.
     * <p>
     * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
     *
     * @param path
     * @param weight_data
     * @param weight_smooth
     * @param tolerance
     * @return
     */
    public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {
        //copy array
        double[][] newPath = ArrayCopy(path);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++) {
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        return newPath;
    }

    /**
     * This method calculates the optimal parameters for determining what amount of nodes to inject into the path
     * to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable
     * results for the final smooth path.
     * <p>
     * Big O: Constant Time
     *
     * @param numNodeOnlyPoints
     * @param maxTimeToComplete
     * @param timeStep
     */
    public int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep) {
        int first = 0;
        int second = 0;
        int third = 0;

        double oldPointsTotal = 0;

        int[] ret;

        double totalPoints = maxTimeToComplete / timeStep;

        if (totalPoints < 100) {
            double pointsFirst;
            double pointsTotal;


            for (int i = 4; i <= 6; i++)
                for (int j = 1; j <= 8; j++) {
                    pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                    pointsTotal = (j * (pointsFirst - 1) + pointsFirst);

                    if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {
                        first = i;
                        second = j;
                        oldPointsTotal = pointsTotal;
                    }
                }

            ret = new int[]{first, second, third};
        } else {

            double pointsFirst;
            double pointsSecond;
            double pointsTotal;

            for (int i = 1; i <= 5; i++)
                for (int j = 1; j <= 8; j++)
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

            ret = new int[]{first, second, third};
        }

        return ret;
    }

    public void setPathAlpha(double alpha) {
        pathAlpha = alpha;
    }

    public void setPathBeta(double beta) {
        pathBeta = beta;
    }

    public void setPathTolerance(double tolerance) {
        pathTolerance = tolerance;
    }

    /**
     * This code will calculate a smooth path based on the program parameters. If the user doesn't set any parameters, the will use the defaults optimized for most cases. The results will be saved into the corresponding
     * class members. The user can then access .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity, .smoothRightVelocity, .smoothLeftVelocity as needed.
     * <p>
     * After calling this method, the user only needs to pass .smoothRightVelocity[1], .smoothLeftVelocity[1] to the corresponding speed controllers on the Robot, and step through each setPoint.
     *
     * @param totalTime - time the user wishes to complete the path in seconds. (this is the maximum amount of time the robot is allowed to take to traverse the path.)
     * @param timeStep  - the frequency at which the robot controller is running on the robot.
     */
    public void calculate(double totalTime, double timeStep) {
        /**
         * pseudo code
         *
         * 1. Reduce input waypoints to only essential (direction changing) node points
         * 2. Calculate how many total datapoints we need to satisfy the controller for "playback"
         * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number
         *    of datapoints, and which follows the waypoint path.
         */

        //Figure out how many nodes to inject
        int[] inject = injectionCounter2Steps(origPath.length, totalTime, timeStep);

        //iteratively inject and smooth the path
        for (int i = 0; i < inject.length; i++) {
            if (i == 0) {
                smoothPath = inject(origPath, inject[0]);
                smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
            } else {
                smoothPath = inject(smoothPath, inject[i]);
                smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
            }
        }
    }
}	
