# r-planner

r-planner is a Python project that implements a sampling-based path planner. The planner is based on the BIT* (Biased Informed Trees) algorithm but incorporates an efficient replanning phase. The replanning phase is designed to reuse old valid samples and progressively shrink the search region, resulting in improved planning performance.

## Features

- Implementation of BIT* algorithm for path planning.
- Efficient replanning phase utilizing old valid samples.
- Progressive shrinking of the search region for focused exploration.

## Usage

You can test r-palnner by following these steps:

1. Clone the repository: https://github.com/nicolazande/r-planner.git
2. Navigate to the project directory: ./
3. Install the required dependencies: matplotlib, scipy
4. Either modify the start and end pose in `replan.py::main()` or import all file classes `from replan import *` and use it as follows:

## Example
In the following section you can find an example of usage:

```python
def main():

    results = []
    tot_iter = 1

    # problem data
    x_start = (5, 5) #start pose
    x_goal = (46, 28) #end pose
    steps = 4 #numer of steps per iteration
    iter = 0 #numer of iterations

    while iter < tot_iter:
        print(iter)
        # create planner
        bit = BITStar(x_start, x_goal, 2, 0)
        # first raw planning
        x_est = bit.planning(steps = steps) 
        # get sampling density
        sample_density = bit.m / bit.calc_dist(bit.x_start, bit.x_goal) 
        start_time = time.time()

        # start simulation
        while x_est != None:

            # move obstacles (no immediate collison)
            bit.moveObstacles(3, 3, x_est, bit.utils.delta)
            
            bit.cMin = bit.calc_dist(x_est, bit.x_goal) #new min cost
            bit.m = round(sample_density * bit.cMin) #new sample number

            # prune old samples
            bit.Tree.V = set(filter(bit.removePrevious, bit.Tree.V))

            # update start position
            bit.x_start = x_est
            bit.x_start.parent = None
            bit.Tree.V.add(bit.x_start)

            # update ellipsoid center
            bit.xCenter = np.array([[(bit.x_start.S[0] + bit.x_goal.S[0]) / 2.0],
                                    [(bit.x_start.S[1] + bit.x_goal.S[1]) / 2.0], [0.0]])

            # update best solution cost
            bit.cBest = bit.g_F[bit.x_goal] - bit.gF(x_est)

            # rotate frame
            bit.RotationToWorldFrame()

            # start replanning
            bit.replan()

            # get position estimate
            x_est = bit.planning(steps)
            
            # sicura
            if not(bit.plot_on or bit.animation_on):
                if time.time()-start_time > 3:
                    break

        results.append(bit.benchmark_list)
        iter = iter + 1

    # save data
    if tot_iter > 10:
        data_dir = os.path.join("../data", "replan.npy")
        np.save(data_dir, results)
```

## Result
In the following sequence of images you can see how the algorithm works:
![1](https://github.com/nicolazande/r-planner/assets/115359494/54dcc3b3-724a-4bdb-a633-22ed3ecf7ae1)
![2](https://github.com/nicolazande/r-planner/assets/115359494/0ea9ee1f-ebce-4a65-8344-11a9e90745cc)
![3](https://github.com/nicolazande/r-planner/assets/115359494/8a15deb5-d4df-453e-9a78-935bf4d81406)
![4](https://github.com/nicolazande/r-planner/assets/115359494/eca1f4a6-acaa-402d-8e0e-debebce43496)
![5](https://github.com/nicolazande/r-planner/assets/115359494/a22e1ae3-4da2-42fc-b9c8-58dafb82b76c)
![6](https://github.com/nicolazande/r-planner/assets/115359494/39d560a2-d753-43b7-986a-870bd9fbda3c)



