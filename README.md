# r-planner

r-planner is a Python project that aims to implement a sampling-based path planner. The planner is based on the BIT* (Biased Informed Trees) algorithm but incorporates an efficient replanning phase. The replanning phase is designed to reuse old valid samples and progressively shrink the search region, resulting in improved planning performance.

## Features

- Implementation of BIT* algorithm for path planning.
- Efficient replanning phase utilizing old valid samples.
- Progressive shrinking of the search region for focused exploration.

## Installation

You can install P-Planner by following these steps:

1. Clone the repository: https://github.com/nicolazande/r-planner.git
2. Navigate to the project directory: ./
3. Install the required dependencies: matplotlib, scipy

## Example
In the following sequence of images you can see how the algorithm works:
![1](https://github.com/nicolazande/r-planner/assets/115359494/54dcc3b3-724a-4bdb-a633-22ed3ecf7ae1)
![2](https://github.com/nicolazande/r-planner/assets/115359494/0ea9ee1f-ebce-4a65-8344-11a9e90745cc)
![3](https://github.com/nicolazande/r-planner/assets/115359494/8a15deb5-d4df-453e-9a78-935bf4d81406)
![4](https://github.com/nicolazande/r-planner/assets/115359494/eca1f4a6-acaa-402d-8e0e-debebce43496)
![5](https://github.com/nicolazande/r-planner/assets/115359494/a22e1ae3-4da2-42fc-b9c8-58dafb82b76c)
![6](https://github.com/nicolazande/r-planner/assets/115359494/39d560a2-d753-43b7-986a-870bd9fbda3c)


## Usage

To use P-Planner in your Python project, import the necessary modules and create an instance of the `PPlanner` class. Then, set the planning parameters and call the `plan()` method to generate a path.

```python
from replan import BITStar

planner = BITStar(
 start=(0, 0),
 goal=(10, 10)
)

path = planner.replan()
