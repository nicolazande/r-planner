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

## Usage

To use P-Planner in your Python project, import the necessary modules and create an instance of the `PPlanner` class. Then, set the planning parameters and call the `plan()` method to generate a path.

```python
from replan import BITStar

planner = BITStar(
 start=(0, 0),
 goal=(10, 10)
)

path = planner.replan()