# MotionPlanningRRTCourse
Motion Planning using Tree Algorithms - Udemy Course Assignments

Assignments for the Motion Planning Tree Algorithms Course on Udemy
Please download all the files, the cSpace.npy is used by all assignments.

# My Corrections
- Run the python files through terminal. New plots are generated iteratively in Spyder/
- `ellipseSample.py` - Add `plt.show()` at the end
- `assignment0.py` - Add `plt.show()` after `plt.plot(....)` in line 73
- `assignment1.py` 
	- In the function `isInObstacle()`, change `self.grid([round(testPoint[1]).astype(int64), round(testPoint[0]).astype(int64)] == 1:` to `self.grid([round(testPoint[1]), round(testPoint[0])] == 1:` 
	- In `addChild` function, the condition under `if` statement for when the goal is found is changed to `if(locationX == self.goal.locationX and locationY == self.goal.locationY):`
	- Add `plt.show()` at the end for the figure to persist after code execution