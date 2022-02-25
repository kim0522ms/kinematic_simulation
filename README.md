# kinematic_simulation
This simulation draws a kinematic trajectory for the bicycle model. The simulation represents two situations.

![image](https://user-images.githubusercontent.com/33797412/155631111-5238d495-8ab2-4425-814d-a315c476408b.png)

The first is a vehicle that assumes the wheels are already sufficiently turned.
The second is a vehicle in which the wheels receive angular acceleration from 0 degrees and increase slowly, and after reaching the target angle, the vehicle moves at an constant angular velocity.

The first plot shows the trajectory of the vehicle.
The second plot is the derivative of the angular velocity of the vehicle.
The third plot is a graph integrating the total distance traveled by the vehicle.

The integral precision is based on the actual execution time of the code.
