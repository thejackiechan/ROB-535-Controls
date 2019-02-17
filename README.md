# ROB535 Controls Project

Team members: Alejandro Azocar, Jackie Chan, Ryan Kim, Paul List

## Part 1: Following a trajectory with no obstacles
Using the trajectory information found in starterfiles (in TestTrack.mat) as well as assuming Pacejka tire dynamics, we designed a PID controller for longitudinal force and steering. Force inputs were commanded to maintain constant velocity while steering inputs were given to minimize cross-tracking error (vehicle distance from centerline). We had previously attempted to use fmincon for nonlinear trajectory optimization, but was unsuccessful in our endeavors. MPC was implemented to provide corrective feedback gains for our PID trajectory, though it depended on having a very good reference trajectory to work. We realized that forward integrating using ode45/Runge Kutta (rather than Euler integration) with the appropriate tolerances yielded a feasible trajectory that worked well with PID alone.

## Part 2: Trajectory planning with obstacles
Essentially the same as part 1, but with additional logic statements to avoid obstacles generated randomly at run-time. Again, PID with steering based on cross-tracking error proved to be successful on its own. My future plans are to reimplement this with MPC using our better trajectories obtained via forward integration using ode45.

### Project specifications can be found in ROB535_Control_Project.pdf
