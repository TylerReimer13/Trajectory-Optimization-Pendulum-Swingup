# Trajectory-Optimization-for-Pendulum-Swingup
Python implementation of trajectory optimization for swinging up an inverted pendulum.

Optimal Trajectory is found by solving a nonlinear programming optimization problem, using actuator effor as the cost function. The dynamics constraints are decomposed into equality constraints by using trapezoidal collocation[1], and the cost function is also discretized. The NLP problem is then solved using scipy-optimize.

[1] "https://epubs.siam.org/doi/pdf/10.1137/16M1062569"

![](https://github.com/TylerReimer13/Trajectory-Optimization-Pendulum-Swingup/blob/main/Pendulum%20Swingup/movie.gif)

![](https://github.com/TylerReimer13/Trajectory-Optimization-Pendulum-Swingup/blob/main/Pendulum%20Swingup/Optimal%20Inputs.png)

![](https://github.com/TylerReimer13/Trajectory-Optimization-Pendulum-Swingup/blob/main/Pendulum%20Swingup/States.png)
