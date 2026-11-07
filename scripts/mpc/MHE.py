'''
Moving horizon estimation MHE receding horizon esimtation

estimate the internal state of the system from the measurmeent of the input and output of the system!--> state observer (estimator)
this input and ooutput could be noisy . other wise if the system is also linear we can use full order observer 
if output and input noisy and sys is linear -->  KF
if the system is non linear --> EKF - UKF etc


MHE does the same job but its based on optimization --> estimates fits the measurement using optimization


SISO:

    x(k+1) = f(x(k), u(k)) + vx,
    y(k) = h(x(k)) + vy   --> h is new equation which is for output (sensor)


    EX --> y(k) = x(k) + vy --> sensors measure directly your state because you used x(k) as the output!


    vx and vy are noise on the system and sensor



    in MHE we look backward for length of N_MHE and we look at all the measurement and control action in that window
    we know what control command (or measure using imu/odometry etc) we sent and what was the measuremnt

    then find best fit for input and best fit for output 
    so we find best state trjaecotyr that best fit the considered window of measuremnet!

    cost function adn constraint are some how the same!
    ybar is measured output and h(x(u)) is the predicted output! ubar is the measured input and the u(i) is the predicted input!
    J(x,u) = sum from i=K-N to K of (ybar(i)-h(x(u)))^2 + sum from K-N to K-1 of (ubar(i)-u(i))^2 --> with weight matrices V and W!

    mind that in above the first sum is one step longer thant the sum for the input because we are intereseted in measuring the next state which is depdentat on the previous control (at step k)

    min J(x,u)

    s.t. x(i+1) = f(x(i,u(i))) --> prediction step --> this is system dynamics so that we can get x and then get h(x()) for the predicted output!
         u(i) in U for all i in [K-N_MHE,k-1] --> constraint on control action
         x(i) in X for all i in [K-N_MHE,k] --> constraint on state


    advantage of MHE --> deals with exact non linear system --> EKF does linearization at operating point but here we dont do
                        it relaxes markov assumption --> current state depden only the prevous state and previous control action -->so in EKF you update only based on your most recent estimate!
                                                        and this is markov assumpotion but here you look at your most recent window of measurement (horizon!)
                        handles constraints in natural way!


                        
    example:
        mobile robot!

        Disturbed Motion model --> we assume this! --> what ever control action v,w you send (vbar, omegabar) is disturbed!
            nominal plus gaussian /white noise!! --> nominal is what your controller calculated!
            vbar(.) = v(.) + v_v    
            omegabar(.) = omega(.) + v_omega

            so this is a distrubed motion model!



            x(k+1) = x(k) + deltaT (vbar(k) cos(theta(k)))
            y(k+1) = y(k) + deltaT (vbar(k) sin(theta(k)))
            theta(k+1) = theta(k) + deltaT omegaBar(k)




            sensor ? beacon at the origin --> gives the range (distance from origna to center of robot) and the alpha (angle of the vector of orign to robot wrt x axis! )

            y(k) = h(x(k)) + vy

            r = sqrt(x^2 + y^2)+vr
            alpha = arctan(y/x) + v_alpha


            vr and vlpha are additive gausian noise with standard deviaton sigma r and sigam alpha!

            our sensor is noisy!
            

            robot state vector [x y theta]^T



            would the system be observable under this sensor!??? observability matrix (A and C in linear system is full rank or not but here its nonlinear!)

            the diff drive system is only observable when the linear speed is non zero! --> imagien the diff drive turn in place then the range and bearing would be fixed! --> not observable! because you can seimtate x and y but theta of the robot!
            for theta you need to interpolate between two positions to get the theta!



            weight matrices for J :

             if noise is gaussian!

                W = inv[sigma_v  0
                         0      sigma_w] -->w is omega!
                    
                    if sigma_v is large means error is large but inverse of it is low so its not effect my cost function but if you reduce v its gonna dominate the cost function

                V = inv [sigma_r   0
                            0    sigma_alpha]
            

'''


'''
lets first simulate distrubed model! covariance (diagonal matrix of std_dev of linear and angular speed )^2! --> std dev of 5 mimimeter per second in linear speed and 2 degrees per second for angular speed!
                now apply that noise when you propagate the dynamics (in the while loop of the MPC!)


                know that when you plot you can only see nominal control actions you calcualted!!!! but you DONT KNOW THE DISTRUBED CONTROL ACTIONS! --> you get to estimate them latter though!

synthesize the measurement!
        measuemrent covariance matrix -->0.1 error in range measuremnet (std_dev) and 2 degreee (deg to rad) std_dev in bearing measurement

        we go for each x,y and use the measuremnt equation we find r and alpha  and y_measurement is [r, alpha];

        if we know r and alpha we can figure out x and y ==> x =r cos(alpha) and y = r sin(alpha) --> good for visualization with noise and the real x and y you used at first!

        froom measuremnt model we cant find theta directly and thats where state estimator comes into picture

        so there is a real x and y and there is a noisy measurement and state sesitmator need to find a better esimate and also give us esimtate for angualr orientaion (theta)




        IMPORTANT --> we discretized the diff drive model because its differential but the measurement model is already algebraic so no need for euler discritization!


        every time time you have to give MHE window of measurement (output r and alpha and also control action v and omega!) so there is p matrix with 2 rows and (N_MHE+1)+N_MHE columns


        also rememebr we have to initilize the values everytime for the optimization variables! U0 and X0 --> at first you init to zero but you can have a better esimtate for U0 by the measure Us and you have r and alpha so you can init the X0 aswell 


        first estimate would be after N steps because we look backward so we have to wait for it! we use the latest estimate one in the feedback controller!
        at the end of the diff drive simulation since the linear speed goes to zero you see angluar (theta) estimation is bad because it gets unobservable when speed is zero!

'''



# # MHE for Diff drive for estimating the states x ,y and theta based on noisy measurement! 

# import casadi as ca
# import numpy as np
# import matplotlib.pyplot as plt

# Ts = 0.1
# N = 30


# def robot_dynamics():
#     x = ca.MX.sym('x')
#     y = ca.MX.sym('y')
#     theta = ca.MX.sym('theta')

#     state = ca.vertcat(x,y,theta)

#     v = ca.MX.sym('v')
#     w = ca.MX.sym('w')

#     control = ca.vertcat(v,w)

#     next_state = state + Ts * ca.vertcat(
#         v*ca.cos(theta),
#         v*ca.sin(theta),
#         w
#     )

#     f = ca.Function('f', [state,control],[next_state])

#     return f


# f = robot_dynamics()


# def measurement_model():
#     x = ca.MX.sym('x')
#     y = ca.MX.sym('y')
#     theta = ca.MX.sym('theta')

#     state = ca.vertcat(x,y,theta)

#     r = ca.sqrt(x**2+y**2)
#     alpha = ca.atan2(y,x)

#     measurement = ca.vertcat(r,alpha)

#     h = ca.Function('h', [state], [measurement])
#     return h

# h = measurement_model()

# # x = h([1,2,0])
# # print(x)   ---> mind that theta (heading of the robot!) disapears because the beacon cannot measure it! --> alpha is the angle of the vector from origin to com of the robot wrt x axis!

# opti = ca.Opti()

# '''
#     v0 v1 ... vN-1
#     w0 w1 ... wN-1
# '''
# U = opti.variable(2,N) # I guess this can also be parameter instead of variable becaue we already know the control action we used
# X = opti.variable(3,N+1)

# '''
#     [r0 alpha0]
#     [r1 alpha1]
#     ...
#     [rN alphaN]
# '''
# Y = opti.parameter(2,N+1) # we need to feed new data every iteration!
# U_measured = opti.parameter(2,N)


# x_initial = opti.parameter(3)
# # opti.subject_to( X[:,0]==x_initial)




# sigma_v = 0.005
# sigma_w = np.deg2rad(2)

# sigma_r = 0.1
# sigma_alpha = np.deg2rad(2)

# R_meas = np.diag([
#     sigma_r**2,
#     sigma_alpha**2
# ])

# Q_proc = np.diag([
#     sigma_v**2,
#     sigma_w**2
# ])

# V = np.linalg.inv(R_meas)
# W = np.linalg.inv(Q_proc)

# cost = 0

# arrival_error=X[:,0]-x_initial

# cost += ca.mtimes([
#     arrival_error.T,
#     np.diag([10,10,100]),
#     arrival_error
# ])


# error_y0 = Y[:,0]-h(X[:,0])

# cost += ca.mtimes([
#     error_y0.T,
#     V,
#     error_y0
# ])

# for k in range(N):
#     X_k = X[:,k]
#     X_next = X[:,k+1]
#     U_k = U[:,k]

#     opti.subject_to(X_next == f(X_k,U_k)) #"The estimated trajectory must obey physics."

#     y_pred = h(X_next)
#     error_y = Y[:,k+1] - y_pred

#     cost += ca.mtimes([error_y.T,V,error_y]) #If predicted sensor reading differs from real sensor reading, increase cost.

#     error_u = U_measured[:,k] - U_k

#     cost += ca.mtimes([error_u.T,W,error_u]) #"Actual input can differ from commanded input, but not too much."



# for k in range(N):

#     opti.subject_to(U[0,k] <= 0.8)
#     opti.subject_to(U[0,k] >= 0.2)

#     opti.subject_to(U[1,k] <= 0.3)
#     opti.subject_to(U[1,k] >= -0.1)



# opti.minimize(cost)
# opti.solver('ipopt')

# ##### SIMULATION SIDE!

# def propagate_real(x,u):

#     noisy_u =np.array([
#         u[0]+np.random.normal(0,sigma_v),
#         u[1]+np.random.normal(0,sigma_w),
#     ])

#     x_next = f(x,noisy_u)
#     return np.array(x_next).reshape(-1)



# def get_measurement(x):
#     r = np.sqrt(x[0]**2+x[1]**2)
#     alpha = np.arctan2(x[1],x[0])
#     return np.array([
#         r+np.random.normal(0,sigma_r),
#         alpha+np.random.normal(0,sigma_alpha)
#     ])



# x_true = np.array([1,1,0]) # real robot
# x_est = np.array([1,1,0])
# x_arrival = np.array([1,1,0])

# u_history = []
# y_history =[]
# u_command = np.array([0.5,0.1])
# '''
#     because:

#     u_history contains:
#     [
#     [v0,w0],
#     [v1,w1],
#     ...
#     ]

#     Convert to:

#     U_data =
#     [
#     [v0 v1 ...]
#     [w0 w1 ...]
#     ]
# '''


# true_history = []
# est_history = []
# meas_history = []

# for t in range(400):

#     u_command=np.array([
#         0.5,
#         0.3*np.sin(0.2*t)
#     ])


#     x_true = propagate_real(x_true, u_command)
#     y= get_measurement(x_true)

#     true_history.append(x_true.copy())
#     meas_history.append([
#         y[0] * np.cos(y[1]),
#         y[0] * np.sin(y[1])
#     ])


#     u_history.append(u_command)
#     y_history.append(y)
#     if len(y_history) >= N+1:
#         Y_data = np.array(y_history[-(N+1):]).T
#         U_data = np.array(u_history[-N:]).T
#         opti.set_value(Y,Y_data)
#         opti.set_value(U_measured,U_data)
#         opti.set_value(x_initial,x_arrival)

#         opti.set_initial(X,np.tile(x_est.reshape(-1,1),(1,N+1)))
#         opti.set_initial(U,U_data)        


#         sol = opti.solve()
#         X_solution = sol.value(X)

#         x_est = X_solution[:,-1]

#         x_arrival = X_solution[:,1]
#         est_history.append(x_est.copy())

#         print(
#             "true:",
#             x_true,
#             "estimate:",
#             x_est
#         )

# true_history = np.array(true_history)
# est_history = np.array(est_history)
# meas_history = np.array(meas_history)


# plt.figure()

# plt.plot(
#     true_history[:,0],
#     true_history[:,1],
#     label="True"
# )

# plt.scatter(
#     meas_history[:,0],
#     meas_history[:,1],
#     s=10,
#     alpha=0.5,
#     label="Noisy measurement"
# )

# plt.plot(
#     est_history[:,0],
#     est_history[:,1],
#     label="MHE estimate"
# )

# plt.xlabel("x [m]")
# plt.ylabel("y [m]")
# plt.axis("equal")
# plt.legend()
# plt.grid()

# plt.show()



# time_true = np.arange(len(true_history)) * Ts
# time_est = np.arange(N, len(true_history)) * Ts

# fig, axes = plt.subplots(3, 1, figsize=(10, 8))

# axes[0].plot(
#     time_true,
#     true_history[:,0],
#     label="True x"
# )

# axes[0].plot(
#     time_est,
#     est_history[:,0],
#     label="MHE x"
# )

# axes[0].set_ylabel("x [m]")
# axes[0].legend()
# axes[0].grid()


# axes[1].plot(
#     time_true,
#     true_history[:,1],
#     label="True y"
# )

# axes[1].plot(
#     time_est,
#     est_history[:,1],
#     label="MHE y"
# )

# axes[1].set_ylabel("y [m]")
# axes[1].legend()
# axes[1].grid()


# axes[2].plot(
#     time_true,
#     true_history[:,2],
#     label="True θ"
# )

# axes[2].plot(
#     time_est,
#     est_history[:,2],
#     label="MHE θ"
# )

# axes[2].set_ylabel("θ [rad]")
# axes[2].set_xlabel("time [s]")
# axes[2].legend()
# axes[2].grid()

# plt.show()



import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# SETTINGS
# ============================================================

Ts = 0.1          # sampling time [s]
N = 30            # MHE horizon


# ============================================================
# ROBOT DYNAMICS
#
# x(k+1) = f(x(k), u(k))
#
# state  = [x, y, theta]
# input  = [v, omega]
# ============================================================

def robot_dynamics():

    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')

    state = ca.vertcat(x, y, theta)

    v = ca.MX.sym('v')
    omega = ca.MX.sym('omega')

    control = ca.vertcat(v, omega)

    next_state = state + Ts * ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega
    )

    f = ca.Function(
        'f',
        [state, control],
        [next_state]
    )

    return f


f = robot_dynamics()


# ============================================================
# MEASUREMENT MODEL
#
# Beacon is at the origin.
#
# y = h(x)
#
# measurement = [range, bearing]
#
# theta does NOT appear in the measurement.
# ============================================================

def measurement_model():

    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')

    state = ca.vertcat(x, y, theta)

    r = ca.sqrt(x**2 + y**2)

    alpha = ca.atan2(y, x)

    measurement = ca.vertcat(r, alpha)

    h = ca.Function(
        'h',
        [state],
        [measurement]
    )

    return h


h = measurement_model()


# ============================================================
# NOISE PARAMETERS
# ============================================================

# Process/input noise
#
# actual v     = commanded v     + noise
# actual omega = commanded omega + noise

sigma_v = 0.005                  # m/s
sigma_w = np.deg2rad(2)          # rad/s


# Measurement noise

sigma_r = 0.1                    # m
sigma_alpha = np.deg2rad(2)      # rad


# ============================================================
# COVARIANCE / WEIGHT MATRICES
# ============================================================

R_meas = np.diag([
    sigma_r**2,
    sigma_alpha**2
])

Q_proc = np.diag([
    sigma_v**2,
    sigma_w**2
])


# MHE uses inverse covariance as weights

V = np.linalg.inv(R_meas)
W = np.linalg.inv(Q_proc)


# ============================================================
# BUILD THE MHE OPTIMIZATION PROBLEM
# ============================================================

opti = ca.Opti()


# ------------------------------------------------------------
# Decision variables
# ------------------------------------------------------------

# Estimated actual inputs:
#
#       v0  v1  ... v(N-1)
# U = [
#       w0  w1  ... w(N-1)
#     ]

U = opti.variable(2, N)


# Estimated states:
#
#       x0  x1  ... xN
# X =   y0  y1  ... yN
#       t0  t1  ... tN
#
# There are N+1 states because N inputs propagate
# the state N times.

X = opti.variable(3, N + 1)


# ------------------------------------------------------------
# Parameters
# ------------------------------------------------------------

# Measurements:
#
#       [r0     r1     ... rN]
# Y =   [alpha0 alpha1 ... alphaN]

Y = opti.parameter(2, N + 1)


# Commanded/measured inputs:
#
#       [v0  v1  ... v(N-1)]
# U_measured =
#       [w0  w1  ... w(N-1)]

U_measured = opti.parameter(2, N)


# Arrival-state estimate/prior

x_initial = opti.parameter(3)


# ============================================================
# COST FUNCTION
# ============================================================

cost = 0


# ------------------------------------------------------------
# 1. ARRIVAL COST
#
# We have an estimate of the state at the beginning of
# the horizon from the previous MHE solution.
#
# X[:,0] should stay reasonably close to that estimate,
# but it is NOT hard-constrained to it.
# ------------------------------------------------------------

arrival_error = X[:, 0] - x_initial

arrival_weight = np.diag([
    10,
    10,
    10
])

cost += ca.mtimes([
    arrival_error.T,
    arrival_weight,
    arrival_error
])


# ------------------------------------------------------------
# 2. MEASUREMENT COST
#
# The first state X[:,0] must explain measurement Y[:,0].
# ------------------------------------------------------------

y_pred = h(X[:, 0])


# Range error

range_error = Y[0, 0] - y_pred[0]


# Bearing error
#
# IMPORTANT:
# Angles wrap around at +/- pi.
#
# atan2(sin(error), cos(error))
# gives the shortest angular difference.

bearing_error = ca.atan2(
    ca.sin(Y[1, 0] - y_pred[1]),
    ca.cos(Y[1, 0] - y_pred[1])
)


measurement_error = ca.vertcat(
    range_error,
    bearing_error
)


cost += ca.mtimes([
    measurement_error.T,
    V,
    measurement_error
])


# ============================================================
# 3. DYNAMICS + MEASUREMENT + INPUT COST
# ============================================================

for k in range(N):

    # Current state
    X_k = X[:, k]

    # Next state
    X_next = X[:, k + 1]

    # Estimated actual input
    U_k = U[:, k]


    # --------------------------------------------------------
    # Dynamics constraint
    #
    # X(k+1) = f(X(k), U(k))
    #
    # This forces the estimated trajectory to obey
    # the robot's motion model.
    # --------------------------------------------------------

    opti.subject_to(
        X_next == f(X_k, U_k)
    )


    # --------------------------------------------------------
    # Measurement prediction at X(k+1)
    # --------------------------------------------------------

    y_pred = h(X_next)


    # Range error

    range_error = Y[0, k + 1] - y_pred[0]


    # Bearing error
    #
    # Wrapped angular residual

    bearing_error = ca.atan2(
        ca.sin(Y[1, k + 1] - y_pred[1]),
        ca.cos(Y[1, k + 1] - y_pred[1])
    )


    measurement_error = ca.vertcat(
        range_error,
        bearing_error
    )


    # --------------------------------------------------------
    # Measurement cost
    # --------------------------------------------------------

    cost += ca.mtimes([
        measurement_error.T,
        V,
        measurement_error
    ])


    # --------------------------------------------------------
    # Input disturbance cost
    #
    # U_measured = commanded input
    # U          = estimated actual input
    #
    # We expect them to be close, but not identical.
    # --------------------------------------------------------

    input_error = U_measured[:, k] - U_k


    cost += ca.mtimes([
        input_error.T,
        W,
        input_error
    ])


# ============================================================
# INPUT CONSTRAINTS
# ============================================================

for k in range(N):

    # Linear velocity

    opti.subject_to(
        U[0, k] >= 0.0
    )

    opti.subject_to(
        U[0, k] <= 0.8
    )


    # Angular velocity
    #
    # Must cover the actual simulation range:
    # omega = 0.3*sin(...)
    #
    # therefore omega is approximately [-0.3, +0.3]

    opti.subject_to(
        U[1, k] >= -0.4
    )

    opti.subject_to(
        U[1, k] <= 0.4
    )


# ============================================================
# MINIMIZE
# ============================================================

opti.minimize(cost)


# ============================================================
# SOLVER
# ============================================================

opti.solver(
    'ipopt',
    {
        'ipopt.print_level': 0,
        'print_time': 0
    }
)


# ============================================================
# REAL ROBOT SIMULATION
# ============================================================

def propagate_real(x, u):

    # The robot does NOT receive exactly the commanded input.
    #
    # Actual input =
    # commanded input + noise

    noisy_u = np.array([
        u[0] + np.random.normal(0, sigma_v),
        u[1] + np.random.normal(0, sigma_w)
    ])


    # Propagate using the noisy/actual input

    x_next = f(
        x,
        noisy_u
    )


    return np.array(x_next).reshape(-1)


# ============================================================
# SENSOR SIMULATION
# ============================================================

def get_measurement(x):

    # True range

    r = np.sqrt(
        x[0]**2 +
        x[1]**2
    )


    # True bearing

    alpha = np.arctan2(
        x[1],
        x[0]
    )


    # Add sensor noise

    r_measured = (
        r +
        np.random.normal(0, sigma_r)
    )


    alpha_measured = (
        alpha +
        np.random.normal(0, sigma_alpha)
    )


    return np.array([
        r_measured,
        alpha_measured
    ])


# ============================================================
# INITIAL CONDITIONS
# ============================================================

# True robot state

x_true = np.array([
    1.0,
    1.0,
    0.0
])


# Initial MHE estimate

x_est = np.array([
    1.0,
    1.0,
    0.0
])


# Initial arrival estimate

x_arrival = np.array([
    1.0,
    1.0,
    0.0
])


# ============================================================
# HISTORY
# ============================================================

u_history = []

y_history = []

true_history = []

est_history = []

meas_history = []


# Previous solution for warm starting

X_solution = None
U_solution = None


# ============================================================
# SIMULATION LOOP
# ============================================================

for t in range(400):


    # --------------------------------------------------------
    # Command generated by controller
    # --------------------------------------------------------

    u_command = np.array([
        0.5,
        0.3 
    ])


    # --------------------------------------------------------
    # REAL ROBOT
    #
    # Actual input is noisy.
    # --------------------------------------------------------

    x_true = propagate_real(
        x_true,
        u_command
    )


    # --------------------------------------------------------
    # SENSOR
    #
    # Sensor measures range and bearing.
    # --------------------------------------------------------

    y = get_measurement(
        x_true
    )


    # --------------------------------------------------------
    # Save true state
    # --------------------------------------------------------

    true_history.append(
        x_true.copy()
    )


    # --------------------------------------------------------
    # Convert noisy range/bearing to x/y
    #
    # This is ONLY for visualization.
    #
    # MHE itself uses [r, alpha].
    # --------------------------------------------------------

    meas_history.append([
        y[0] * np.cos(y[1]),
        y[0] * np.sin(y[1])
    ])


    # --------------------------------------------------------
    # Save input and measurement
    # --------------------------------------------------------

    u_history.append(
        u_command.copy()
    )

    y_history.append(
        y.copy()
    )


    # ========================================================
    # WAIT UNTIL WE HAVE A FULL HORIZON
    # ========================================================

    if len(y_history) >= N + 1:


        # ----------------------------------------------------
        # Extract latest N+1 measurements
        # ----------------------------------------------------

        Y_data = np.array(
            y_history[-(N + 1):]
        ).T


        # ----------------------------------------------------
        # Extract latest N commanded inputs
        # ----------------------------------------------------

        U_data = np.array(
            u_history[-N:]
        ).T


        # ----------------------------------------------------
        # Give measurements to MHE
        # ----------------------------------------------------

        opti.set_value(
            Y,
            Y_data
        )


        opti.set_value(
            U_measured,
            U_data
        )


        # ----------------------------------------------------
        # Give arrival-state prior
        # ----------------------------------------------------

        # opti.set_value(
        #     x_initial,
        #     x_arrival
        # )

        opti.set_value(x_initial, x_est)


        # ====================================================
        # INITIAL GUESS
        # ====================================================

        if X_solution is None:

            # First MHE solve.
            #
            # We don't have a previous trajectory yet.

            opti.set_initial(
                X,
                np.tile(
                    x_est.reshape(-1, 1),
                    (1, N + 1)
                )
            )


            opti.set_initial(
                U,
                U_data
            )

        else:
            # ------------------------------------------------
            # Shift previous state trajectory
            #
            # Previous:
            #
            # X0 X1 X2 ... XN
            #
            # New:
            #
            # X1 X2 X3 ... XN XN
            #
            # This is the standard receding-horizon warm start.
            # ------------------------------------------------

            X_guess = np.hstack([
                X_solution[:, 1:],
                X_solution[:, -1:]
            ])


            # ------------------------------------------------
            # Shift previous input trajectory
            #
            # Previous:
            #
            # U0 U1 U2 ... U(N-1)
            #
            # New:
            #
            # U1 U2 ... U(N-1) U(N-1)
            # ------------------------------------------------

            U_guess = np.hstack([
                U_solution[:, 1:],
                U_solution[:, -1:]
            ])


            opti.set_initial(
                X,
                X_guess
            )


            opti.set_initial(
                U,
                U_guess
            )


        # ====================================================
        # SOLVE MHE
        # ====================================================

        sol = opti.solve()


        # ====================================================
        # GET SOLUTION
        # ====================================================

        X_solution = sol.value(X)

        U_solution = sol.value(U)


        # ----------------------------------------------------
        # Current state estimate
        #
        # The right-most state is our newest estimate.
        # ----------------------------------------------------

        x_est = X_solution[:, -1]


        # ----------------------------------------------------
        # Arrival estimate for NEXT horizon
        #
        # IMPORTANT:
        #
        # The next horizon starts one sample later.
        #
        # Therefore use X1, not X0.
        # ----------------------------------------------------

        x_arrival = X_solution[:, 1]


        # ----------------------------------------------------
        # Save current estimate
        # ----------------------------------------------------

        est_history.append(
            x_est.copy()
        )


        # ----------------------------------------------------
        # Print
        # ----------------------------------------------------

        print(
            "true:     ",
            x_true,
            "estimate: ",
            x_est
        )


# ============================================================
# CONVERT HISTORY TO NUMPY ARRAYS
# ============================================================

true_history = np.array(
    true_history
)

est_history = np.array(
    est_history
)

meas_history = np.array(
    meas_history
)


# ============================================================
# PLOT XY TRAJECTORY
# ============================================================

plt.figure()

plt.plot(
    true_history[:, 0],
    true_history[:, 1],
    label="True"
)

plt.scatter(
    meas_history[:, 0],
    meas_history[:, 1],
    s=10,
    alpha=0.5,
    label="Noisy measurement"
)

plt.plot(
    est_history[:, 0],
    est_history[:, 1],
    label="MHE estimate"
)

plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.axis("equal")

plt.legend()

plt.grid()

plt.show()


# ============================================================
# TIME AXIS
# ============================================================

time_true = (
    np.arange(len(true_history))
    * Ts
)

time_est = (
    np.arange(N, len(true_history))
    * Ts
)


# ============================================================
# STATE ESTIMATION PLOTS
# ============================================================

fig, axes = plt.subplots(
    3,
    1,
    figsize=(10, 8)
)


# ------------------------------------------------------------
# X
# ------------------------------------------------------------

axes[0].plot(
    time_true,
    true_history[:, 0],
    label="True x"
)

axes[0].plot(
    time_est,
    est_history[:, 0],
    label="MHE x"
)

axes[0].set_ylabel(
    "x [m]"
)

axes[0].legend()

axes[0].grid()


# ------------------------------------------------------------
# Y
# ------------------------------------------------------------

axes[1].plot(
    time_true,
    true_history[:, 1],
    label="True y"
)

axes[1].plot(
    time_est,
    est_history[:, 1],
    label="MHE y"
)

axes[1].set_ylabel(
    "y [m]"
)

axes[1].legend()

axes[1].grid()


# ------------------------------------------------------------
# THETA
# ------------------------------------------------------------

axes[2].plot(
    time_true,
    true_history[:, 2],
    label="True theta"
)

axes[2].plot(
    time_est,
    est_history[:, 2],
    label="MHE theta"
)

axes[2].set_ylabel(
    "theta [rad]"
)

axes[2].set_xlabel(
    "time [s]"
)

axes[2].legend()

axes[2].grid()


plt.tight_layout()

plt.show()



'''
============================================================
MOVING HORIZON ESTIMATION (MHE) - GENERAL TEMPLATE
============================================================

Given:

    System dynamics:
        x(i+1) = f(x(i), u(i)) + w(i)

    Measurement model:
        y(i) = h(x(i)) + v(i)

    At current time k, use a horizon of N steps:

        i = k-N, ..., k

    Available data:
        N+1 measurements: y(k-N), ..., y(k)
        N inputs:         u(k-N), ..., u(k-1)


------------------------------------------------------------
1. MHE DECISION VARIABLES
------------------------------------------------------------

Estimate the states over the whole horizon:

    X = {x(k-N), ..., x(k)}

If input/process disturbances are estimated:

    U = {u(k-N), ..., u(k-1)}

or, if the measured input is known but noisy:

    U = {estimated actual inputs}


------------------------------------------------------------
2. MHE OBJECTIVE
------------------------------------------------------------

Find the state/input trajectory that best explains:

    • previous state estimate
    • measured outputs
    • measured inputs

Generic form:

    minimize

        arrival cost
        +
        Σ measurement error cost
        +
        Σ input/process disturbance cost


For example:

    min
        ||x(k-N) - x_arrival||²_P
        +
        Σ ||y(i) - h(x(i))||²_R⁻¹
        +
        Σ ||u_measured(i) - u(i)||²_Q⁻¹


where:

    P        = arrival-state weight
    R        = measurement covariance
    Q        = input/process-noise covariance

and:

    ||e||²_W = eᵀ W e


------------------------------------------------------------
3. EQUALITY CONSTRAINTS: SYSTEM DYNAMICS
------------------------------------------------------------

THIS IS THE PREDICTION STEP.

For every transition:

    x(i+1) = f(x(i), u(i))

for:

    i = k-N, ..., k-1


This connects all states together:

    x(k-N) --u(k-N)--> x(k-N+1)
                  ...
    x(k-1)  --u(k-1)--> x(k)


The dynamics constraint means:

    estimated state + estimated input
                    ↓
               system model
                    ↓
             next estimated state


Then the predicted measurement is:

    y_hat(i) = h(x(i))


Therefore:

    x(i)
      ↓
    h(x(i))
      ↓
    y_hat(i)

and the measurement residual is:

    e_y(i) = y(i) - h(x(i))


------------------------------------------------------------
4. STATE CONSTRAINTS
------------------------------------------------------------

If the physical system has state limits:

    x(i) ∈ X

for:

    i = k-N, ..., k


Examples:

    x_min ≤ x(i) ≤ x_max

    position inside map

    velocity limits

    angle limits

    actuator-related state limits


------------------------------------------------------------
5. INPUT CONSTRAINTS
------------------------------------------------------------

If inputs have limits:

    u(i) ∈ U

for:

    i = k-N, ..., k-1


Examples:

    v_min ≤ v(i) ≤ v_max

    omega_min ≤ omega(i) ≤ omega_max


------------------------------------------------------------
6. COMPLETE GENERIC MHE PROBLEM
------------------------------------------------------------

                         k
                         ↓
    -------------------------------------------------
    Estimate:
        x(k-N), ..., x(k)
        u(k-N), ..., u(k-1)
    -------------------------------------------------

    minimize:

        ||x(k-N) - x_arrival||²_P

        + Σ ||y(i) - h(x(i))||²_R⁻¹

        + Σ ||u_measured(i) - u(i)||²_Q⁻¹

    subject to:

        x(i+1) = f(x(i),u(i))

        x(i) ∈ X

        u(i) ∈ U

    for the appropriate i over the horizon.


------------------------------------------------------------
7. WHAT HAPPENS EVERY TIME MHE RUNS?
------------------------------------------------------------

At time k:

    1. Collect latest N+1 measurements:

           y(k-N), ..., y(k)

    2. Collect latest N inputs:

           u(k-N), ..., u(k-1)

    3. Put them into the parameter/data matrix P.

    4. Solve the optimization problem.

    5. Take the newest state:

           x_hat(k) = X[:,N]

       as the current state estimate.

    6. Move the horizon forward by one sample.

    7. Repeat.


------------------------------------------------------------
8. DATA MATRIX P
------------------------------------------------------------

P contains the measurements and known inputs supplied
to the MHE at each optimization.

For example:

              k-N   ...   k-1    k
             -------------------------
    y        | y0    ...   yN-1   yN
    u        | u0    ...   uN-1    -

The exact arrangement of P depends on the implementation.

Important:

    P = known data / parameters

    X,U = optimization variables


------------------------------------------------------------
9. THE KEY IDEA TO MEMORIZE
------------------------------------------------------------

MHE =

    "Find the state trajectory over the last N steps
     that best fits the measurements and inputs while
     obeying the system dynamics and constraints."

The three most important pieces are:

    OBJECTIVE:
        fit measurements + respect prior + explain input noise

    EQUALITY CONSTRAINT:
        x(i+1) = f(x(i),u(i))

    INEQUALITY CONSTRAINTS:
        x(i) ∈ X
        u(i) ∈ U


============================================================
SHORT VERSION TO MEMORIZE
============================================================

Given:

    x(i+1) = f(x(i),u(i)) + w(i)
    y(i)   = h(x(i)) + v(i)

At time k, use:

    y(k-N), ..., y(k)
    u(k-N), ..., u(k-1)

Solve:

    min_{X,U}

        ||x(k-N)-x_arrival||²_P
        +
        Σ ||y(i)-h(x(i))||²_R⁻¹
        +
        Σ ||u_measured(i)-u(i)||²_Q⁻¹

    s.t.

        x(i+1) = f(x(i),u(i))

        x(i) ∈ X

        u(i) ∈ U

Take:

        x_hat(k) = x(k)

Then move the horizon forward and repeat.


ONE-LINE MEMORY:

    MHE = optimization over a moving window
          + measurement fitting
          + dynamics equality constraints
          + state/input constraints.



'''

# CASADI TEMPLATE!

'''
# ============================================================
# MHE TEMPLATE: FORMULATION → CASADI
# ============================================================

# 1. DEFINE THE SYSTEM
#
# State dynamics:
#       x(k+1) = f(x(k), u(k))
#
# Measurement model:
#       y(k)   = h(x(k))
#
# Example:
#       x_next = f(x, u)
#       y_pred = h(x)


# ============================================================
# 2. DEFINE THE MHE WINDOW
# ============================================================

# At current time k, use:

# states:
#       X = [x(k-N), ..., x(k)]       → N+1 states
#
# inputs:
#       U = [u(k-N), ..., u(k-1)]      → N inputs
#
# measurements:
#       Y = [y(k-N), ..., y(k)]        → N+1 measurements


# ============================================================
# 3. CREATE CASADI OPTIMIZATION PROBLEM
# ============================================================

opti = ca.Opti()


# ============================================================
# 4. CREATE DECISION VARIABLES
# ============================================================

# States are ALWAYS decision variables in MHE

X = opti.variable(nx, N+1)

# If actual inputs/process disturbances are estimated:
# inputs are also decision variables

U = opti.variable(nu, N)


# ============================================================
# 5. CREATE PARAMETERS FOR MEASURED DATA
# ============================================================

# Measurements are known data → parameters

Y = opti.parameter(ny, N+1)

# Measured/commanded inputs are known data → parameters

U_measured = opti.parameter(nu, N)


# ============================================================
# 6. DEFINE THE OBJECTIVE
# ============================================================

cost = 0


# ------------------------------------------------------------
# Measurement error
# ------------------------------------------------------------

for i in range(N+1):

    # Predict measurement from estimated state
    y_pred = h(X[:, i])

    # Measurement residual
    e_y = Y[:, i] - y_pred

    # Add weighted measurement error
    cost += e_y.T @ R_inv @ e_y


# ------------------------------------------------------------
# Input / process error
# ------------------------------------------------------------

for i in range(N):

    # Difference between estimated actual input
    # and measured/commanded input

    e_u = U_measured[:, i] - U[:, i]

    # Add weighted input/process error
    cost += e_u.T @ Q_inv @ e_u


# ============================================================
# 7. ADD THE DYNAMICS EQUALITY CONSTRAINTS
# ============================================================

for i in range(N):

    # Current state
    x_i = X[:, i]

    # Current estimated input
    u_i = U[:, i]

    # Predict next state using system model
    x_next = f(x_i, u_i)

    # FORCE the next optimization state to follow dynamics
    opti.subject_to(
        X[:, i+1] == x_next
    )

# THIS IS THE IMPORTANT MHE EQUALITY CONSTRAINT:
#
#       x(i+1) = f(x(i), u(i))
#
# It connects all states inside the moving horizon.


# ============================================================
# 8. ADD STATE CONSTRAINTS
# ============================================================

for i in range(N+1):

    opti.subject_to(
        X[:, i] >= x_lower
    )

    opti.subject_to(
        X[:, i] <= x_upper
    )

# Conceptually:
#
#       x(i) ∈ X
#
# for i = k-N ... k


# ============================================================
# 9. ADD INPUT CONSTRAINTS
# ============================================================

for i in range(N):

    opti.subject_to(
        U[:, i] >= u_lower
    )

    opti.subject_to(
        U[:, i] <= u_upper
    )

# Conceptually:
#
#       u(i) ∈ U
#
# for i = k-N ... k-1


# ============================================================
# 10. GIVE THE OBJECTIVE TO CASADI
# ============================================================

opti.minimize(cost)


# ============================================================
# 11. SOLVE
# ============================================================

sol = opti.solve()


# ============================================================
# 12. EXTRACT THE CURRENT STATE ESTIMATE
# ============================================================

X_solution = sol.value(X)

# The newest state is the last column

x_hat_k = X_solution[:, -1]


# ============================================================
# 13. MOVE THE WINDOW
# ============================================================

# At the next time step:

# discard:
#       y(k-N)
#       u(k-N)
#
# add:
#       y(k+1)
#       u(k)

# New window:
#
#       y(k-N+1) ... y(k+1)
#       u(k-N+1) ... u(k)


# ============================================================
# THE WHOLE MHE IN ONE PICTURE
# ============================================================

#
#             MEASUREMENTS
#        y0    y1    y2        yN
#         ↓     ↓     ↓         ↓
#         h     h     h         h
#         ↓     ↓     ↓         ↓
#       x0 →  x1 →  x2 → ... → xN
#        ↑     ↑     ↑          ↑
#        u0    u1    u2        uN-1
#
#             ↑
#             |
#       dynamics constraints
#
#       x(i+1) = f(x(i),u(i))
#
#
# OPTIMIZATION:
#
#   minimize
#
#       measurement errors
#       +
#       input/process errors
#
#   subject to
#
#       dynamics equality
#       state constraints
#       input constraints


# ============================================================
# MEMORY FORMULA
# ============================================================

# MHE =

#   DECISION VARIABLES:
#       X, U
#
#   PARAMETERS:
#       Y, U_measured
#
#   COST:
#       measurement error
#       +
#       process/input error
#
#   CONSTRAINTS:
#       X_next = f(X_current, U_current)
#       state bounds
#       input bounds
#
#   SOLUTION:
#       newest state X[:, -1]
#
#   THEN:
#       shift window → add newest data → solve again



'''