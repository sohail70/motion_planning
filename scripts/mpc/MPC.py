'''
    Nonlinear programming problem! (NLP):

        min Q(w)  Objective function s.t g_1(w) <= 0 (ineuqliaty constrinat) and g_2(w) = 0 (equality constraint)
         w

        phi(.) and g1(.) and g2(.) are usually assume to be differentiable

        special cases of NLP:
        Linear programming (LP) the above 3 functions are affine (can be expressed as linear combinations of the elements of w)
        Quadratic programming (g1 and g2 are affine but phi is linear quadratic function)

        so learning NLP is enough and it covers LP and QP


        maximation is equal to negative minimzation


        max Q(w) = - min Q(w)     same constriants



        we look for local minimzation problem --> global min is normally challenging and time consuming and here in this workshop local solution is enough!

        solution:
        w* = argmin Q(w)
                w
        
        Q(w*) := min Q(w)
                  w

        Q is phi
        Ex: Q(w) = W^2 -6w +13   unconstraint (quadratic function)

            min W^2 -6w+ 13
             w        

        dQ/dw = 2w-6 = 0 --> w = 3 --> Q(w*) = 4 --> IF WE SHIFT (ADD A CONSTATN TO Q) the w* would be the same 3 but Q(w*) changes! so argmin is the same!!!


        NOW USE CasADi: to solve NLP's / QP's and Root finding problem and initial-vaue problems in ODE/DAE 
        today we solve for NLP
'''

# import casadi as ca

# opti = ca.Opti()
# w = opti.variable()
# J = w**2 - 6*w + 13
# opti.minimize(J)
# opti.solver("ipopt") #interior point optimizer
# sol = opti.solve();
# print("Optimal w =", sol.value(w))
# print("Minimum J =", sol.value(J))



'''
    min e^(0.2w) sin(w)
     w

    s.t. w>=0 and w<=4pi


    set_initial would be important because this problem has 3 minimum 2 of which are local and another is global! so if you choose initial w of 1.0 would get zero
'''


# import casadi as ca

# opti = ca.Opti()
# w = opti.variable()
# J = ca.exp(0.2*w)*ca.sin(w)
# opti.minimize(J)
# opti.subject_to(w>=0)
# opti.subject_to(w<=4*ca.pi)
# opti.set_initial(w,10.0) # w0=0 and w0=5 and w0=10

# opti.solver("ipopt") #interior point optimizer
# sol = opti.solve();
# print("Optimal w =", sol.value(w))
# print("Minimum J =", sol.value(J))



'''
    linear regression example
    y = ax+b 

    we have data point we want to fit a function to those data points
    least sqaure --> minimize the output of the model wrt to the data point


    J(a,b) = sum (yi - (a*xi+b))^2
              i
'''


# import casadi as ca

# opti = ca.Opti()
# x_data = [0,45,90,135,180]
# y_data = [667,661,757,871,1210]


# a = opti.variable()
# b = opti.variable()

# J = 0
# for x, y in zip(x_data,y_data):
#     y_hat = a*x+b
#     error = y-y_hat
#     J+=error**2
# opti.minimize(J)

# opti.solver("ipopt") #interior point optimizer
# sol = opti.solve();
# a_opt = sol.value(a)
# b_opt = sol.value(b)

# print("a =", a_opt)
# print("b =", b_opt)
# print("J =", sol.value(J))


############################################

# import casadi as ca
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


# # -----------------------------
# # Data
# # -----------------------------
# x_data = np.array([0, 45, 90, 135, 180])
# y_data = np.array([667, 661, 757, 871, 1210])


# # -----------------------------
# # Create symbolic variables
# # -----------------------------
# a = ca.MX.sym('a')   # slope
# b = ca.MX.sym('b')   # intercept


# # -----------------------------
# # Build cost function J(a,b)
# # -----------------------------
# J = 0

# for x, y in zip(x_data, y_data):
#     y_prediction = a*x + b
#     error = y - y_prediction
#     J += error**2


# print("Symbolic J:")
# print(J)


# # -----------------------------
# # Convert symbolic expression
# # into a callable function
# # -----------------------------
# J_func = ca.Function(
#     'J_func',
#     [a, b],
#     [J]
# )


# # Test evaluation
# print("\nExample evaluations:")
# print("J(0,0) =", J_func(0,0))
# print("J(5,600) =", J_func(5,600))


# # -----------------------------
# # Create grid for plotting ---> so if you wanna find the min from the plot it would be approximation and casadi would be more accurate
# # -----------------------------
# a_values = np.linspace(0, 8, 100)
# b_values = np.linspace(400, 900, 100)

# A, B = np.meshgrid(a_values, b_values)


# # Store cost values
# Z = np.zeros(A.shape)


# for i in range(A.shape[0]):
#     for j in range(A.shape[1]):

#         Z[i,j] = float(
#             J_func(
#                 A[i,j],
#                 B[i,j]
#             )
#         )


# # -----------------------------
# # 3D Plot of J(a,b)
# # -----------------------------
# fig = plt.figure(figsize=(10,7))

# ax = fig.add_subplot(
#     111,
#     projection='3d'
# )

# ax.plot_surface(
#     A,
#     B,
#     Z
# )

# ax.set_xlabel("Slope a")
# ax.set_ylabel("Intercept b")
# ax.set_zlabel("Cost J")

# ax.set_title(
#     "Regression Cost Function J(a,b)"
# )

# plt.show()


# # -----------------------------
# # Contour plot (very useful for optimization)
# # -----------------------------
# plt.figure(figsize=(8,6))

# contours = plt.contour(
#     A,
#     B,
#     Z,
#     levels=30
# )

# plt.clabel(contours)

# plt.xlabel("Slope a")
# plt.ylabel("Intercept b")
# plt.title("Cost Landscape Contours")

# plt.grid()
# plt.show()




######################################################################
'''
    MPC : SISO


        x(k+1) = f(x(k) , u(k))
        predict -> optimization between prediction and reference and send the first u you found
        only apply the first action because of inaccuracy so we do things all over again!

        l(x,u) = ||x_u - x_prime||_Q ^2 + ||u-u_prime||_R ^2 --->x_prime is the refence! u_prime is zero! (reference value for control)

        min J_N(x,u) = sum from k=zero to N-1 of the above
         u

        subject to x_u(k+1) = f(x_u(k) , u(k)),   --> system model
        x_u(0) = x_0 --> initial value of the state
        u(k) in U for all k in [0,N-1] --> constaint on u?
        x_u(k) in X for all k in [0,N] --> constraint on state you should repsect for all the prediction

        
        value function V_n(x) = min J_N(x0,u)
                                 u


        MPC can be applied to nonlinear MIMO systems

        issues? stability? performance? how do you measure optimality of the solution? how long does the optimziation horizon N shoudl be?

        

        diff drive non holonomic mobile robot

            x = [x y theta]^T
        kinemtic model:
            x_dot = r/2 (phi_dot_r + phi_dot_l)cos(theta)
            y_dot = r/2 (phi_dot_r + phi_dot_l)sin(theta)
            theta_dot = r/2 (phi_dot_r - phi_dot_l)l D

        assume v and w are control varaibles:
        v = r/2 (phi_dot_r + phi_dot_l)
        w = r/2D (phi_dot_r - phi_dot_l)

        x_dot  =v*cos(theta)
        y_dot  =v*sin(theta)
        theta_dot = omega
        
        for holonomic we have vx and vy and theta_dot becase robot can move both sides


        
        point stabilization vs trajectory tracking problem --> going to a point or traj following a time paramterized traj!

        formulate MPC for this problem:

        discretize! first  (euler!)

        x(k+1) = x(k) + deltaT (v(k)* cos(theta(k)))
        y(k+1) = y(k) + deltaT (v(k)*sin(theta(k)))
        theta(k+1) = theta(k) + deltaT(omega(k))


        running cost => l(x,u) = ||x_u - x_ref||_Q ^2 + ||u-u_ref||_R ^2
        min J_N (x0,u) = sum l(x_u(k),u(k))  --> sum from k=0 to N-1
         u (admissible)

        subject to x_u(k+1) = f(x_u(k) , u(k)),   --> system model
        x_u(0) = x_0 --> initial value of the state
        u(k) in U for all k in [0,N-1] --> constaint on u?
        x_u(k) in X for all k in [0,N] --> constraint on state you should repsect for all the prediction


        HOW TO TURN THE OPTIMAL CONTROL PROBLEM INTO NLP? method of : 1)shooting 2) multiple shooting etc
        min Q(w)  Objective function s.t g_1(w) <= 0 (ineuqliaty constrinat) and g_2(w) = 0 (equality constraint)


        
        ALGORITHM:

            state dimension --> nx
            control dimension --> nu

            U = optimizer variables (nu,N)

            X = prediction

            for k:
                X = dynamics(X,U[:,k])

                error = X-reference

                J += error.T Q error
                J += U.T R U

            add constraints

            minimize(J)

            loop:
                update x0
                solve
                take first u
                simulate
                warm start

'''

# # DIFF DRIVE!
# import casadi as ca
# import numpy as np
# import time

# import matplotlib.pyplot as plt
# def plot_triangle(x, y, theta, size=0.2):

#     # triangle in robot frame
#     points = np.array([
#         [size, 0],
#         [-size/2, size/2],
#         [-size/2, -size/2]
#     ])


#     # rotation matrix

#     R = np.array([
#         [np.cos(theta), -np.sin(theta)],
#         [np.sin(theta),  np.cos(theta)]
#     ])


#     # rotate and translate

#     points = points @ R.T

#     points[:,0] += x
#     points[:,1] += y


#     # close triangle

#     points = np.vstack([points, points[0]])


#     plt.plot(
#         points[:,0],
#         points[:,1],
#         'b-'
#     )


# Ts = 0.1
# def robot_dynamics():
#     x = ca.MX.sym('x')
#     y = ca.MX.sym('y')
#     theta = ca.MX.sym('theta')
#     state = ca.vertcat(x,y,theta)
#     v = ca.MX.sym('v')
#     omega = ca.MX.sym('omega')
#     control = ca.vertcat(v,omega)
#     next_state = state + Ts * ca.vertcat(
#         v*ca.cos(theta),
#         v*ca.sin(theta),
#         omega
#     )
#     f = ca.Function('f',[state,control],[next_state])
#     return f

# f = robot_dynamics()

# # f([0,0,0],[0.5,0])


# Q = np.diag([100,100,1])
# R = np.diag([1.0,0.5])
# N = 20
# opti = ca.Opti()
# '''
# [v0 v1 v2;
#  w0 w1 w2] --> 6 variables
# '''
# U = opti.variable(2,N)
# x0 = opti.parameter(3)
# opti.set_value(x0, np.array([0,0,0]))
# X = x0
# xref = np.array([1.5,1.5,-ca.pi])
# cost = 0
# # construct J(U) --> a function of 6 variables
# for k in range(N):
#     U_k = U[:,k]
#     X = f(X,U_k)
#     # robot should stay inside map
#     opti.subject_to(-2 <= X[0])
#     opti.subject_to(X[0] <= 2)
#     opti.subject_to(-2 <= X[1])
#     opti.subject_to(X[1] <= 2)
#     error = X-xref
#     cost += ca.mtimes([error.T,Q,error]) #Tracking error: e^T Q e --> its a scalar --> why use Q ? because not all errors are equal! --> position error is more important than heading error for parking for example!
#     cost += ca.mtimes([U_k.T,R,U_k]) #Control effort penalty

# # terminal cost (optinal) --> the final predicted point is important!
# error = X-xref
# Qf = np.diag([100,100,10])
# cost += ca.mtimes([error.T,Qf,error])

# # constraints v and omega
# for k in range(N):
#     opti.subject_to(-0.6 <= U[0,k])
#     opti.subject_to(U[0,k] <= 0.6)
#     opti.subject_to(-np.pi/4 <= U[1,k])
#     opti.subject_to(U[1,k] <= np.pi/4)

# # tell optimizer what to minimize
# opti.minimize(cost)
# opti.solver("ipopt")

# # initial guess

# u_guess = np.zeros((2,N)) # starting point fo ipopt

# opti.set_initial(U,u_guess)
# # MPC loop
# sim_time = 10
# t = 0


# #vis
# #vis
# trajectory = []
# x_current = np.array([0,0,0])

# plt.figure(figsize=(7,7))
# plt.axis("equal")
# plt.xlim(-2.5,2.5)
# plt.ylim(-2.5,2.5)
# plt.grid()

# # reference triangle
# plot_triangle(
#     xref[0],
#     xref[1],
#     xref[2],
#     size=0.3
# )

# plt.scatter(
#     xref[0],
#     xref[1],
#     marker='x',
#     s=100,
#     label="reference"
# )

# plt.legend()




# while t < sim_time:
#     if np.linalg.norm(x_current - xref) < 1e-2:
#         break
#     opti.set_value(x0,x_current)
#     opti.set_initial(U,u_guess)
#     start = time.time()
#     sol = opti.solve()
#     end = time.time()
#     mpc_time = end-start
#     '''
#     [v0 v1 v2
#      w0 w1 w2]
#     '''
#     u_opt = sol.value(U)
#     u_apply = u_opt[:,0] # apply only the first control

#     # propagate
#     x_current = np.array(
#         f(x_current,u_apply)
#     ).reshape(-1)


#     # live visualization
#     trajectory.append(x_current.copy())
#     trajectory_np = np.array(trajectory)
#     # clear previous drawing
#     plt.cla()
#     # redraw map
#     plt.xlim(-2.5,2.5)
#     plt.ylim(-2.5,2.5)
#     plt.axis("equal")
#     plt.grid()
#     # reference triangle
#     plot_triangle(
#         xref[0],
#         xref[1],
#         xref[2],
#         size=0.3
#     )
#     # path
#     if len(trajectory_np) > 1:
#         plt.plot(
#             trajectory_np[:,0],
#             trajectory_np[:,1],
#             'k--'
#         )
#     # current robot
#     plot_triangle(
#         x_current[0],
#         x_current[1],
#         x_current[2]
#     )
#     plt.pause(0.2)




#     # warm start --> to make MPC faster
#     # shift old solution one step
#     u_guess = np.hstack(
#         [
#             u_opt[:,1:],
#             np.zeros((2,1))
#         ]
#     )
#     t += Ts

# print("Final state:", x_current)
# print("Simulation finished")





# ########################################### 2D drone point mass without plot ###########################

# import casadi as ca
# import numpy as np

# Ts = 0.1
# def robot_dynamics():
#     # 4 states
#     px = ca.MX.sym('px')
#     py = ca.MX.sym('py')
#     vx = ca.MX.sym('vx')
#     vy = ca.MX.sym('vy')

#     #control
#     ax = ca.MX.sym('ax')
#     ay = ca.MX.sym('ay')

#     state = ca.vertcat(px,py,vx,vy)
#     control =  ca.vertcat(ax,ay)
#     '''
#     physics: p is position!
#         Pdot_x = vx
#         Pdot_y = vy
#         vdot_x = ax
#         vdot_y = ay
    
#     '''
#     # X_k+1 = X_k + Ts Xdot_k --> euler discretization!
#     next_state = state + Ts * ca.vertcat(vx,
#                                          vy,
#                                          ax,
#                                          ay)
#     f = ca.Function('f', [state,control],[next_state]) # gets state and control and gives next_state
#     return f


# f = robot_dynamics()
# # x0 = np.array([0,0,0,0])
# # u0 = np.array([1,0])
# # x1 = f(x0,u0)
# # print(x1)

# '''
# define Q and R and N 
# create opti and variables of optimziation (U)
# x0 as the initil state
# define xref
# define cost and fill it
# define constrinat
# opti.minimize
# opti.solver
# define u_guess as zero for the first iteration
# define while loop and opti.solve
# propagate and change the guess 
# '''
# Q = np.diag([1,1,1,1]) # 4 states
# R = np.diag([1,1]) # 2 controls
# N = 3
# opti = ca.Opti()
# '''
# for N=3
#         U= [ax0 ax1 ax2
#             ay0 ay1 ay2]
#     so:
#         U = [u_k , u_k+1 , u_k+2]

# '''
# U = opti.variable(2,N) # 2 is the control inputs and N is the horizon 
# x0 = opti.parameter(4)
# opti.set_value(x0, np.array([0,0,0,0]))
# X = x0
# xref = np.array([1,1,0,0])
# cost = 0
# '''
#     J? tracking error + control effor penalty + maybe terminal cost?

# '''
# for k in range(N):
#     uk = U[:,k]
#     X = f(X,uk)
#     error = X-xref
#     tracking_error = ca.mtimes([error.T ,Q, error] )
#     control_penalty = ca.mtimes([uk.T,R,uk])
#     cost += tracking_error + control_penalty
# # constraints
# for k in range(N):
#     opti.subject_to(U[0,k] >= -2)
#     opti.subject_to(U[0,k] <= 2)
#     opti.subject_to(U[1,k] >= -2)
#     opti.subject_to(U[1,k] <= 2)


# opti.minimize(cost)
# opti.solver('ipopt')


# u_guess = np.zeros((2,N))


# sim_time = 10
# t = 0
# x_current = np.array([0.0,0.0,0.0,0.0]) # dont use x_current = x0 because x0 is a casadi paramter

# while t<sim_time:
#     if np.linalg.norm(x_current - xref) < 1e-2:
#         break
#     opti.set_value(x0,x_current)
#     opti.set_initial(U,u_guess)
#     sol = opti.solve()
#     u_opt = sol.value(U)
#     u_apply = u_opt[:,0]
#     x_current = np.array(f(x_current,u_apply)).reshape(-1) # reshape(-1) just flattens the 2D array [px,py,vx,vy]
#     u_guess = np.hstack([u_opt[:,1:],np.zeros((2,1))])
#     t+=Ts




# ####################################with plots of the 2D drone!#################################
# import casadi as ca
# import numpy as np
# import matplotlib.pyplot as plt


# Ts = 0.1


# def robot_dynamics():

#     # 4 states
#     px = ca.MX.sym('px')
#     py = ca.MX.sym('py')
#     vx = ca.MX.sym('vx')
#     vy = ca.MX.sym('vy')

#     # control
#     ax = ca.MX.sym('ax')
#     ay = ca.MX.sym('ay')

#     state = ca.vertcat(px, py, vx, vy)
#     control = ca.vertcat(ax, ay)

#     '''
#     physics: p is position!

#         Pdot_x = vx
#         Pdot_y = vy
#         vdot_x = ax
#         vdot_y = ay
#     '''

#     # X_k+1 = X_k + Ts * Xdot_k --> Euler discretization
#     next_state = state + Ts * ca.vertcat(
#         vx,
#         vy,
#         ax,
#         ay
#     )

#     # gets state and control and gives next_state
#     f = ca.Function(
#         'f',
#         [state, control],
#         [next_state]
#     )

#     return f


# f = robot_dynamics()


# # quick test
# # x0_test = np.array([0, 0, 0, 0])
# # u0_test = np.array([1, 0])
# # x1 = f(x0_test, u0_test)
# # print(x1)


# '''
# define Q and R and N
# create opti and variables of optimization (U)
# x0 as the initial state
# define xref
# define cost and fill it
# define constraints
# opti.minimize
# opti.solver
# define u_guess as zero for the first iteration
# define while loop and opti.solve
# propagate and change the guess
# '''

# '''
# smooth drone
# Q = np.diag([10,10,10,10])
# R = np.diag([10,10])

# Aggressive drone
# Q = np.diag([500,500,1,1])
# R = np.diag([0.01,0.01])


# "Stop perfectly" drone
# Q = np.diag([100,100,100,100])
# R = np.diag([0.1,0.1])

# '''

# # Q = np.diag([100, 100, 50, 50])   # 4 states
# # R = np.diag([100, 100])         # 2 controls
# Q = np.diag([10,10,10,10])
# R = np.diag([0.1,0.1])
# N = 30

# opti = ca.Opti()


# '''
# for N = 3:

# U = [ax0 ax1 ax2
#      ay0 ay1 ay2]

# so:

# U = [u_k, u_k+1, u_k+2]
# '''


# # 2 control inputs, N prediction steps
# U = opti.variable(2, N)


# # current state entering the MPC problem
# x0 = opti.parameter(4)

# opti.set_value(
#     x0,
#     np.array([0, 0, 0, 0])
# )


# X = x0


# # desired:
# # px = 1
# # py = 1
# # vx = 0
# # vy = 0
# xref = np.array([1, 1, 0, 0])


# cost = 0


# '''
# J = tracking error + control effort penalty
# '''


# for k in range(N):

#     uk = U[:, k]

#     # predict next state
#     X = f(X, uk)

#     error = X - xref

#     tracking_error = ca.mtimes(
#         [error.T, Q, error]
#     )

#     control_penalty = ca.mtimes(
#         [uk.T, R, uk]
#     )

#     cost += tracking_error + control_penalty


# # constraints
# for k in range(N):

#     # ax constraints
#     opti.subject_to(U[0, k] >= -0.2)
#     opti.subject_to(U[0, k] <= 0.2)

#     # ay constraints
#     opti.subject_to(U[1, k] >= -0.2)
#     opti.subject_to(U[1, k] <= 0.2)


# opti.minimize(cost)


# # quiet IPOPT output
# opti.solver(
#     'ipopt',
#     {'print_time': False},
#     {
#         'print_level': 0,
#         'sb': 'yes'
#     }
# )


# # first guess for all future accelerations
# u_guess = np.zeros((2, N))


# sim_time = 30
# t = 0


# # don't use x_current = x0
# # x0 is a CasADi parameter
# x_current = np.array(
#     [0.0, 0.0, 0.0, 0.0]
# )


# # -----------------------------------
# # history for plotting
# # -----------------------------------

# time_history = [0.0]

# state_history = [
#     x_current.copy()
# ]

# control_history = []


# # -----------------------------------
# # live visualization
# # -----------------------------------

# plt.ion()

# fig, ax_plot = plt.subplots(figsize=(7, 7))


# while t < sim_time:

#     if np.linalg.norm(x_current - xref) < 1e-2:
#         break


#     # tell MPC where the system currently is
#     opti.set_value(
#         x0,
#         x_current
#     )


#     # initial guess for the optimizer
#     opti.set_initial(
#         U,
#         u_guess
#     )


#     # solve MPC problem
#     sol = opti.solve()


#     '''
#     u_opt:

#     [ax0 ax1 ax2
#      ay0 ay1 ay2]
#     '''

#     u_opt = sol.value(U)


#     # MPC only applies the first control
#     u_apply = u_opt[:, 0]


#     # propagate the real/simulated system one timestep
#     x_current = np.array(
#         f(x_current, u_apply)
#     ).reshape(-1)


#     # store history
#     t += Ts

#     time_history.append(t)

#     state_history.append(
#         x_current.copy()
#     )

#     control_history.append(
#         u_apply.copy()
#     )


#     # -----------------------------------
#     # live 2D plot
#     # -----------------------------------

#     states = np.array(state_history)

#     ax_plot.clear()

#     # complete trajectory so far
#     ax_plot.plot(
#         states[:, 0],
#         states[:, 1],
#         'k--',
#         label='trajectory'
#     )


#     # current point mass
#     ax_plot.plot(
#         x_current[0],
#         x_current[1],
#         'bo',
#         markersize=10,
#         label='point mass'
#     )


#     # target
#     ax_plot.plot(
#         xref[0],
#         xref[1],
#         'rx',
#         markersize=12,
#         label='reference'
#     )


#     # current velocity vector
#     ax_plot.quiver(
#         x_current[0],
#         x_current[1],
#         x_current[2],
#         x_current[3],
#         angles='xy',
#         scale_units='xy',
#         scale=1
#     )


#     ax_plot.set_xlim(-0.5, 1.5)
#     ax_plot.set_ylim(-0.5, 1.5)

#     ax_plot.set_xlabel("px [m]")
#     ax_plot.set_ylabel("py [m]")

#     ax_plot.set_title(
#         f"2D Point-Mass MPC   t = {t:.1f} s"
#     )

#     ax_plot.axis('equal')
#     ax_plot.grid()
#     ax_plot.legend()

#     plt.pause(0.1)


#     # -----------------------------------
#     # warm start
#     # -----------------------------------

#     '''
#     Previous optimal solution:

#     [ax0 ax1 ax2
#      ay0 ay1 ay2]

#     We applied [ax0, ay0].

#     Next initial guess:

#     [ax1 ax2 0
#      ay1 ay2 0]
#     '''

#     u_guess = np.hstack([
#         u_opt[:, 1:],
#         np.zeros((2, 1))
#     ])


# plt.ioff()


# print("Final state:")
# print(x_current)

# print("Simulation time:")
# print(t)


# # -----------------------------------
# # convert histories to numpy arrays
# # -----------------------------------

# state_history = np.array(state_history)

# control_history = np.array(control_history)

# time_history = np.array(time_history)


# # -----------------------------------
# # state plots
# # -----------------------------------

# fig_states, axs = plt.subplots(
#     2,
#     2,
#     figsize=(10, 7)
# )


# axs[0, 0].plot(
#     time_history,
#     state_history[:, 0]
# )

# axs[0, 0].axhline(
#     xref[0],
#     linestyle='--'
# )

# axs[0, 0].set_ylabel("px [m]")
# axs[0, 0].grid()


# axs[0, 1].plot(
#     time_history,
#     state_history[:, 1]
# )

# axs[0, 1].axhline(
#     xref[1],
#     linestyle='--'
# )

# axs[0, 1].set_ylabel("py [m]")
# axs[0, 1].grid()


# axs[1, 0].plot(
#     time_history,
#     state_history[:, 2]
# )

# axs[1, 0].axhline(
#     xref[2],
#     linestyle='--'
# )

# axs[1, 0].set_xlabel("time [s]")
# axs[1, 0].set_ylabel("vx [m/s]")
# axs[1, 0].grid()


# axs[1, 1].plot(
#     time_history,
#     state_history[:, 3]
# )

# axs[1, 1].axhline(
#     xref[3],
#     linestyle='--'
# )

# axs[1, 1].set_xlabel("time [s]")
# axs[1, 1].set_ylabel("vy [m/s]")
# axs[1, 1].grid()


# fig_states.suptitle(
#     "MPC State Evolution"
# )

# plt.tight_layout()


# # -----------------------------------
# # control plots
# # -----------------------------------

# if len(control_history) > 0:

#     control_time = time_history[1:]

#     fig_controls, axs_control = plt.subplots(
#         2,
#         1,
#         figsize=(9, 6)
#     )


#     axs_control[0].plot(
#         control_time,
#         control_history[:, 0]
#     )

#     axs_control[0].axhline(
#         0.2,
#         linestyle='--'
#     )

#     axs_control[0].axhline(
#         -0.2,
#         linestyle='--'
#     )

#     axs_control[0].set_ylabel("ax [m/s²]")
#     axs_control[0].grid()


#     axs_control[1].plot(
#         control_time,
#         control_history[:, 1]
#     )

#     axs_control[1].axhline(
#         0.2,
#         linestyle='--'
#     )

#     axs_control[1].axhline(
#         -0.2,
#         linestyle='--'
#     )

#     axs_control[1].set_xlabel("time [s]")
#     axs_control[1].set_ylabel("ay [m/s²]")
#     axs_control[1].grid()


#     fig_controls.suptitle(
#         "Applied MPC Controls"
#     )

#     plt.tight_layout()


# plt.show()





################################ MORE SOPHISTICATED CAR MODEL
'''
X_k+1 = X_k + Ts Xdot_k

x_k+1 = x_k + Ts vk cos (theta_k)
y_k+1 = y_k + Ts vk sin (theta_k)
theta_k+1 = theta_k + Ts v_k/L tan(delta k)
v_k+1 = v_k + Ts ak


L (wheel base length)
a --> acc
delta --> steering angle (positive means turn left and negative measn turn right!)

controls are steering and accelration!

'''





'''
what we did right now was single shooting!
now lets do multiple shooting!

main drawback of single shooting! -->non linearity propagation! too much computations and has affects on convergence problem for large horizions

single shooting --> we had that equality constrinat only at initial state (single point) 

multiple shooting is another way of discretizing the optila problem control to NLP
here the optimization variable is not just u (likesingle shooting) but also x as well!
so we have more optimization variables!and we add an equality constrait as below:
    x(k+1) - f(x(k),u(k)) = 0 --> actual state minus predicted state(model)!

w = [u0 ... u_N-1, x0 ... x_N] --> one u tell me what the next x is but current x doenst tell me what the next u is so length of x is one step longer than length of u

min Q(w)
 w


 subject to g1(w) = [g1(x0,u0)                 
                     ...
                     g1(x_N-1, u_N-1)    <=0  --> map margin and control limits!
                     g1(x_N)
                     ]

        g2(w) = [xbar_0 - x0
                f(x0,u0)-x1
                ...                 =0   --> eqaulity constrinat that satisfies system dynamics
                f(x_N-1,u_N-1)-x_N
            ]

it converges faster and its performance is much superioer than single shooting and instead of initzilaition u we can also initzlie x and this is an advantage!
the only thing we cant init is u but normally we can know how the robot would move like but we are not necesserily familiar with controll action and if i would ike to init control action
i have to solve the dynamics inversly to get control action that satifies this motion profile and init my control action with this motion profile but now with having state as optimizatio nvar
we can propse init for x and plug directly to my initilziation!
drawback is larget NLP but its more sparse and it can be solved faster though!

here we dont propagate but create the equality constraint matrix! in the code!


so single shooting --> casadi only choose u0,u1....,u_N-1 and then every state is created automatically suing the dynamic equation x1=f(x0,u0) ... x3=f(x2,u2)
multiple shooting --> U in R^(2*N) abd X in R^(3*(N+1)) --> for the diff drive problem --> you may choose botht the controla nad predicted states but every neighboring pair of states must obey the dynamics



'''


# # DIFF DRIVE! --> MULTIPLE SHOOTING 
# import casadi as ca
# import numpy as np
# import time

# import matplotlib.pyplot as plt
# def plot_triangle(x, y, theta, size=0.2):

#     # triangle in robot frame
#     points = np.array([
#         [size, 0],
#         [-size/2, size/2],
#         [-size/2, -size/2]
#     ])


#     # rotation matrix

#     R = np.array([
#         [np.cos(theta), -np.sin(theta)],
#         [np.sin(theta),  np.cos(theta)]
#     ])


#     # rotate and translate

#     points = points @ R.T

#     points[:,0] += x
#     points[:,1] += y


#     # close triangle

#     points = np.vstack([points, points[0]])


#     plt.plot(
#         points[:,0],
#         points[:,1],
#         'b-'
#     )


# Ts = 0.1
# def robot_dynamics():
#     x = ca.MX.sym('x')
#     y = ca.MX.sym('y')
#     theta = ca.MX.sym('theta')
#     state = ca.vertcat(x,y,theta)
#     v = ca.MX.sym('v')
#     omega = ca.MX.sym('omega')
#     control = ca.vertcat(v,omega)
#     next_state = state + Ts * ca.vertcat(
#         v*ca.cos(theta),
#         v*ca.sin(theta),
#         omega
#     )
#     f = ca.Function('f',[state,control],[next_state])
#     return f

# f = robot_dynamics()

# # f([0,0,0],[0.5,0])


# Q = np.diag([100,100,1])
# R = np.diag([1.0,0.5])
# N = 20
# opti = ca.Opti()

# # controls
# #
# # [v0 v1 ... vN-1
# #  w0 w1 ... wN-1]
# U = opti.variable(2, N)


# # predicted states
# #
# # [x0 x1 x2 ... xN
# #  y0 y1 y2 ... yN
# #  t0 t1 t2 ... tN]
# X = opti.variable(3, N + 1)

# x0 = opti.parameter(3)
# opti.set_value(x0, np.array([0,0,0]))
# # X = x0
# opti.subject_to(X[:,0] == x0)
# xref = np.array([1.5,1.5,-ca.pi])
# cost = 0
# # construct J(U) --> a function of 6 variables
# for k in range(N):
#     X_k = X[:,k]
#     U_k = U[:,k]
#     X_next = X[:,k+1]
#     X_predicted = f(X_k,U_k)
#     opti.subject_to(X_next==X_predicted) 

#     error = X_next-xref
#     cost += ca.mtimes([error.T,Q,error]) #Tracking error: e^T Q e --> its a scalar --> why use Q ? because not all errors are equal! --> position error is more important than heading error for parking for example!
#     cost += ca.mtimes([U_k.T,R,U_k]) #Control effort penalty

#     # -----------------------------
#     # map constraints
#     # -----------------------------

#     opti.subject_to(-2 <= X_next[0])
#     opti.subject_to(X_next[0] <= 2)

#     opti.subject_to(-2 <= X_next[1])
#     opti.subject_to(X_next[1] <= 2)

# # terminal cost (optinal) --> the final predicted point is important!
# error = X[:,N]-xref
# Qf = np.diag([100,100,10])
# cost += ca.mtimes([error.T,Qf,error])

# # constraints v and omega
# for k in range(N):
#     opti.subject_to(-0.6 <= U[0,k])
#     opti.subject_to(U[0,k] <= 0.6)
#     opti.subject_to(-np.pi/4 <= U[1,k])
#     opti.subject_to(U[1,k] <= np.pi/4)

# # tell optimizer what to minimize
# opti.minimize(cost)
# opti.solver("ipopt")

# # initial guess

# u_guess = np.zeros((2,N)) # starting point fo ipopt

# opti.set_initial(U,u_guess)
# # MPC loop
# sim_time = 10
# t = 0


# #vis
# #vis
# trajectory = []
# x_current = np.array([0,0,0])

# plt.figure(figsize=(7,7))
# plt.axis("equal")
# plt.xlim(-2.5,2.5)
# plt.ylim(-2.5,2.5)
# plt.grid()

# # reference triangle
# plot_triangle(
#     xref[0],
#     xref[1],
#     xref[2],
#     size=0.3
# )

# plt.scatter(
#     xref[0],
#     xref[1],
#     marker='x',
#     s=100,
#     label="reference"
# )

# plt.legend()




# while t < sim_time:
#     if np.linalg.norm(x_current - xref) < 1e-2:
#         break
#     opti.set_value(x0,x_current)
#     opti.set_initial(U,u_guess)
#     start = time.time()
#     sol = opti.solve()
#     end = time.time()
#     mpc_time = end-start
#     '''
#     [v0 v1 v2
#      w0 w1 w2]
#     '''
#     u_opt = sol.value(U)
#     u_apply = u_opt[:,0] # apply only the first control

#     # propagate
#     x_current = np.array(
#         f(x_current,u_apply)
#     ).reshape(-1)


#     # live visualization
#     trajectory.append(x_current.copy())
#     trajectory_np = np.array(trajectory)
#     # clear previous drawing
#     plt.cla()
#     # redraw map
#     plt.xlim(-2.5,2.5)
#     plt.ylim(-2.5,2.5)
#     plt.axis("equal")
#     plt.grid()
#     # reference triangle
#     plot_triangle(
#         xref[0],
#         xref[1],
#         xref[2],
#         size=0.3
#     )
#     # path
#     if len(trajectory_np) > 1:
#         plt.plot(
#             trajectory_np[:,0],
#             trajectory_np[:,1],
#             'k--'
#         )
#     # current robot
#     plot_triangle(
#         x_current[0],
#         x_current[1],
#         x_current[2]
#     )
#     plt.pause(0.2)




#     # warm start --> to make MPC faster
#     # shift old solution one step
#     u_guess = np.hstack(
#         [
#             u_opt[:,1:],
#             np.zeros((2,1))
#         ]
#     )
#     t += Ts

# print("Final state:", x_current)
# print("Simulation finished")




##############################################


'''
Obstace avoidance is just another constraint!

'''

# DIFF DRIVE! --> MULTIPLE SHOOTING + Obstacle avoidance!
import casadi as ca
import numpy as np
import time

import matplotlib.pyplot as plt
def plot_triangle(x, y, theta, size=0.2):

    # triangle in robot frame
    points = np.array([
        [size, 0],
        [-size/2, size/2],
        [-size/2, -size/2]
    ])


    # rotation matrix

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])


    # rotate and translate

    points = points @ R.T

    points[:,0] += x
    points[:,1] += y


    # close triangle

    points = np.vstack([points, points[0]])


    plt.plot(
        points[:,0],
        points[:,1],
        'b-'
    )


Ts = 0.1
def robot_dynamics():
    x = ca.MX.sym('x')
    y = ca.MX.sym('y')
    theta = ca.MX.sym('theta')
    state = ca.vertcat(x,y,theta)
    v = ca.MX.sym('v')
    omega = ca.MX.sym('omega')
    control = ca.vertcat(v,omega)
    next_state = state + Ts * ca.vertcat(
        v*ca.cos(theta),
        v*ca.sin(theta),
        omega
    )
    f = ca.Function('f',[state,control],[next_state])
    return f

f = robot_dynamics()

# f([0,0,0],[0.5,0])


Q = np.diag([100,100,1])
R = np.diag([1.0,0.5])
N = 20
opti = ca.Opti()

# controls
#
# [v0 v1 ... vN-1
#  w0 w1 ... wN-1]
U = opti.variable(2, N)


# predicted states
#
# [x0 x1 x2 ... xN
#  y0 y1 y2 ... yN
#  t0 t1 t2 ... tN]
X = opti.variable(3, N + 1)

x0 = opti.parameter(3)
opti.set_value(x0, np.array([0,0,0]))
# X = x0
opti.subject_to(X[:,0] == x0)
xref = np.array([1.5,1.5,-ca.pi])

# obstacle
obs_x = 1.2
obs_y = 0.8
obs_r = 0.2
robot_r = 0.2
safe_r = obs_r + robot_r


cost = 0
# construct J(U) --> a function of 6 variables
for k in range(N):
    X_k = X[:,k]
    U_k = U[:,k]
    X_next = X[:,k+1]
    X_predicted = f(X_k,U_k)
    opti.subject_to(X_next==X_predicted) 

    error = X_next-xref
    cost += ca.mtimes([error.T,Q,error]) #Tracking error: e^T Q e --> its a scalar --> why use Q ? because not all errors are equal! --> position error is more important than heading error for parking for example!
    cost += ca.mtimes([U_k.T,R,U_k]) #Control effort penalty

    # -----------------------------
    # map constraints
    # -----------------------------

    opti.subject_to(-2 <= X_next[0])
    opti.subject_to(X_next[0] <= 2)

    opti.subject_to(-2 <= X_next[1])
    opti.subject_to(X_next[1] <= 2)

    # obstacle avoidance constraint
    distance_squared = (
        (X_next[0]-obs_x)**2 +
        (X_next[1]-obs_y)**2
    )

    opti.subject_to(
        distance_squared >= safe_r**2
    )


# terminal cost (optinal) --> the final predicted point is important!
error = X[:,N]-xref
Qf = np.diag([100,100,10])
cost += ca.mtimes([error.T,Qf,error])

# constraints v and omega
for k in range(N):
    opti.subject_to(-0.6 <= U[0,k])
    opti.subject_to(U[0,k] <= 0.6)
    opti.subject_to(-np.pi/4 <= U[1,k])
    opti.subject_to(U[1,k] <= np.pi/4)

# tell optimizer what to minimize
opti.minimize(cost)
opti.solver("ipopt")

# initial guess

u_guess = np.zeros((2,N)) # starting point fo ipopt

opti.set_initial(U,u_guess)
# MPC loop
sim_time = 10
t = 0


#vis
#vis
trajectory = []
x_current = np.array([0,0,0])

plt.figure(figsize=(7,7))
plt.axis("equal")
plt.xlim(-2.5,2.5)
plt.ylim(-2.5,2.5)
plt.grid()

# reference triangle
plot_triangle(
    xref[0],
    xref[1],
    xref[2],
    size=0.3
)

plt.scatter(
    xref[0],
    xref[1],
    marker='x',
    s=100,
    label="reference"
)

plt.legend()



circle = plt.Circle(
    (obs_x,obs_y),
    obs_r,
    color='red',
    fill=False
)

plt.gca().add_patch(circle)



while t < sim_time:
    if np.linalg.norm(x_current - xref) < 1e-2:
        break
    opti.set_value(x0,x_current)
    opti.set_initial(U,u_guess)
    start = time.time()
    sol = opti.solve()
    end = time.time()
    mpc_time = end-start
    '''
    [v0 v1 v2
     w0 w1 w2]
    '''
    u_opt = sol.value(U)
    u_apply = u_opt[:,0] # apply only the first control

    # propagate
    x_current = np.array(
        f(x_current,u_apply)
    ).reshape(-1)


    # live visualization
    trajectory.append(x_current.copy())
    trajectory_np = np.array(trajectory)
    # clear previous drawing
    plt.cla()
    # redraw map
    plt.xlim(-2.5,2.5)
    plt.ylim(-2.5,2.5)
    plt.axis("equal")
    plt.grid()

    circle = plt.Circle(
        (obs_x,obs_y),
        obs_r,
        color='red',
        fill=False
    )

    plt.gca().add_patch(circle)


    # reference triangle
    plot_triangle(
        xref[0],
        xref[1],
        xref[2],
        size=0.3
    )
    # path
    if len(trajectory_np) > 1:
        plt.plot(
            trajectory_np[:,0],
            trajectory_np[:,1],
            'k--'
        )
    # current robot
    plot_triangle(
        x_current[0],
        x_current[1],
        x_current[2]
    )
    plt.pause(0.2)




    # warm start --> to make MPC faster
    # shift old solution one step
    u_guess = np.hstack(
        [
            u_opt[:,1:],
            np.zeros((2,1))
        ]
    )
    t += Ts

print("Final state:", x_current)
print("Simulation finished")