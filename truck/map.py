from scipy.io import loadmat
from scipy.interpolate import interp2d
import matplotlib.pyplot as plt
import numpy as np
import casadi as ca

engine_data = loadmat('truck/enginedata.mat')
engine_map = loadmat('truck/enginemap.mat')['data_engine_map']
print(engine_data)
print(engine_map)

from scipy.io import loadmat
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

# Assuming engine_map contains your data
engine_map = loadmat('truck/enginemap.mat')['data_engine_map']

# Original x, y, and z values
x = engine_map[:, 0]  # Engine Speed (RPM)
y = engine_map[:, 1]*5  # Torque (Nm)

z = engine_map[:, 2]  # Power (W)
z = clipped_arr = np.clip(z, None, 410)
# Create a grid
xi, yi = np.meshgrid(np.linspace(x.min(), x.max(), 1000), 
                     np.linspace(y.min(), y.max(), 1000))

# Interpolate z values on this grid
zi = griddata((x, y), z, (xi, yi), method='cubic')

# Plotting RdYlGn
# coutourf = plt.contourf(xi, yi, zi, levels=20, cmap="viridis")
plt.figure(figsize=(6,4))
coutourf = plt.contourf(xi, yi, zi, levels=10, cmap="viridis")
contours = plt.contour(xi, yi, zi, 10, colors='yellow')
plt.clabel(contours, inline=True, fontsize=8)
plt.xlabel('Engine Speed (RPM)')
plt.ylabel('Torque (Nm)')
plt.xlim((1000,6000))
plt.ylim((100, 740))
plt.colorbar(coutourf, label='Power (W)')
plt.show()

kwh_map = engine_data['fc_map_gpkwh_map']
spd_list = engine_data['fc_map_spd_list'].flatten()
trq_list = engine_data['fc_map_trq_list'].flatten()


# Note: Ensure spd_list and trq_list are 1D arrays of the unique speed and torque values, respectively
fc_interp = interp2d(spd_list, trq_list, kwh_map, kind='cubic')

# Now, you can use fc_interp to find fuel consumption at any speed and torque within the range
# For example, to predict fuel consumption at a speed of 1500 RPM and torque of 200 Nm, you do:
speed_example = 1500  # Example engine speed
torque_example = 200  # Example torque
fc_prediction = fc_interp(speed_example, torque_example)

print(f"Predicted fuel consumption at {speed_example} RPM and {torque_example} Nm: {fc_prediction} g/kWh")

# If you want to visualize the interpolated data over a grid, you can create a meshgrid and evaluate the function over it
spd_grid, trq_grid = np.meshgrid(np.linspace(spd_list.min(), spd_list.max(), 100),
                                 np.linspace(trq_list.min(), trq_list.max(), 100))

# You've already flattened spd_grid and trq_grid for interpolation:
fc_grid_flat = fc_interp(np.linspace(spd_list.min(), spd_list.max(), 100), np.linspace(trq_list.min(), trq_list.max(), 100))
print("spd_grid.flatten() ", spd_grid.flatten().shape)
print("fc_grid_flat shape: ", fc_grid_flat.shape)
print(fc_grid_flat)

# Now, reshape fc_grid_flat back to a 2D array that matches the shape of spd_grid and trq_grid
fc_grid = fc_grid_flat.reshape(spd_grid.shape)

# Confirming the shape after reshaping
print("Reshaped fc grid:", fc_grid.shape)

# Now, plotting should work without the shape mismatch error
plt.figure(figsize=(6,4))
contour = plt.contourf(spd_grid, trq_grid, fc_grid, levels=20, cmap='viridis')
contours = plt.contour(spd_grid, trq_grid, fc_grid, 20, colors='yellow')
plt.clabel(contours, inline=True, fontsize=8)
plt.xlabel('Engine Speed (RPM)')
plt.ylabel('Torque (Nm)')
plt.title('Fuel Efficiency Map')
plt.colorbar(coutourf, label='Fuel Efficiency (g/kwh)')
plt.show()


M, Av, rho, Cd, mu, g = 4800, 2.5, 1.184, 0.6, 0.006, 9.81
wheel_r = 0.5
gear_list = [7.59, 5.06, 3.38, 2.25, 1.5, 1.0, 0.75]
final_drive = 3.1; trans_eff = 0.92
k1 = 0.5*Cd*rho*Av/M ; k2 = mu*g; k3 = g

Te_max = 720
v_range = np.linspace(0., 30., 300)
# u_range = np.linspace(0.05, 2.25, 10)
a_range = np.linspace(0.00, 2.0, 20)
# v_grid, u_grid = np.meshgrid(v_range,
#                              u_range)
# print("v_grid: ", v_grid)
# print("u_grid: ", u_grid)
data_list = []
v_list = []
u_list = []
fr_list = []
opt_gear_list = [] # gear id : 1,2,...,7
for i in range(len(v_range)):
    for k in range(len(a_range)):
        v = v_range[i]
        # u = u_range[k]
        a = a_range[k]
        u = a + k1*v**2 + k2 + k3*0
        F = u*M
        Tw = F* wheel_r
        opt_fr = np.inf
        opt_gear = 0
        for g in range(len(gear_list)):
            gear_ratio = gear_list[g] * final_drive
            Te = Tw / gear_ratio / trans_eff 
            # if Te >= Te_max:
            #     continue;
            we = max(62, v/wheel_r * gear_ratio) # idoling rotation speed 62 rad/s, 600rpm
            pe = Te * we 
            pe_kwh = pe/3600/1000
            feiff = fc_interp(we, Te)
            fc_g = feiff*pe_kwh 
            fc_ml = fc_g /0.85
            if fc_ml < opt_fr:
                opt_fr = fc_ml
                opt_gear = g+1

            print("************** gear: ", g+1)
            print("we: {}, Te: {}, pe: {}, pe_kwh: {}, feiff: {}".format(we, Te, pe, pe_kwh, feiff ))
            print("v, u: ", u,v)
            print("fc_g, ml:", fc_g, fc_ml)
        v_list.append(v)
        u_list.append(u)
        fr_list.append(opt_fr[0])
        opt_gear_list.append(opt_gear)
        # data_list.append()
        data_list.append([v, u, opt_fr[0], opt_gear])
print("v list:", v_list)
print("u list:", u_list)
print("fr list:", fr_list)
print("gear list:", opt_gear_list)
data_list.sort(key=lambda x: x[0])
print("data_list: ", data_list)            

data = np.array(data_list)
print(data.shape)
nr,nc = data.shape

p = ca.SX.sym('p', nr, nc)
X = ca.SX.sym('X', 8)
J = 0
g = []
for i in range(len(data_list)):
    prediction = X[0] + X[1]* p[i,0] + X[2] * p[i,0]**2 + \
        X[3] * p[i,0]**3 + X[4]*p[i,0]**4 + \
        (X[5] + X[6]*p[i,0] + X[7]*p[i,0]**2) * p[i,1]
    y = p[i,2]
    # J += y*(y-prediction)**2
    J += (y-prediction)**2
    g.append(prediction)
g = ca.vertcat(*g)
lb_g = 0.058 * np.ones(len(data_list))
ub_g = 30.0 * np.ones(len(data_list))
lb_x = -np.inf * np.ones(8)
ub_x = np.inf * np.ones(8)
lb_x = np.zeros(8)
nlp = {'x': X, 'f': J, 'g': g, 'p': p}
opts = {'ipopt.print_level': 5,'print_time': True,'ipopt.tol': 1e-8}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
x_guess = ca.vertcat([4.46e-1, -1.127e-1, 1.96e-2, -1.01e-3, 1.65e-5, 2.68e-1,3.43e-1,7.6e-4])
res = solver(x0 = x_guess, lbx = lb_x, ubx = ub_x, lbg = lb_g, ubg = ub_g, p = data)
print("res: ", res)

params = res['x'].toarray()
o0,o1,o2,o3,o4,c0,c1,c2 = params.flatten()
o0,o1,o2,o3,o4,c0,c1,c2 = [3.35052784e-01, 9.09017093e-03, 3.75745106e-08, 3.49357492e-8,
 6.97402869e-08, 1.65498859e-01, 3.60700787e-01, 2.42297665e-04]
p_data_list = []
error_pct_list = []
error_list = []
for array in data_list:
    v, u, fr, gear = array
    p_fr = o0 + o1*v +o2*v**2 + o3*v**3 + o4 *v**4 +\
            (c0 + c1*v + c2*v**2) * u
    p_fr = p_fr
    error = abs(fr-p_fr)
    error_pct = error / fr * 100
    array.append(p_fr); array.append(error); array.append(error_pct)
    error_pct_list.append(error_pct)
    error_list.append(error)
    p_data_list.append(array)
avg_error_pct = sum(error_pct_list)/ len(error_pct_list)
avg_error = sum(error_list) / len(error_list)
print("predicted data: ", p_data_list)
print("average percentage prediciton error: ", avg_error_pct)
print("average absolute error (ml/s): ", avg_error)
print(" parameters: ",params.flatten())

# o0,o1,o2,o3,o4,c0,c1,c2 = [3.35052784e-01, 9.09017093e-03, 0.00000000e+00, 0.00000000e+00,
#  6.97402869e-08, 1.65498859e-01, 3.60700787e-01, 2.42297665e-04]
xv, xu = 30., 2.22536
fr_test =  o0 + o1*xv + o2*xv**2 + o3*xv**3 + (c0 + c1*xv + c2*xv**2)*xu

print(" xv: {},  xu: {} , fr_test: {} ".format(xv, xu, fr_test))
# Build a meshgrid
v_plot = np.arange(0, 30, 0.1)
u_plot = np.arange(0.1, 2.5, 0.1)
V, A = np.meshgrid(v_plot, u_plot)
# Calculate fuel consumption model fv for each combination of v and a
FV = o0 + o1*V + o2*V**2 + o3*V**3 + (c0 + c1*V + c2*V**2)*A
# Plotting contour map
plt.figure(figsize=(8,4))
contour = plt.contourf(V, A, FV, levels=40, cmap='RdYlGn_r')  # Reversed Red-Yellow-Green colormap for desired color scheme
plt.colorbar(contour)
contour_lines = plt.contour(V, A, FV, levels=40, colors='black', linewidths=0.5)  # Highlight contour lines in black
plt.clabel(contour_lines, inline=True, fontsize=8, fmt='%1.1f')
plt.title('Fitted Fuel Consumption Model Contour Map')
plt.xlabel('Vehicle Velocity (v) [m/s]')
plt.ylabel('Traction Acceleration (u) [m/s^2]')
plt.xlim((v_plot[0], v_plot[-1]))
plt.ylim((u_plot[0], u_plot[-1]))
plt.show()


import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

# # Example data_list
# data_list = [
#     [10, 0.1, 2.5, 1],
#     [15, 0.15, 2.8, 2],
#     [20, 0.2, 3.0, 3],
#     # Add more data points as needed
# ]

# Unpack the data into separate lists
v = [item[0] for item in data_list]
u = [item[1] for item in data_list]
opt_gear = [item[3] for item in data_list]

# Generating a random colormap
# Creating a list of random colors
num_gears = max(opt_gear)  # Assuming the max gear value represents the number of gears
random_colors = np.random.rand(num_gears, 3)  # Generate random colors
random_cmap = ListedColormap(random_colors)

# Create the contour plot using the random colormap
plt.figure(figsize=(10, 6))
contour = plt.tricontourf(v, u, opt_gear, levels=np.arange(0.5, num_gears+1.5, 1), cmap='RdYlGn_r')
plt.colorbar(contour, ticks=range(1, num_gears+1))

contour_lines = plt.tricontour(v, u, opt_gear, levels=np.arange(0.5, num_gears+1.5, 1), colors='k', linewidths=0.5)
plt.clabel(contour_lines, inline=True, fontsize=8, fmt='%1.1f')
# Labeling
plt.xlabel('Vehicle Speed (v) [m/s]')
plt.ylabel('Attraction Acceleration (u) [m/s^2]')
plt.title('Optimal Gear Selection Map')

plt.show()

