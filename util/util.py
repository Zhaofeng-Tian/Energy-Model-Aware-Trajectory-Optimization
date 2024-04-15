from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import pickle

def get_leading_profile(cycle = 'HWFET', plot=False):
    # Load the data
    if cycle == 'HWFET':
        data = loadmat('util/CYC_HWFET.mat') 
    elif cycle == 'EUDC':
        data = loadmat('util/CYC_EUDC.mat') 
    elif cycle == 'INDIA_HWY':
        data = loadmat('util/CYC_INDIA_HWY_SAMPLE')
    elif cycle == 'INDIA_URBAN':
        data = loadmat('util/CYC_INDIA_URBAN_SAMPLE')
    elif cycle == 'MANHATTAN':
        data = loadmat('util/CYC_MANHATTAN')
    elif cycle == 'NYCC':
        data = loadmat('util/CYC_NYCC')
    elif cycle == 'NYCTRUCK':
        data = loadmat('util/CYC_NYCTRUCK')
    else:
        raise ValueError(" Data File Not Found!")
    cyc_mph_data = data['cyc_mph']

    # Original time and velocity
    time_seconds_original = cyc_mph_data[:, 0]
    speed_mph_original = cyc_mph_data[:, 1]

    # Create a new time array with 0.1 second intervals
    time_seconds_fine = np.arange(time_seconds_original[0], time_seconds_original[-1], 0.1)

    # Interpolate speed to the finer time grid
    speed_interpolator = interp1d(time_seconds_original, speed_mph_original, kind='linear')
    speed_mph_fine = speed_interpolator(time_seconds_fine)

    # Convert speed to meters per second for calculations
    speed_mps_fine = speed_mph_fine * (1609.34 / 3600)

    # Calculate distance by integrating velocity over time on the finer time grid
    distance_meters_fine = np.cumsum(speed_mps_fine * 0.1)  # Use 0.1 seconds as the time step for integration

    # Calculate acceleration (difference in velocity / time interval) on the finer time grid
    acceleration_mps2_fine = np.diff(speed_mps_fine, prepend=[speed_mps_fine[0]]) / 0.1


    if plot:
        # Plotting
        plt.figure(figsize=(14, 10))

        # Time limits for plots
        time_min, time_max = time_seconds_fine[0], time_seconds_fine[-1]

        # Distance plot
        plt.subplot(3, 1, 1)
        plt.plot(time_seconds_fine, distance_meters_fine, label='Distance (m)')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.title('Distance Profile')
        plt.xlim(time_min, time_max)
        plt.grid(True)

        # Velocity plot in m/s
        plt.subplot(3, 1, 2)
        plt.plot(time_seconds_fine, speed_mps_fine, label='Velocity (m/s)', color='orange')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity Profile')
        plt.xlim(time_min, time_max)
        plt.grid(True)

        # Acceleration plot
        plt.subplot(3, 1, 3)
        plt.plot(time_seconds_fine, acceleration_mps2_fine, label='Acceleration (m/s²)', color='green')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Acceleration Profile')
        plt.xlim(time_min, time_max)
        plt.grid(True)

        plt.tight_layout()
        plt.show()
    # Return the full profiles as dictionaries for easier access
    return {
        'time': time_seconds_fine,
        'velocity_mps': speed_mps_fine,
        'distance_meters': distance_meters_fine,
        'acceleration_mps2': acceleration_mps2_fine
    }

def local_approx( profile, time_target, delta_time,dt = 0.1,acc_approx = True, plot=False):
    profiles = profile

    # time_interval = np.arange(time_target - delta_time, time_target + delta_time + dt, dt)
    time_interval = np.arange(time_target, time_target + delta_time + dt, dt)
    # Find the index for 100 seconds in the fine time grid
    time_index = np.searchsorted(profiles['time'], time_target)

    # Use local velocity (in m/s) and acceleration (in m/s^2) at 100 seconds for prediction
    local_velocity = profiles['velocity_mps'][time_index]
    local_acceleration = profiles['acceleration_mps2'][time_index]

    # Predict distance change using velocity and acceleration
    # For simplicity in this approximation, use s = ut + 0.5at^2 where
    # s is the distance change, u is the initial velocity, a is the acceleration, and t is the time change from 100 seconds
    time_change = time_interval - time_target
    predicted_distance_change = local_velocity*time_change
    if acc_approx:
        predicted_distance_change = local_velocity * time_change + 0.5 * local_acceleration * time_change**2

    # Real distance change based on actual data
    # Calculate the distance at 90 seconds and at 110 seconds, then find the change relative to the distance at 100 seconds
    real_distance_at_100 = profiles['distance_meters'][time_index]
    real_distance_change = profiles['distance_meters'][np.searchsorted(profiles['time'], time_interval)] - real_distance_at_100

    if plot:
        # Plot comparison
        plt.figure(figsize=(10, 6))
        plt.plot(time_interval, predicted_distance_change, label='Predicted Distance Change', linestyle='--', color='blue')
        plt.plot(time_interval, real_distance_change, label='Real Distance Change', linestyle='-', color='red')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance Change (m)')
        plt.title('Predicted vs. Real Distance Change around 100 Seconds')
        plt.legend()
        plt.grid(True)
        plt.show()
    # Local velocity and acceleration at the target time
    local_velocity = profiles['velocity_mps'][time_index]
    local_acceleration = profiles['acceleration_mps2'][time_index] 

    # Predicted profiles
    time_change = time_interval - time_target
    predicted_velocities = local_velocity + local_acceleration * time_change
    predicted_positions = profiles['distance_meters'][time_index] + local_velocity * time_change + 0.5 * local_acceleration * time_change**2
    predicted_accelerations = np.full_like(time_interval, local_acceleration)

    # Plotting if required
    if plot:
        plt.figure(figsize=(15, 5))

        # Plot predicted velocities
        plt.subplot(1, 3, 1)
        plt.plot(time_interval, predicted_velocities, label='Predicted Velocities')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Predicted Velocities')
        plt.legend()
        plt.grid(True)

        # Plot predicted positions
        plt.subplot(1, 3, 2)
        plt.plot(time_interval, predicted_positions, label='Predicted Positions')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Predicted Positions')
        plt.legend()
        plt.grid(True)

        # Plot constant acceleration
        plt.subplot(1, 3, 3)
        plt.plot(time_interval, predicted_accelerations, label='Predicted Accelerations')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Predicted Accelerations')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()
    return predicted_positions, predicted_velocities, predicted_accelerations
    
def calc_at(v, av, theta,k):
    """ 
    Calculate acceleration of tracktion.
    """
    k1,k2,k3 = k
    at = av+ k1*v**2 + k2*np.cos(theta) + k3*np.sin(theta)
    at = max(0.,at)
    return at

def calc_fuel(v, at,c,o):

    c0, c1, c2 = c
    o0,o1,o2,o3,o4 = o
    fu = o0 + o1*v + o2*v**2 + o3*v**3 + o4*v**4 + (c0 + c1*v + c2*v**2)*at
    return fu


def get_gradient_profile(feature = 'steep',plot=False):
    # Given parameters
    if feature == 'normal':
        dslope1, dslope2, dslope3 = 0.04, 0.02,0.00
        # len_wave1, len_wave2, len_wave3 = 1870,1136,976  # Example wavelength
        len_wave1, len_wave2, len_wave3 = 2870,2136,1976  # Example wavelength
    elif feature == 'steep':
        dslope1, dslope2, dslope3 = 0.05, 0.02,0.01
        # len_wave1, len_wave2, len_wave3 = 1380,860,430  # Example wavelength
        len_wave1, len_wave2, len_wave3 = 2380,1860,1430  # Example wavelength
    else:
        dslope1, dslope2, dslope3 = 0.05, 0.02,0.00
        len_wave1, len_wave2, len_wave3 = 1870,1136,976  # Example wavelengt

    print(dslope1*(2*np.pi/len_wave1) + dslope2*(2*np.pi/len_wave2) + dslope1*(2*np.pi/len_wave2))
    initial_altitude = 100  # Initial altitude in meters
    len_road = 20000 # 20km
    n_points = int(len_road)
    len_point = len_road/n_points
    # Define the range of x values
    x = np.linspace(0, len_road, n_points)  # From 0 to 100000 meters

    # Calculate the slope for the same range of x values
    slope = dslope1 * np.sin(2 * np.pi * x / len_wave1) + dslope2 * np.sin(2 * np.pi * x / len_wave2) \
            + dslope3*np.sin(2*np.pi*x/len_wave3)
    if feature == 'steep':
        # slope += 0.05
        slope += 0.02
    elif feature == 'flat':
        slope = np.zeros(len(slope))

    g_slope = np.diff(slope, n=1)/len_point

    print(" gradient of the slope: ", g_slope)
    print("maximum gradient: ", np.max(g_slope))
    tan_slope =  np.tan(slope)
    # Calculate the integral of the slope function to get altitude
    # The integral of a*sin(bx) is -a/b * cos(bx) + constant
    altitude = initial_altitude - (dslope1 * len_wave1 / (2 * np.pi) * np.cos(2 * np.pi * x / len_wave1)) - \
            (dslope2 * len_wave2 / (2 * np.pi) * np.cos(2 * np.pi * x / len_wave2))-\
            (dslope3 * len_wave3 / (2 * np.pi) * np.cos(2 * np.pi * x / len_wave3))
    altitude = np.cumsum(tan_slope) + initial_altitude
    # The constants of integration are adjusted so that the altitude starts from 100 meters
    # Adjust the altitude by subtracting the initial adjustment to start from 100 meters
    # altitude_adjustment = (dslope1 * len_wave1 / (2 * np.pi)) + (dslope2 * len_wave2/ (2 * np.pi)) +(dslope3*len_wave3/(2*np.pi))
    # altitude += altitude_adjustment
    if plot==True:
        # Create subplots to display both altitude and slope
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))

        # Plot altitude
        axs[0].plot(x, altitude, label='Altitude', color='blue')
        axs[0].set_title('Altitude along the x-axis')
        axs[0].set_xlabel('Distance (m)')
        axs[0].set_ylabel('Altitude (m)')
        # axs[0].set_ylim(70, 2000)
        axs[0].grid(True)
        axs[0].legend()

        # Plot slope
        axs[1].plot(x, slope, label='gradient', color='red')
        axs[1].set_title('Gradient along the x-axis')
        axs[1].set_xlabel('Distance (m)')
        axs[1].set_ylabel('Gradient')
        axs[1].grid(True)
        axs[1].legend()

        # Plot slope
        axs[2].plot(x, 100*tan_slope, label='slope', color='red')
        axs[2].set_title('Slope along the x-axis')
        axs[2].set_xlabel('Distance (m)')
        axs[2].set_ylabel('Slope (%)')
        axs[2].grid(True)
        axs[2].legend()
        plt.tight_layout()
        plt.show()
    
    return altitude, slope


        
# # Note: When implementing this, ensure the plotting code is updated to handle the finer time resolution if 'plot=True'.
if __name__ == "__main__":
    # 'HWFET', 'EUDC
    # profile = get_leading_profile(cycle='NYCTRUCK', plot = True)
    # print("time: ", profile["time"])
    # print("acceleration: ", profile["acceleration_mps2"])
    # print("len of acceleration: ", len(profile["acceleration_mps2"]))
    # print(" Leading init distance: ", profile['distance_meters'][500])
    # print(" Ego init distance: ", profile['distance_meters'][500]-100)

    # local_approx(profile,time_target=150, delta_time=10,dt=0.1, acc_approx = True, plot=True)

    _ , gd_profile = get_gradient_profile(feature='normal', plot=True)
    print(gd_profile[551])

    # file = "data/car_HWFET_flat_SQP_0.1_5_5_gd_True_ptime_10.pkl"
    # rder, param = get_data(file)