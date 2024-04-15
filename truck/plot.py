import matplotlib.pyplot as plt
import numpy as np

# o0,o1,o2,o3,o4,c0,c1,c2 = [3.35052784e-01, 9.09017093e-03, 0.00000000e+00, 0.00000000e+00,
#  6.97402869e-08, 1.65498859e-01, 3.60700787e-01, 2.42297665e-04]

o0,o1,o2,o3,o4,c0,c1,c2 = [3.35052784e-01, 9.09017093e-03, 3.75745106e-08, 3.49357492e-8,
 6.97402869e-08, 1.65498859e-01, 3.60700787e-01, 2.42297665e-04]

xv, xu = 30., 2.22536
fr_test =  o0 + o1*xv + o2*xv**2 + o3*xv**3 + (c0 + c1*xv + c2*xv**2)*xu

print(" xv: {},  xu: {} , fr_test: {} ".format(xv, xu, fr_test))
# Build a meshgrid
v_plot = np.arange(0, 30, 0.1)
u_plot = np.arange(0.1, 2.6, 0.1)
V, A = np.meshgrid(v_plot, u_plot)
# Calculate fuel consumption model fv for each combination of v and a
FV = o0 + o1*V + o2*V**2 + o3*V**3 + (c0 + c1*V + c2*V**2)*A
# Plotting contour map
title_settings = {'family': 'Times New Roman', 'size': 18}
font_settings = {'family': 'Times New Roman', 'size': 16}
plt.figure(figsize=(8,6))
contour = plt.contourf(V, A, FV, levels=40, cmap='RdYlGn_r')  # Reversed Red-Yellow-Green colormap for desired color scheme
cbar = plt.colorbar(contour)
cbar.set_label('Fuel Efficiency', fontdict=font_settings, labelpad=20)
contour_lines = plt.contour(V, A, FV, levels=40, colors='black', linewidths=0.5)  # Highlight contour lines in black
plt.clabel(contour_lines, inline=True, fontsize=8, fmt='%1.1f')
plt.title('Fuel Consumption Model Contour Map',  fontdict=title_settings)
plt.xlabel('Vehicle Velocity (v) [m/s]', fontdict=font_settings)
plt.ylabel('Traction Acceleration (u) [m/s^2]', fontdict=font_settings)
plt.xlim((v_plot[0], v_plot[-1]))
plt.ylim((u_plot[0], u_plot[-1]))
plt.xticks(fontsize=14, fontname='Times New Roman')
plt.yticks(fontsize=14, fontname='Times New Roman')
plt.show()