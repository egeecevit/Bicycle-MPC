import matplotlib.pyplot as plt
import numpy as np

## /10 path
velocities = np.arange(2.0,11.0,1.0) # m/s
## J path
# max_cte_values_case1 = [0.09756703802860815,0.10872536556723846,0.1320558504432906,0.1464369489286176,0.16003291228656621,0.18557512161392297,0.19760975768402095,0.1984446673626185,0.2810629037945678] # mpc no rate model with rate #worst
# max_cte_values_case2 = [0.0976855030723813,0.10963707230870273,0.13222032265783973,0.14628385804306698,0.15984625292590082,0.18424775480280625,0.1955022203701063,0.19704366413188075,0.27978988949090333] # mpc rate model with rate #second best
# max_cte_values_case3 = [0.09719343235318176,0.10908467531815333,0.13074981650025935,0.14576970947574389,0.15688659485944262,0.18050487884829955,0.18933170863469287,0.19442887959332764,0.21117208028458453] # mpc no rate model no rate #best

max_cte_values_case1 = [0.09756703802860815,0.10872536556723846,0.1320558504432906,0.1464369489286176,0.16003291228656621,0.18557512161392297,0.19760975768402095,0.1984446673626185,0.2810629037945678] # mpc no rate model with rate #worst
max_cte_values_case2 = [0.0976855030723813,0.10963707230870273,0.13222032265783973,0.14628385804306698,0.15984625292590082,0.18424775480280625,0.1955022203701063,0.19704366413188075,0.27978988949090333] # mpc rate model with rate #second best
max_cte_values_case3 = [0.09719343235318176,0.10908467531815333,0.13074981650025935,0.14576970947574389,0.15688659485944262,0.18050487884829955,0.18933170863469287,0.19442887959332764,0.21117208028458453] # mpc no rate model no rate #best



# # Plot the CTE vs Velocity
# plt.figure()
# plt.plot(velocities, max_cte_values, color='r', marker='o', linestyle='-', label='CTE vs Velocity')
# plt.xlabel("Velocity (m/s)")
# plt.ylabel("Cross Track Error (CTE)")
# plt.title("CTE vs Velocity for Different Reference Velocities")
# plt.legend()
# plt.grid(True)
# plt.show()

# plt.figure()

plt.plot(velocities, max_cte_values_case1, color='r', marker='s', linestyle='-', label='MPC Has No Rate Model With Rate')
plt.plot(velocities, max_cte_values_case2, color='g', marker='^', linestyle='--', label='MPC and Model Both With Rate')
plt.plot(velocities, max_cte_values_case3, color='b', marker='d', linestyle='-', label='MPC and Model Has No Rate')

# Labels, title, legend, and grid
plt.xlabel("Velocity (m/s)")
plt.ylabel("Cross Track Error (CTE)")
plt.title("CTE vs Velocity for Different Cases")
plt.legend(fontsize=12)
plt.grid(True)

# Show the plot
plt.show()
