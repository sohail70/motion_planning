import matplotlib.pyplot as plt
import glob

# List all robot_costs_#.txt files
cost_files = glob.glob('../build/robot_costs_*.txt')

# Initialize a list to store all costs from different files
all_costs = []

# Loop through all files and read the costs
for file in cost_files:
    costs = []
    with open(file, 'r') as f:
        for line in f:
            try:
                costs.append(float(line.strip()))
            except ValueError:
                pass  # Skip any lines that are not valid floats
    all_costs.append(costs)

# Plot the costs for each file
plt.figure(figsize=(10, 6))

for i, costs in enumerate(all_costs, start=1):
    plt.plot(costs, label=f'robot_costs_{i}')

# Add labels and title
plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('Robot Costs Over Time')
plt.legend()  # Display a legend
plt.grid(True)

# Show the plot
plt.show()