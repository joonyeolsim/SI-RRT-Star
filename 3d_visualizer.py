import re

import matplotlib.pyplot as plt


# Function to parse the file content and extract the paths
def parse_paths(file_content):
    paths = []
    agent_data = file_content.split('\n')

    for agent in agent_data:
        if agent:
            points = re.findall(r'\((.*?)\)', agent)
            path = []
            for point in points:
                x, y, time = map(float, point.split(','))
                path.append((x, y, time))
            paths.append(path)
    return paths


# Read the file content
file_path = 'solution.txt'
with open(file_path, 'r') as file:
    file_content = file.read()

# Parse the file content
paths = parse_paths(file_content)

# Creating the 3D plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot each path and draw spheres at each point
for i, path in enumerate(paths):
    if i == 0 or i == 26:
        x, y, time = zip(*path)
        ax.plot(x, y, time, color='k')  # Plotting the path

# Setting labels
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Time')

# Show the plot
plt.show()
