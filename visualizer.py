import argparse
import re

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Circle, Rectangle


def parse_data(data):
    agents = {}
    no_path_agents = []
    for line in data:
        match = re.match(r'Agent (\d+):', line)
        if match:
            agent_id = int(match.group(1))
            points = re.findall(r'\(([^)]+)\)', line)
            if points:
                agents[agent_id] = [tuple(map(float, point.split(','))) for point in points]
            else:
                no_path_agents.append(agent_id)
    return agents, no_path_agents


def interpolate_path(path):
    interpolated_path = []
    for i in range(len(path) - 1):
        x0, y0, t0 = path[i]
        x1, y1, t1 = path[i + 1]
        times = np.linspace(t0, t1, int((t1 - t0) * sample))
        xs = np.linspace(x0, x1, len(times))
        ys = np.linspace(y0, y1, len(times))
        interpolated_path.extend(list(zip(xs, ys, times)))
    return interpolated_path


def add_start_end_points(ax, agents):
    for agent_id, path in agents.items():
        if not path:
            continue
        start_x, start_y, _ = path[0]
        end_x, end_y, _ = path[-1]
        ax.plot(start_x, start_y, 'go')
        ax.plot(end_x, end_y, 'ro')

        # 에이전트 번호 추가
        ax.text(start_x, start_y, f'{agent_id}', color='black', fontsize=8)
        ax.text(end_x, end_y, f'{agent_id}', color='black', fontsize=8)


def add_rect_obstacles(ax, yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    for obstacle_data in yaml_data['obstacles']:
        center = obstacle_data['center']
        width = obstacle_data['width']
        height = obstacle_data['height']
        rectangle = Rectangle((center[0] - width / 2, center[1] - height / 2), width, height, fc='black')
        ax.add_patch(rectangle)


def add_circ_obstacles(ax, yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    for obstacle_data in yaml_data['obstacles']:
        center = obstacle_data['center']
        radius = obstacle_data['radius']
        circle = Circle((center[0], center[1]), radius, fc='black')
        ax.add_patch(circle)


def init():
    time_text.set_text('')
    for agent_id, path in agents.items():
        if path:  # 경로가 있는 경우에만 초기화
            circle = circles[agent_id]
            circle.center = (0, 0)
            ax.text(0, 0, str(agent_id), color='black', fontsize=8, ha='center', va='center')
    return [time_text] + list(circles.values())


def animate(frame):
    time = frame / sample
    time_text.set_text(f'Time: {time:.2f}s')
    for agent_id, path in agents.items():
        if not path:
            agent_annotations[agent_id].set_visible(False)
            continue
        position = next(((x, y) for x, y, t in path if t >= time), path[-1][:2])
        circle = circles[agent_id]
        circle.center = position
        agent_annotations[agent_id].set_position(position)

    for agent_id, circle in circles.items():
        circle.set_facecolor('blue')
        for other_agent_id, other_circle in circles.items():
            if agent_id == other_agent_id:
                continue
            if np.linalg.norm(np.array(circle.center) - np.array(other_circle.center)) < radii[agent_id] * 0.85 + radii[
                other_agent_id] * 0.85:
                circle.set_facecolor('red')
                print(f'Collision between agents {agent_id} and {other_agent_id} at time {time:.2f}s')
    return [time_text] + list(circles.values()) + list(agent_annotations.values())


if __name__ == '__main__':
    sample = 10

    # get mapname obs robotnum testnum using argparse
    parser = argparse.ArgumentParser()

    parser.add_argument("--mapname", "-m", type=str)
    parser.add_argument("--obs", "-o", type=str)
    parser.add_argument("--robotnum", "-r", type=str)
    parser.add_argument("--testnum", "-t", type=str)

    args = parser.parse_args()

    mapname = args.mapname
    obs = args.obs
    robotnum = args.robotnum
    testnum = args.testnum

    benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum + "_" + testnum + ".yaml"
    solutionPath = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum + "_" + testnum + "_solution.txt"

    with open(solutionPath, 'r') as file:
        data = file.readlines()

    radii = [0.5 for _ in range(100)]

    agents, no_path_agents = parse_data(data)

    print(f'Agents without a path: {no_path_agents}')
    print(f'Number of agents: {len(agents)}')

    for agent_id, path in agents.items():
        agents[agent_id] = interpolate_path(path)

    total_time = max([path[-1][2] for path in agents.values() if path])

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 40)
    add_start_end_points(ax, agents)

    if mapname == "RectEnv":
        add_rect_obstacles(ax, benchmarkPath)
    else:
        add_circ_obstacles(ax, benchmarkPath)

    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    circles = {}
    for agent_id, path in agents.items():
        if path:
            radius = radii[agent_id] * 0.9
            circles[agent_id] = Circle((0, 0), radius, fc='blue')
        ax.add_patch(circles.get(agent_id, Circle((0, 0), 0)))

    agent_annotations = {}
    for agent_id in agents.keys():
        agent_annotations[agent_id] = ax.text(0, 0, str(agent_id), color='white', fontsize=8, ha='center', va='center',
                                              visible=True)

    ani = animation.FuncAnimation(fig, animate, frames=int(total_time * sample), init_func=init, blit=True, interval=1)
    plt.show()
