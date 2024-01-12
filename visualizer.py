import re

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.patches import Circle

sample = 10

# 파일에서 데이터 읽기
file_path = '/home/joonyeol/CLionProjects/SI-CBS/solution/CircleEnv_20/agents80/CircleEnv_20_80_1_solution.txt'
with open(file_path, 'r') as file:
    data = file.readlines()

# radii.txt 파일에서 반지름 데이터 읽기
# radii_file_path = 'radii.txt'
# with open(radii_file_path, 'r') as file:
#     radii_data = file.readlines()
# radii = [float(radius.strip()) for radius in radii_data]
radii = [0.5 for _ in range(100)]


# 데이터 파싱
def parse_data(data):
    agents = {}
    no_path_agents = []  # 경로가 없는 에이전트를 저장할 리스트
    for line in data:
        match = re.match(r'Agent (\d+):', line)
        if match:
            agent_id = int(match.group(1))
            points = re.findall(r'\(([^)]+)\)', line)
            if points:
                agents[agent_id] = [tuple(map(float, point.split(','))) for point in points]
            else:
                no_path_agents.append(agent_id)  # 경로가 없으면 리스트에 추가
    return agents, no_path_agents


agents, no_path_agents = parse_data(data)

# 경로가 없는 에이전트 출력
print(f'Agents without a path: {no_path_agents}')
print(f'Number of agents: {len(agents)}')


# 경로 보간
def interpolate_path(path):
    interpolated_path = []
    for i in range(len(path) - 1):
        x0, y0, t0 = path[i]
        x1, y1, t1 = path[i + 1]
        times = np.linspace(t0, t1, int((t1 - t0) * sample))  # 2배의 시간 해상도로 보간
        xs = np.linspace(x0, x1, len(times))
        ys = np.linspace(y0, y1, len(times))
        interpolated_path.extend(list(zip(xs, ys, times)))
    return interpolated_path


for agent_id, path in agents.items():
    agents[agent_id] = interpolate_path(path)

# 애니메이션 총 시간 계산
total_time = max([path[-1][2] for path in agents.values() if path])


def add_start_end_points(ax, agents):
    for agent_id, path in agents.items():
        # 시작점과 도착점 추가
        if not path:
            continue
        start_x, start_y, _ = path[0]
        end_x, end_y, _ = path[-1]
        ax.plot(start_x, start_y, 'go')  # 녹색은 시작점
        ax.plot(end_x, end_y, 'ro')  # 빨간색은 도착점

        # 에이전트 번호 추가
        ax.text(start_x, start_y, f'{agent_id}', color='black', fontsize=8)
        ax.text(end_x, end_y, f'{agent_id}', color='black', fontsize=8)


# 각 에이전트에 번호를 표시하는 코드 추가

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
    annotations = []
    for agent_id, path in agents.items():
        if not path:
            continue  # 경로가 없는 에이전트는 무시
        position = next(((x, y) for x, y, t in path if t >= time), path[-1][:2])
        circle = circles[agent_id]
        circle.center = position
        annotation = ax.text(position[0], position[1], str(agent_id), color='white', fontsize=8, ha='center',
                             va='center')
        annotations.append(annotation)
    # check if agents have collided

    for agent_id, circle in circles.items():
        circle.set_facecolor('blue')
        for other_agent_id, other_circle in circles.items():
            if agent_id == other_agent_id:
                continue
            if np.linalg.norm(np.array(circle.center) - np.array(other_circle.center)) < radii[agent_id] * 0.85 + radii[
                other_agent_id] * 0.85:
                circle.set_facecolor('red')
                print(f'Collision between agents {agent_id} and {other_agent_id} at time {time:.2f}s')
    return [time_text] + list(circles.values()) + annotations


# def add_obstacles(ax, yaml_file_path):
#     with open(yaml_file_path, 'r') as file:
#         yaml_data = yaml.safe_load(file)
#
#     for obstacle_data in yaml_data['obstacles']:
#         center = obstacle_data['center']
#         width = obstacle_data['width']
#         height = obstacle_data['height']
#         rectangle = Rectangle((center[0] - width / 2, center[1] - height / 2), width, height, fc='black')
#         ax.add_patch(rectangle)

def add_obstacles(ax, yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)

    for obstacle_data in yaml_data['obstacles']:
        center = obstacle_data['center']
        radius = obstacle_data['radius']
        circle = Circle((center[0], center[1]), radius, fc='black')
        ax.add_patch(circle)


# 기존 애니메이션 구성 코드는 그대로 유지
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(0, 40)
ax.set_ylim(0, 40)
add_start_end_points(ax, agents)  # 시작점과 도착점, 에이전트 번호 추가

yaml_file_path = "/home/joonyeol/CLionProjects/SI-CBS/benchmark/CircleEnv_20/agents80/CircleEnv_20_80_1.yaml"
add_obstacles(ax, yaml_file_path)

time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
circles = {}
for agent_id, path in agents.items():
    if path:
        radius = radii[agent_id] * 0.9  # 예를 들어 0.9를 곱하여 실제 크기를 조정할 수 있습니다.
        circles[agent_id] = Circle((0, 0), radius, fc='blue')
    ax.add_patch(circles.get(agent_id, Circle((0, 0), 0)))

ani = animation.FuncAnimation(fig, animate, frames=int(total_time * sample), init_func=init, blit=True, interval=1)

plt.show()
