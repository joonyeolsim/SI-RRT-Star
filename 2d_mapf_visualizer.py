import matplotlib.animation as animation
import matplotlib.pyplot as plt

time_step = 500


def cbs_animate(input_data, path_data, interp_steps=time_step):
    # Calculate interpolated coordinates for smooth
    # er transitions
    x_coords_list_interp = []
    y_coords_list_interp = []
    times_interp = []

    for path in path_data:
        x_coords = [point[0][0] for point in path]
        y_coords = [point[0][1] for point in path]
        times = [point[1] for point in path]

        x_coords_new = []
        y_coords_new = []
        times_new = []
        for i in range(len(x_coords) - 1):
            for j in range(interp_steps):
                fraction = j / interp_steps
                x_coords_new.append(x_coords[i] + fraction * (x_coords[i + 1] - x_coords[i]))
                y_coords_new.append(y_coords[i] + fraction * (y_coords[i + 1] - y_coords[i]))
                times_new.append(times[i] + fraction * (times[i + 1] - times[i]))

        x_coords_list_interp.append(x_coords_new + [x_coords[-1]])
        y_coords_list_interp.append(y_coords_new + [y_coords[-1]])
        if len(times_new) > len(times_interp):
            times_interp = times_new[:] + [times[-1]]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, input_data["space_limits"][0])
    ax.set_ylim(0, input_data["space_limits"][1])
    # 추가된 원을 그리는 부분
    circle = plt.Circle((5, 5), 2, color='red', fill=False)  # 중심이 (5, 5)이고, 반지름이 2인 원
    ax.add_patch(circle)
    ax.grid(True, which="both", linestyle="-", linewidth=1)
    ax.set_xticks(range(input_data["space_limits"][0] + 1))
    ax.set_yticks(range(input_data["space_limits"][1] + 1))
    ax.set_aspect("equal")
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.tick_params(size=0)

    # Static obstacles
    for obstacle in input_data["static_obstacles"]:
        ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), 1, 1, facecolor="gray"))

    # Start and goal points
    for start_point, goal_point in zip(
            input_data["start_points"],
            input_data["goal_points"],
    ):
        ax.plot(
            start_point[0],
            start_point[1],
            "go",
            markersize=10,
        )
        ax.plot(
            goal_point[0],
            goal_point[1],
            "ro",
            markersize=10,
        )

    time_text = ax.text(0.05, 0.95, "", transform=ax.transAxes, color="red", fontsize=12)

    circles = []
    agent_texts = []
    for index, (
            x_coords,
            y_coords,
    ) in enumerate(zip(x_coords_list_interp, y_coords_list_interp)):
        circle = plt.Circle(
            (x_coords[0], y_coords[0]), 0.5, facecolor="blue"
        )
        circles.append(circle)
        agent_text = ax.text(
            x_coords[0],
            y_coords[0],
            str(index),
            ha="center",
            va="center",
            color="white",
        )
        agent_texts.append(agent_text)

    # Dynamic obstacles (initially set to out-of-bounds location)
    # dynamic_obstacles_rects = [
    #     plt.Rectangle((-10, -10), 1, 1, facecolor="orange") for _ in input_data["dynamic_obstacles"]
    # ]
    # for obstacle in dynamic_obstacles_rects:
    #     ax.add_patch(obstacle)

    def init():
        for circle in circles:
            ax.add_patch(circle)
        time_text.set_text("")
        # for obstacle in dynamic_obstacles_rects:
        #     obstacle.set_xy((-10, -10))
        for agent_text in agent_texts:
            agent_text.set_visible(True)
        return [*circles, time_text, *agent_texts]

    def animate(i):
        positions = []  # 로봇들의 현재 위치를 저장할 리스트

        for index, (x_coords, y_coords, circle, agent_text) in enumerate(
                zip(x_coords_list_interp, y_coords_list_interp, circles, agent_texts)
        ):
            if i < len(x_coords):
                new_position = (x_coords[i], y_coords[i])
                circle.set_center(new_position)
                agent_text.set_position(new_position)
                positions.append((index, new_position))  # 로봇의 인덱스와 위치 추가

        if i < len(times_interp):
            time_text.set_text(f"Time: {times_interp[i]}")

        # 충돌 감지
        for idx1, pos1 in positions:
            for idx2, pos2 in positions:
                if idx1 != idx2 and pos1 == pos2:
                    print(
                        f"Collision detected between robot {idx1} and robot {idx2} at time {times_interp[i]}"
                    )

        return [*circles, time_text, *agent_texts]

    anim = animation.FuncAnimation(
        fig,
        animate,
        init_func=init,
        frames=len(times_interp),
        interval=time_step / interp_steps,  # Adjusted interval for smoother transitions
        repeat=False,
        blit=True,
    )
    plt.show()


def parse_grid(file_path):
    with open(file_path, "r") as f:
        lines = f.readlines()

    height = len(lines) - 4
    static_obstacles = []
    for y, line in enumerate(lines[4:]):  # Skipping the header lines
        for x, char in enumerate(line.strip()):
            if char == "@" or char == "T":
                static_obstacles.append((x, height - 1 - y))  # Invert y-coordinate

    space_limits = (len(lines[4].strip()), height)
    start_points = []
    goal_points = []

    input_data = {
        "space_limits": space_limits,
        "static_obstacles": static_obstacles,
        "start_points": start_points,
        "goal_points": goal_points,
    }
    return input_data


def parse_paths(file_path, height):
    with open(file_path, "r") as f:
        lines = f.readlines()

    path_data = []
    for line in lines:
        agent_path = []
        coords = line.strip().split(":")[1].split("->")
        for t, coord in enumerate(coords):
            if coord:
                x, y, time = map(float, coord.strip("()").split(","))
                agent_path.append(((y, height - 1 - x), t))  # Invert y-coordinate
        path_data.append(agent_path)

    return path_data


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", "-i", type=str, help="Input file path")
    parser.add_argument("--output", "-o", type=str, help="Output file path")
    args = parser.parse_args()

    input_data = parse_grid(args.input)
    path_data = parse_paths(args.output, input_data["space_limits"][1])
    for path in path_data:
        input_data["start_points"].append(path[0][0])
        input_data["goal_points"].append(path[-1][0])

    cbs_animate(input_data, path_data)
