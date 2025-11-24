import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Link lengths
L1 = 1.2
L2 = 1.0

# Animation parameters
STEPS = 300
DT = 0.05

fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_title("2-Link Robot Arm – Forward Kinematics (FK)")


def forward_kinematics(theta1, theta2):
    """
    Compute elbow and end-effector position using FK.
    """
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x1, y1), (x2, y2)


# Plot elements
base_plot, = ax.plot([], [], 'ko', markersize=8)
elbow_plot, = ax.plot([], [], 'bo', markersize=6)
end_plot, = ax.plot([], [], 'ro', markersize=6)
link1_line, = ax.plot([], [], 'b-', linewidth=3)
link2_line, = ax.plot([], [], 'r-', linewidth=3)


def update(i):
    # Simple oscillating joint motion
    theta1 = 1.0 * math.sin(i * DT)
    theta2 = 1.2 * math.cos(i * DT * 0.7)

    (x1, y1), (x2, y2) = forward_kinematics(theta1, theta2)

    # Plot points
    base_plot.set_data([0], [0])
    elbow_plot.set_data([x1], [y1])
    end_plot.set_data([x2], [y2])

    # Plot links
    link1_line.set_data([0, x1], [0, y1])
    link2_line.set_data([x1, x2], [y1, y2])

    return base_plot, elbow_plot, end_plot, link1_line, link2_line


if __name__ == "__main__":
    ax.legend(loc='upper left')
    anim = FuncAnimation(fig, update, frames=150, interval=50, blit=True)

    SAVE_GIF = True  # set to False when you just want to view

    if SAVE_GIF:
        import os
        from matplotlib.animation import PillowWriter

        os.makedirs("assets", exist_ok=True)

        print("Saving GIF... this may take a few seconds.")
        gif_writer = PillowWriter(fps=10)  # lower fps = fewer frames in final GIF
        anim.save("assets/sim_fk.gif", writer=gif_writer, dpi=80)
        print("Saved GIF to assets/sim_fk.gif")
    else:
        plt.show()
