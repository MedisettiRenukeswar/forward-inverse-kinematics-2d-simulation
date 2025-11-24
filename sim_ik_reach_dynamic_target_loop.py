import math
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Robot parameters ---
L1 = 1.2
L2 = 1.0

STEPS = 800
DT = 0.05

# How often to change target (in frames)
TARGET_CHANGE_INTERVAL = 120  # every 120 frames


def inverse_kinematics_2link(x, y, L1, L2, elbow_up=True):
    """
    Solve IK for a 2-link planar arm.
    Returns (theta1, theta2) in radians, or None if unreachable.
    """
    r2 = x * x + y * y
    r = math.sqrt(r2)

    # Reachability check
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None  # unreachable

    # cos(theta2) using law of cosines
    cos_theta2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # numerical safety

    theta2 = math.acos(cos_theta2)
    if not elbow_up:
        theta2 = -theta2

    # Compute theta1
    phi = math.atan2(y, x)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    psi = math.atan2(k2, k1)

    theta1 = phi - psi
    return theta1, theta2


def forward_kinematics(theta1, theta2):
    """Compute elbow and end-effector position using FK."""
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x1, y1), (x2, y2)


def random_reachable_target():
    """
    Sample a random target inside the reachable workspace.
    We use radius in [|L1-L2|+margin, (L1+L2)-margin] and random angle.
    """
    margin = 0.1
    r_min = abs(L1 - L2) + margin
    r_max = (L1 + L2) - margin

    radius = random.uniform(r_min, r_max)
    angle = random.uniform(-math.pi, math.pi)

    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    return x, y


# --- Matplotlib setup ---
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_title("2-Link Robot Arm – Moving Target IK")

# Plot elements
base_plot, = ax.plot([], [], 'ko', markersize=8)
elbow_plot, = ax.plot([], [], 'bo', markersize=6)
end_plot, = ax.plot([], [], 'ro', markersize=6)
link1_line, = ax.plot([], [], 'b-', linewidth=3)
link2_line, = ax.plot([], [], 'r-', linewidth=3)
target_plot, = ax.plot([], [], 'gx', markersize=8, label='Target')

# State variables
theta1_curr = 0.0
theta2_curr = 0.0
theta1_target = 0.0
theta2_target = 0.0
target_x, target_y = 0.0, 0.0


def choose_new_target():
    """
    Pick a new random reachable target and update global target angles.
    """
    global target_x, target_y, theta1_target, theta2_target

    # Try a few times in case of numerical edge cases
    for _ in range(10):
        tx, ty = random_reachable_target()
        ik_sol = inverse_kinematics_2link(tx, ty, L1, L2, elbow_up=True)
        if ik_sol is not None:
            theta1_target, theta2_target = ik_sol
            target_x, target_y = tx, ty
            print(f"New target: ({tx:.3f}, {ty:.3f}) "
                  f"→ IK (deg)=({math.degrees(theta1_target):.1f}, {math.degrees(theta2_target):.1f})")
            return

    # Fallback if something weird happens
    print("WARNING: Could not find a valid random target, keeping old one.")


def init():
    """Initialization for FuncAnimation."""
    choose_new_target()
    base_plot.set_data([0], [0])
    target_plot.set_data([target_x], [target_y])
    return base_plot, elbow_plot, end_plot, link1_line, link2_line, target_plot


def update(frame_idx):
    global theta1_curr, theta2_curr

    # Change target periodically
    if frame_idx % TARGET_CHANGE_INTERVAL == 0 and frame_idx != 0:
        choose_new_target()

    # Smooth joint-space interpolation towards current target
    alpha = 0.08  # step size in joint space
    theta1_curr += alpha * (theta1_target - theta1_curr)
    theta2_curr += alpha * (theta2_target - theta2_curr)

    (x1, y1), (x2, y2) = forward_kinematics(theta1_curr, theta2_curr)

    # Update plots
    base_plot.set_data([0], [0])
    elbow_plot.set_data([x1], [y1])
    end_plot.set_data([x2], [y2])

    link1_line.set_data([0, x1], [0, y1])
    link2_line.set_data([x1, x2], [y1, y2])

    target_plot.set_data([target_x], [target_y])

    return base_plot, elbow_plot, end_plot, link1_line, link2_line, target_plot


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
        anim.save("assets/sim_ik_reach_dynamic_target_loop.gif", writer=gif_writer, dpi=80)
        print("Saved GIF to assets/sim_ik_reach_dynamic_target_loop.gif")
    else:
        plt.show()