import math
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Robot parameters ---
L1 = 1.2
L2 = 1.0

# Target (choose something obviously reachable)
# TARGET_X = 1.4
# TARGET_Y = 0.6

# Generate a random reachable target point
# Distance (radius) allowed: between |L1 - L2| and (L1 + L2)
r_min = abs(L1 - L2) + 0.1
r_max = (L1 + L2) - 0.1

# Choose random radius + angle
radius = random.uniform(r_min, r_max)
angle = random.uniform(-math.pi, math.pi)

TARGET_X = radius * math.cos(angle)
TARGET_Y = radius * math.sin(angle)

STEPS = 400
DT = 0.05

# --- IK function for 2-link planar arm ---
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

    # numerical safety
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.acos(cos_theta2)

    if not elbow_up:
        theta2 = -theta2

    # Compute theta1
    phi = math.atan2(y, x)
    # Avoid division by zero if L1 + L2*cos(theta2) is tiny
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    psi = math.atan2(k2, k1)

    theta1 = phi - psi

    return theta1, theta2


# --- Forward kinematics (reuse from FK sim) ---
def forward_kinematics(theta1, theta2):
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x1, y1), (x2, y2)


# --- Matplotlib setup ---
fig, ax = plt.subplots()
ax.set_aspect('equal', 'box')
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_title("2-Link Robot Arm – Inverse Kinematics Reach")


# Plot elements
base_plot, = ax.plot([], [], 'ko', markersize=8)
elbow_plot, = ax.plot([], [], 'bo', markersize=6)
end_plot, = ax.plot([], [], 'ro', markersize=6)
link1_line, = ax.plot([], [], 'b-', linewidth=3)
link2_line, = ax.plot([], [], 'r-', linewidth=3)
target_plot, = ax.plot([TARGET_X], [TARGET_Y], 'gx', markersize=8, label='Target')

# Keep current joint angles as state
theta1_curr = 0.0
theta2_curr = 0.0

# Try to compute IK for the chosen target
ik_solution = inverse_kinematics_2link(TARGET_X, TARGET_Y, L1, L2, elbow_up=True)
if ik_solution is None:
    print("Target is unreachable for this arm. Choose a different TARGET_X, TARGET_Y.")
    theta1_target, theta2_target = 0.0, 0.0
else:
    theta1_target, theta2_target = ik_solution
    print("IK solution (rad):", theta1_target, theta2_target)
    print("IK solution (deg):", math.degrees(theta1_target), math.degrees(theta2_target))


def update(i):
    global theta1_curr, theta2_curr

    # Simple "joint space" interpolation towards IK target
    alpha = 0.05  # step size (0 < alpha <= 1)
    theta1_curr += alpha * (theta1_target - theta1_curr)
    theta2_curr += alpha * (theta2_target - theta2_curr)

    (x1, y1), (x2, y2) = forward_kinematics(theta1_curr, theta2_curr)

    # Update plots
    base_plot.set_data([0], [0])
    elbow_plot.set_data([x1], [y1])
    end_plot.set_data([x2], [y2])

    link1_line.set_data([0, x1], [0, y1])
    link2_line.set_data([x1, x2], [y1, y2])

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
        anim.save("assets/sim_ik_reach_dynamic_target.gif", writer=gif_writer, dpi=80)
        print("Saved GIF to assets/sim_ik_reach_dynamic_target.gif")
    else:
        plt.show()
