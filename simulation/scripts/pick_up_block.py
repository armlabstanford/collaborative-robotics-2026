#!/usr/bin/env python3
"""
TidyBot2 Block Pickup Demo - Using Mink for Inverse Kinematics

Demonstrates picking up a block using the right arm with top-down grasping.
The block position is read from the simulation and used as the target.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path

import mink


def main():
    # ==========================================================================
    # Load Model
    # ==========================================================================
    script_dir = Path(__file__).parent
    model_path = script_dir / "../assets/mujoco/scene_pickup.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)

    # ==========================================================================
    # Get IDs
    # ==========================================================================
    # Block body
    block_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "block")

    # Right arm joints (for IK)
    right_arm_joints = [
        "right_waist",
        "right_shoulder",
        "right_elbow",
        "right_forearm_roll",
        "right_wrist_angle",
        "right_wrist_rotate",
    ]

    # Get joint qpos addresses
    joint_qpos_addrs = {}
    for jname in right_arm_joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        joint_qpos_addrs[jname] = model.jnt_qposadr[jid]

    # Right arm actuators
    right_arm_actuators = {}
    for jname in right_arm_joints:
        aname = jname  # actuator name matches joint name
        right_arm_actuators[jname] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)

    # Gripper
    right_gripper_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_fingers_actuator")

    # ==========================================================================
    # Initialize
    # ==========================================================================
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # Get block position from simulation
    block_pos = data.xpos[block_body_id].copy()
    print(f"Block position: {block_pos}")

    # ==========================================================================
    # Setup Mink IK
    # ==========================================================================
    configuration = mink.Configuration(model)
    configuration.update(data.qpos)

    # End-effector task for right arm
    ee_task = mink.FrameTask(
        frame_name="right_pinch_site",
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
    )

    # Top-down grasp orientation: gripper pointing down
    # Quaternion (wxyz) for 180 degree rotation around X-axis (gripper pointing down)
    # This rotates the gripper to point straight down for top-down grasping
    top_down_quat = np.array([0, 1, 0, 0])  # wxyz format: 180 deg around X

    # ==========================================================================
    # Define waypoints based on block position
    # ==========================================================================
    approach_pos = block_pos + np.array([0, 0, 0.15])   # 15cm above block
    grasp_pos = block_pos + np.array([0, 0, 0.06])      # At block top (block is 2.5cm half-height)
    lift_pos = block_pos + np.array([0, 0, 0.20])       # Lift to 20cm above original position

    print(f"Approach: {approach_pos}")
    print(f"Grasp: {grasp_pos}")
    print(f"Lift: {lift_pos}")

    # ==========================================================================
    # Run Simulation
    # ==========================================================================
    print("\nStarting pickup sequence...")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Camera setup
        viewer.cam.distance = 1.2
        viewer.cam.elevation = -30
        viewer.cam.azimuth = 150
        viewer.cam.lookat[:] = [0.4, -0.1, 0.2]

        # Show coordinate frames (body frames)
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY

        start_time = time.time()
        phase = "approach"

        # IK parameters
        dt = model.opt.timestep
        ik_steps_per_sim_step = 1

        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - start_time

            # ==================================================================
            # State Machine
            # ==================================================================
            if phase == "approach":
                target_pos = approach_pos
                if elapsed > 3.0:
                    phase = "descend"
                    print("Phase: descend")

            elif phase == "descend":
                # Interpolate from approach to grasp
                t = min((elapsed - 3.0) / 2.0, 1.0)
                target_pos = approach_pos + t * (grasp_pos - approach_pos)
                if elapsed > 5.0:
                    phase = "grasp"
                    print("Phase: grasp - closing gripper")

            elif phase == "grasp":
                target_pos = grasp_pos
                # Close gripper gradually
                gripper_progress = min((elapsed - 5.0) / 1.0, 1.0)
                data.ctrl[right_gripper_ctrl] = gripper_progress * 255
                if elapsed > 6.0:
                    phase = "lift"
                    print("Phase: lift")

            elif phase == "lift":
                # Interpolate from grasp to lift
                t = min((elapsed - 6.0) / 2.0, 1.0)
                target_pos = grasp_pos + t * (lift_pos - grasp_pos)
                data.ctrl[right_gripper_ctrl] = 255  # Keep closed
                if elapsed > 8.0:
                    phase = "done"
                    print("Phase: done - pickup complete!")

            else:  # done
                target_pos = lift_pos
                data.ctrl[right_gripper_ctrl] = 255

            # ==================================================================
            # Solve IK
            # ==================================================================
            # Update configuration with current qpos
            configuration.update(data.qpos)

            # Set target pose (position + top-down orientation)
            target_pose = mink.SE3.from_rotation_and_translation(
                rotation=mink.SO3(top_down_quat),
                translation=target_pos,
            )
            ee_task.set_target(target_pose)

            # Solve IK
            for _ in range(ik_steps_per_sim_step):
                vel = mink.solve_ik(
                    configuration,
                    [ee_task],
                    dt=dt,
                    solver="quadprog",
                    damping=1e-3,
                )
                configuration.integrate_inplace(vel, dt)

            # Apply solved joint positions to actuators
            for jname in right_arm_joints:
                qpos_addr = joint_qpos_addrs[jname]
                act_id = right_arm_actuators[jname]
                data.ctrl[act_id] = configuration.q[qpos_addr]

            # ==================================================================
            # Step Simulation
            # ==================================================================
            mujoco.mj_step(model, data)
            viewer.sync()

            # Real-time sync
            elapsed_step = time.time() - step_start
            sleep_time = dt - elapsed_step
            if sleep_time > 0:
                time.sleep(sleep_time)

    print("Simulation ended.")


if __name__ == "__main__":
    main()
