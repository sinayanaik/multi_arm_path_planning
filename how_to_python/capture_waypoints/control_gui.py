import csv
import inspect
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk, filedialog, messagebox

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


MESHCAT_SUPPORTS_SEPARATE_ROBOTS = "root_node_name" in inspect.signature(MeshcatVisualizer.loadViewerModel).parameters


@dataclass
class RobotConfig:
    name: str
    urdf_path: str
    package_dir: str
    base_xyz_meters: tuple
    home_degrees: list
    end_effector_frame: str


ROBOT_1 = RobotConfig(
    name="Robot 1",
    urdf_path="/home/tcs-research/Documents/multi_arm_path_planning/how_to_python/universal_robots/ur_description/urdf/ur5.urdf",
    package_dir="/home/tcs-research/Documents/multi_arm_path_planning/how_to_python/universal_robots",
    base_xyz_meters=(0.0, 0.0, 0.0),
    home_degrees=[0, -90, -90, 90, 90, 0],
    end_effector_frame="tool0",
)

ROBOT_2 = RobotConfig(
    name="Robot 2",
    urdf_path="/home/tcs-research/Documents/multi_arm_path_planning/how_to_python/other_robot/other_description/urdf/other.urdf",
    package_dir="/home/tcs-research/Documents/multi_arm_path_planning/how_to_python/other_robot",
    base_xyz_meters=(1.5, 0.0, 0.0),
    home_degrees=[0, -90, -90, 90, 90, 0],
    end_effector_frame="tool0",
)


def rotation_matrix_to_rpy(rotation):
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def load_robot_in_meshcat(robot_config, shared_viewer=None, open_browser=False):
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        robot_config.urdf_path, package_dirs=robot_config.package_dir)
    data = model.createData()
    visualizer = MeshcatVisualizer(model, collision_model, visual_model)

    scene_name = robot_config.name.lower().replace(" ", "_")
    if MESHCAT_SUPPORTS_SEPARATE_ROBOTS:
        visualizer.initViewer(viewer=shared_viewer, open=open_browser)
        visualizer.loadViewerModel(root_node_name=scene_name)
    else:
        visualizer.initViewer(open=open_browser)
        visualizer.loadViewerModel()
        if shared_viewer is not None:
            print(f"pinocchio {pin.__version__} cannot show two robots in one MeshCat scene. "
                  f"{robot_config.name} got its own browser tab. "
                  "Run 'pip install --upgrade pin' (pinocchio 3.x) to see both robots side by side.")

    if any(robot_config.base_xyz_meters):
        node_name = scene_name if MESHCAT_SUPPORTS_SEPARATE_ROBOTS else "pinocchio"
        base_transform = np.eye(4)
        base_transform[:3, 3] = robot_config.base_xyz_meters
        try:
            visualizer.viewer[node_name].set_transform(base_transform)
        except (KeyError, TypeError):
            print(f"{robot_config.name} could not be moved to {robot_config.base_xyz_meters}, stays at origin.")

    return model, data, visualizer


class RobotPanel(ttk.LabelFrame):
    def __init__(self, parent, robot_config, model, data, visualizer, on_robot_moved):
        super().__init__(parent, text=robot_config.name, padding=10)
        self.robot_config = robot_config
        self.model = model
        self.data = data
        self.visualizer = visualizer
        self.on_robot_moved = on_robot_moved

        self.end_effector_frame_id = model.getFrameId(robot_config.end_effector_frame)

        lower_radians = np.array(model.lowerPositionLimit, dtype=float)
        upper_radians = np.array(model.upperPositionLimit, dtype=float)
        lower_radians[~np.isfinite(lower_radians)] = -np.pi
        upper_radians[~np.isfinite(upper_radians)] = np.pi
        self.lower_degrees = np.rad2deg(lower_radians)
        self.upper_degrees = np.rad2deg(upper_radians)

        home_radians = np.deg2rad(np.array(robot_config.home_degrees, dtype=float))
        self.home_configuration = home_radians if home_radians.size == model.nq else pin.neutral(model)

        self._updating_widgets = False
        self.slider_variables = []
        self.entry_variables = []
        self._build_joint_widgets()
        self._build_end_effector_widgets()
        self.move_to_home()

    def _build_joint_widgets(self):
        joint_grid = ttk.Frame(self)
        joint_grid.grid(row=0, column=0, sticky="nsew")
        for joint_index in range(self.model.nq):
            slider_variable = tk.DoubleVar(value=0.0)
            entry_variable = tk.StringVar(value="0.0")
            slider_variable.trace_add("write", self.on_slider_changed)
            entry_variable.trace_add(
                "write",
                lambda *_, entry=entry_variable, slider=slider_variable, index=joint_index:
                    self.on_entry_changed(entry, slider, index))
            self.slider_variables.append(slider_variable)
            self.entry_variables.append(entry_variable)

            ttk.Label(joint_grid, text=f"J{joint_index + 1}", width=4).grid(row=joint_index, column=0, sticky="w")
            ttk.Scale(joint_grid, from_=self.lower_degrees[joint_index], to=self.upper_degrees[joint_index],
                      variable=slider_variable, orient="horizontal"
                      ).grid(row=joint_index, column=1, sticky="ew", padx=6, pady=2)
            entry = ttk.Entry(joint_grid, textvariable=entry_variable, width=8, justify="right")
            entry.grid(row=joint_index, column=2, padx=(6, 0))
            entry.bind("<Return>",
                       lambda _, entry=entry_variable, slider=slider_variable: self.normalize_entry_text(entry, slider))
            entry.bind("<FocusOut>",
                       lambda _, entry=entry_variable, slider=slider_variable: self.normalize_entry_text(entry, slider))
        joint_grid.columnconfigure(1, weight=1)
        ttk.Button(self, text="Home", command=self.move_to_home).grid(row=1, column=0, sticky="w", pady=(8, 0))

    def _build_end_effector_widgets(self):
        end_effector_frame_name = self.model.frames[self.end_effector_frame_id].name
        pose_frame = ttk.LabelFrame(self, text=f"End effector: {end_effector_frame_name}   x y z [m]   roll pitch yaw [deg]",
                                    padding=8)
        pose_frame.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self.end_effector_labels = {}
        for label, row, column in [("X", 0, 0), ("Y", 0, 2), ("Z", 0, 4),
                                   ("Roll", 1, 0), ("Pitch", 1, 2), ("Yaw", 1, 4)]:
            ttk.Label(pose_frame, text=f"{label}:").grid(row=row, column=column, sticky="e", padx=(8, 2), pady=2)
            value_label = ttk.Label(pose_frame, text="0.000", width=10, anchor="e", relief="sunken")
            value_label.grid(row=row, column=column + 1, sticky="ew")
            self.end_effector_labels[label] = value_label

    def forward_kinematics(self, joint_angles_radians):
        pin.forwardKinematics(self.model, self.data, joint_angles_radians)
        pin.updateFramePlacements(self.model, self.data)
        frame_placement = self.data.oMf[self.end_effector_frame_id]
        position = frame_placement.translation.copy()
        roll_pitch_yaw = rotation_matrix_to_rpy(frame_placement.rotation)
        return position, roll_pitch_yaw

    def get_joint_angles_radians(self):
        return np.deg2rad([variable.get() for variable in self.slider_variables])

    def get_end_effector_pose(self):
        position, roll_pitch_yaw = self.forward_kinematics(self.get_joint_angles_radians())
        return np.concatenate([position, roll_pitch_yaw])

    def set_joint_angles_radians(self, joint_angles_radians):
        self._updating_widgets = True
        for slider_variable, angle in zip(self.slider_variables, np.rad2deg(joint_angles_radians)):
            slider_variable.set(float(angle))
        self._updating_widgets = False
        self.refresh()

    def move_to_home(self):
        self.set_joint_angles_radians(self.home_configuration)

    def refresh(self):
        joint_angles_radians = self.get_joint_angles_radians()
        self.visualizer.display(joint_angles_radians)
        position, roll_pitch_yaw = self.forward_kinematics(joint_angles_radians)
        displayed_values = {
            "X": position[0], "Y": position[1], "Z": position[2],
            "Roll": np.rad2deg(roll_pitch_yaw[0]),
            "Pitch": np.rad2deg(roll_pitch_yaw[1]),
            "Yaw": np.rad2deg(roll_pitch_yaw[2]),
        }
        for label, value in displayed_values.items():
            self.end_effector_labels[label].config(text=f"{value:.4f}")
        self.on_robot_moved()

    def on_slider_changed(self, *_):
        if self._updating_widgets:
            return
        self._updating_widgets = True
        for slider_variable, entry_variable in zip(self.slider_variables, self.entry_variables):
            entry_variable.set(f"{slider_variable.get():.1f}")
        self._updating_widgets = False
        self.refresh()

    def on_entry_changed(self, entry_variable, slider_variable, joint_index):
        if self._updating_widgets:
            return
        try:
            typed_value = float(entry_variable.get())
        except ValueError:
            return
        clamped_value = float(np.clip(typed_value, self.lower_degrees[joint_index], self.upper_degrees[joint_index]))
        self._updating_widgets = True
        if abs(clamped_value - typed_value) > 1e-9:
            entry_variable.set(f"{clamped_value:.1f}")
        slider_variable.set(clamped_value)
        self._updating_widgets = False
        self.refresh()

    def normalize_entry_text(self, entry_variable, slider_variable):
        entry_variable.set(f"{slider_variable.get():.1f}")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dual-Robot Pinocchio + MeshCat")

        self.robot1_model, self.robot1_data, self.robot1_visualizer = load_robot_in_meshcat(ROBOT_1, open_browser=True)
        self.robot2_model, self.robot2_data, self.robot2_visualizer = load_robot_in_meshcat(
            ROBOT_2, shared_viewer=self.robot1_visualizer.viewer, open_browser=True)
        for robot_config, robot_model in [(ROBOT_1, self.robot1_model), (ROBOT_2, self.robot2_model)]:
            print(f"{robot_config.name}: joints {[name for name in robot_model.names[1:]]}")

        self.is_recording = False
        self.samples = []
        self.recording_start_time = 0.0
        self.playback_samples = []
        self.playback_index = 0
        self.is_playing = False

        self.robot1_panel = RobotPanel(self, ROBOT_1, self.robot1_model, self.robot1_data,
                                       self.robot1_visualizer, on_robot_moved=self.on_robot_moved)
        self.robot1_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.robot2_panel = RobotPanel(self, ROBOT_2, self.robot2_model, self.robot2_data,
                                       self.robot2_visualizer, on_robot_moved=self.on_robot_moved)
        self.robot2_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self._build_toolbar()
        self._build_status_bar()
        self.protocol("WM_DELETE_WINDOW", self.close_application)

    def _build_toolbar(self):
        toolbar = ttk.Frame(self)
        toolbar.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10)
        ttk.Label(toolbar, text="File prefix:").grid(row=0, column=0)
        self.file_prefix_variable = tk.StringVar(value="motion")
        ttk.Entry(toolbar, textvariable=self.file_prefix_variable, width=10).grid(row=0, column=1, padx=(2, 10))
        self.record_button = ttk.Button(toolbar, text="Record", command=self.toggle_recording)
        self.record_button.grid(row=0, column=2, padx=3)
        ttk.Button(toolbar, text="Save CSVs", command=self.save_recordings).grid(row=0, column=3, padx=3)
        ttk.Button(toolbar, text="Load CSVs", command=self.load_recordings).grid(row=0, column=4, padx=3)
        ttk.Button(toolbar, text="Play", command=self.play).grid(row=0, column=5, padx=3)
        ttk.Button(toolbar, text="Pause", command=self.pause_or_resume).grid(row=0, column=6, padx=3)
        ttk.Button(toolbar, text="Stop", command=self.stop_playback).grid(row=0, column=7, padx=3)
        ttk.Label(toolbar, text="Speed:").grid(row=0, column=8, padx=(10, 2))
        self.playback_speed_variable = tk.StringVar(value="1.0")
        ttk.Entry(toolbar, textvariable=self.playback_speed_variable, width=5).grid(row=0, column=9)
        self.loop_playback = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar, text="Loop", variable=self.loop_playback).grid(row=0, column=10, padx=10)

    def _build_status_bar(self):
        self.status_text = tk.StringVar(value="Ready")
        ttk.Label(self, textvariable=self.status_text, relief="sunken", anchor="w"
                  ).grid(row=2, column=0, columnspan=2, sticky="ew", padx=10, pady=(8, 10))

    def on_robot_moved(self):
        if self.is_recording and not self.is_playing:
            self.record_current_sample()

    def toggle_recording(self):
        if self.is_recording:
            self.is_recording = False
            self.record_button.config(text="Record")
            self.status_text.set(f"Recording stopped, {len(self.samples)} samples. Use 'Save CSVs' to keep them.")
        else:
            self.samples = []
            self.recording_start_time = time.perf_counter()
            self.is_recording = True
            self.record_button.config(text="Stop recording")
            self.status_text.set("Recording: every joint change is now saved as a sample.")
            self.record_current_sample()

    def record_current_sample(self):
        elapsed_seconds = time.perf_counter() - self.recording_start_time
        self.samples.append({
            "time": elapsed_seconds,
            "robot1_joints": self.robot1_panel.get_joint_angles_radians(),
            "robot1_end_effector": self.robot1_panel.get_end_effector_pose(),
            "robot2_joints": self.robot2_panel.get_joint_angles_radians(),
            "robot2_end_effector": self.robot2_panel.get_end_effector_pose(),
        })
        self.status_text.set(f"Recording... {len(self.samples)} samples")

    def save_recordings(self):
        if not self.samples:
            messagebox.showinfo("Nothing recorded", "Record some motion first.")
            return
        prefix = self.file_prefix_variable.get().strip() or "motion"
        self.write_csv(f"{prefix}_robot1_joints.csv",
                       ["time_s"] + [f"joint{i + 1}_rad" for i in range(self.robot1_model.nq)],
                       [[f"{sample['time']:.4f}"] + [f"{angle:.6f}" for angle in sample["robot1_joints"]]
                        for sample in self.samples])
        self.write_csv(f"{prefix}_robot2_joints.csv",
                       ["time_s"] + [f"joint{i + 1}_rad" for i in range(self.robot2_model.nq)],
                       [[f"{sample['time']:.4f}"] + [f"{angle:.6f}" for angle in sample["robot2_joints"]]
                        for sample in self.samples])
        end_effector_header = ["time_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"]
        for robot_number, panel in [(1, self.robot1_panel), (2, self.robot2_panel)]:
            key = f"robot{robot_number}_end_effector"
            self.write_csv(f"{prefix}_robot{robot_number}_end_effector.csv", end_effector_header,
                           [[f"{sample['time']:.4f}"] + [f"{value:.6f}" for value in sample[key]]
                            for sample in self.samples])
        self.status_text.set(f"Saved 4 CSV files as '{prefix}_*', {len(self.samples)} samples each")

    def write_csv(self, path, header, rows):
        with open(path, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerows(rows)

    def load_recordings(self):
        robot1_path = filedialog.askopenfilename(title="Select robot 1 joints CSV", filetypes=[("CSV files", "*.csv")])
        if not robot1_path:
            return
        robot2_path = filedialog.askopenfilename(title="Select robot 2 joints CSV", filetypes=[("CSV files", "*.csv")])
        if not robot2_path:
            return
        robot1_rows = read_csv_rows(robot1_path)
        robot2_rows = read_csv_rows(robot2_path)
        if len(robot1_rows) != len(robot2_rows):
            self.status_text.set(f"Row count mismatch ({len(robot1_rows)} vs {len(robot2_rows)}), using the shorter one")
        self.playback_samples = [
            {"time": row1[0], "robot1_joints": np.array(row1[1:]), "robot2_joints": np.array(row2[1:])}
            for row1, row2 in zip(robot1_rows, robot2_rows)
        ]
        self.playback_index = 0
        self.status_text.set(f"Loaded {len(self.playback_samples)} samples. Press Play.")

    def play(self):
        if self.is_playing:
            return
        if not self.playback_samples:
            messagebox.showinfo("No motion loaded", "Load joint CSV files first.")
            return
        if self.playback_index >= len(self.playback_samples):
            self.playback_index = 0
        self.is_playing = True
        self.play_next_sample()

    def play_next_sample(self):
        if not self.is_playing:
            return
        current_sample = self.playback_samples[self.playback_index]
        self.robot1_panel.set_joint_angles_radians(current_sample["robot1_joints"])
        self.robot2_panel.set_joint_angles_radians(current_sample["robot2_joints"])
        self.playback_index += 1
        if self.playback_index >= len(self.playback_samples):
            if self.loop_playback.get():
                self.playback_index = 0
                seconds_until_next = 0.05
            else:
                self.is_playing = False
                self.status_text.set("Playback finished.")
                return
        else:
            seconds_until_next = self.playback_samples[self.playback_index]["time"] - current_sample["time"]
        delay_milliseconds = max(1, int(1000 * seconds_until_next / self.playback_speed()))
        self.after(delay_milliseconds, self.play_next_sample)

    def pause_or_resume(self):
        if self.is_playing:
            self.is_playing = False
            self.status_text.set("Paused.")
        elif self.playback_samples and self.playback_index > 0:
            self.is_playing = True
            self.play_next_sample()

    def stop_playback(self):
        self.is_playing = False
        self.playback_index = 0
        self.status_text.set("Playback stopped.")

    def playback_speed(self):
        try:
            return max(float(self.playback_speed_variable.get()), 0.01)
        except ValueError:
            return 1.0

    def close_application(self):
        self.is_recording = False
        self.is_playing = False
        self.destroy()


def read_csv_rows(path):
    with open(path) as file:
        reader = csv.reader(file)
        next(reader, None)
        return [[float(value) for value in row] for row in reader if row]


if __name__ == "__main__":
    App().mainloop()