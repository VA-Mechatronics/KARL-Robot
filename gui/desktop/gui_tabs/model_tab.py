import logging
from pathlib import Path
import PySide6.QtWidgets as qtw
from PySide6.QtGui import QVector3D
import pyqtgraph.opengl as gl
import numpy as np
from scipy.spatial.transform import Rotation as R
import stl  # numpy-stl
import trimesh

from gui.desktop.gui_core.control_fields import ModelPageCommands
from client.client_api import RobotAPI
from common.target import Target

BASE_DIR = Path(__file__).resolve().parents[3]


class ModelTab(qtw.QWidget):
    def __init__(self, robot_api: RobotAPI):
        super().__init__()
        self.log = logging.getLogger(__name__)
        self.robot = robot_api

        layout = qtw.QVBoxLayout(self)
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=1000, elevation=20, azimuth=45)
        self.view.opts['center'] = QVector3D(0, 0, 300)
        self.view.setBackgroundColor([60, 60, 60])

        grid = gl.GLGridItem()
        grid.setSize(x=2000, y=2000, z=0)
        grid.setSpacing(x=100, y=100, z=0)
        self.view.addItem(grid)
        layout.addWidget(self.view)

        # UI control bar for bounding box toggle
        opt_layout = qtw.QHBoxLayout()
        self.chk_show_bbox = qtw.QCheckBox("Show Bounding Boxes")
        self.chk_show_bbox.toggled.connect(self._toggle_bbox_visibility)
        opt_layout.addWidget(self.chk_show_bbox)
        opt_layout.addStretch()
        layout.addLayout(opt_layout)

        self.model_page_commands = ModelPageCommands(self.robot)
        layout.addLayout(self.model_page_commands.layout)

        self.base_frame = self._create_frame()
        [self.view.addItem(axis) for axis in self.base_frame]

        # Trimesh OBB bounding boxes for native C-based collision checking
        self.trimesh_obbs: list[trimesh.Trimesh] = []
        self.act_bboxes = []

        # Load act & set links
        self.act_links = self._load_links(color=(0.3, 0.6, 1.0, 1), bbox_list=self.act_bboxes)
        [self.view.addItem(item) for item in self.act_links + self.act_bboxes]
        self.act_tcp_frame = self._create_frame()
        [self.view.addItem(axis) for axis in self.act_tcp_frame]

        self.set_links = self._load_links(color=(0.8, 0.0, 0.0, 0.3))
        [self.view.addItem(link) for link in self.set_links]
        self.set_tcp_frame = self._create_frame()
        [self.view.addItem(axis) for axis in self.set_tcp_frame]

        # Explicit pairs to ignore during collision checks
        self.ignored_pairs = {(0, 2), (2, 4), (6, 8)}

        self.log.info("Model_Tab loaded")
        self.robot.on("target", self._on_target_from_api)

    def _toggle_bbox_visibility(self, visible: bool):
        for bbox in self.act_bboxes:
            bbox.setVisible(visible)

    def _on_target_from_api(self, payload):
        target = Target.from_dict(payload)
        transformations = self.robot.kinematics.joints_to_transformations(target.joints)
        self._update_robot(transformations, target.joints[6], self.act_links, self.act_tcp_frame, self.act_bboxes)

    def update_set_positions(self, target: Target):
        transformations = self.robot.kinematics.joints_to_transformations(target.joints)
        self._update_robot(transformations, target.joints[6], self.set_links, self.set_tcp_frame)

    def _update_robot(self, transformations, gripper_val, links, tcp, bboxes=None):
        if not links:
            return

        applied_transforms = list(transformations[:7])

        # Compute gripper transforms for both brackets
        half_opening = gripper_val / 2.0
        T_b1 = np.eye(4); T_b1[0, 3] = half_opening
        T_b2 = np.array([[-1, 0, 0, -half_opening], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        applied_transforms.append(transformations[6] @ T_b1)
        applied_transforms.append(transformations[6] @ T_b2)

        # Apply transformations to STL meshes
        for link, T in zip(links, applied_transforms):
            self._homogeneous_transform(link, T)

        # Apply transformations to bounding box items including OBB offset
        if bboxes:
            for bbox_item, T in zip(bboxes, applied_transforms):
                # Multiply robot transformation T with the local OBB offset transform
                T_total = T @ bbox_item.obb_offset
                self._homogeneous_transform(bbox_item, T_total)

        # Transform TCP coordinate frame
        for axis_item in tcp:
            self._homogeneous_transform(axis_item, transformations[7])

        # Perform collision check and update bounding box colors
        if bboxes:
            collided_indices = self._check_collisions(applied_transforms)
            for idx, bbox in enumerate(bboxes):
                color = (1, 0, 0, 1.0) if idx in collided_indices else (0, 1, 0, 0.8)
                bbox.setData(color=color)

    def _check_collisions(self, transforms: list[np.ndarray]) -> set[int]:
        """Utilizes trimesh CollisionManager for fast OBB-SAT collision queries."""
        manager = trimesh.collision.CollisionManager()

        for idx, (obb, T) in enumerate(zip(self.trimesh_obbs, transforms)):
            manager.add_object(str(idx), obb, transform=T)

        collided_indices = set()
        n = len(transforms)

        for i in range(n):
            for j in range(i + 1, n):
                if (i, j) in self.ignored_pairs or (j, i) in self.ignored_pairs or j == i + 1:
                    continue

                # Fast internal OBB collision test
                if manager.in_collision_internal():
                    sub_mgr = trimesh.collision.CollisionManager()
                    sub_mgr.add_object("A", self.trimesh_obbs[i], transform=transforms[i])
                    sub_mgr.add_object("B", self.trimesh_obbs[j], transform=transforms[j])
                    if sub_mgr.in_collision_internal():
                        collided_indices.update([i, j])

        return collided_indices

    def _homogeneous_transform(self, item: gl.GLLinePlotItem | gl.GLMeshItem, transformation: np.ndarray) -> None:
        item.resetTransform()
        try:
            rot = R.from_matrix(transformation[:3, :3])
            item.rotate(rot.magnitude() * 180 / np.pi, *rot.as_rotvec())
            item.translate(transformation[0, 3], transformation[1, 3], transformation[2, 3])
        except Exception:
            item.resetTransform()

    def _create_frame(self, size: float = 300) -> list[gl.GLLinePlotItem]:
        return [
            gl.GLLinePlotItem(pos=[[0, 0, 0], [size, 0, 0]], color=(1, 0, 0, 1), width=2),
            gl.GLLinePlotItem(pos=[[0, 0, 0], [0, size, 0]], color=(0, 1, 0, 1), width=2),
            gl.GLLinePlotItem(pos=[[0, 0, 0], [0, 0, size]], color=(0, 0, 1, 1), width=2)
        ]

    def _load_stl(self, path, color: tuple[float] = (0.3, 0.6, 1.0, 1), bbox_list: list = None) -> gl.GLMeshItem:
        tm_mesh = trimesh.load(path)
        vertices, faces = tm_mesh.vertices, tm_mesh.faces

        item = gl.GLMeshItem(
            vertexes=vertices, faces=faces, smooth=False, drawEdges=True,
            edgeColor=(0, 0, 0, 0.4), shader="shaded", polygonOffset=True
        )
        item.setColor(color)
        item.setGLOptions('translucent')

        if bbox_list is not None:
            # Generate oriented bounding box (OBB) using trimesh
            obb = tm_mesh.bounding_box_oriented
            self.trimesh_obbs.append(obb)

            # Extract local unrotated extents for the wireframe box
            extents = obb.primitive.extents / 2.0
            x, y, z = extents

            # Define 12 local edges aligned centered at (0,0,0)
            lines = np.array([
                [-x, -y, -z], [x, -y, -z], [x, -y, -z], [x, y, -z],
                [x, y, -z], [-x, y, -z], [-x, y, -z], [-x, -y, -z],
                [-x, -y, z], [x, -y, z], [x, -y, z], [x, y, z],
                [x, y, z], [-x, y, z], [-x, y, z], [-x, -y, z],
                [-x, -y, -z], [-x, -y, z], [x, -y, -z], [x, -y, z],
                [x, y, -z], [x, y, z], [-x, y, -z], [-x, y, z],
            ])

            bbox_item = gl.GLLinePlotItem(pos=lines, color=(1, 1, 0, 0.8), width=1.5, mode='lines')
            bbox_item.setVisible(False)

            # Store the local OBB offset transformation inside the pyqtgraph item
            bbox_item.obb_offset = obb.primitive.transform
            bbox_list.append(bbox_item)

        return item

    def _load_links(self, color: tuple[float], bbox_list: list = None) -> list[gl.GLMeshItem]:
        files = ["Base.stl", "Shoulder.stl", "Upper_Arm.stl", "Lower_Arm.stl",
                 "Wrist.stl", "Hand.stl", "Tool1.stl", "Bracket.stl", "Bracket.stl"]
        try:
            return [self._load_stl(BASE_DIR / "gui" / "desktop" / "mesh_model" / f, color, bbox_list) for f in files]
        except Exception as e:
            self.log.warning(f"stl files not found. {e}")
            return []
