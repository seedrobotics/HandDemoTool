from OpenGL.GL import (
	GL_COLOR_BUFFER_BIT,
	GL_DEPTH_BUFFER_BIT,
	GL_DEPTH_TEST,
	GL_LEQUAL,
	GL_BLEND,
	GL_LIGHTING,
	GL_LIGHT0,
	GL_POSITION,
	GL_DIFFUSE,
	GL_AMBIENT,
	GL_SPECULAR,
	GL_COLOR_MATERIAL,
	GL_NORMALIZE,
	GL_POLYGON_SMOOTH,
	GL_LINE_SMOOTH,
	GL_LINES,
	GL_LINE_STRIP,
	GL_TRIANGLES,
	GL_COMPILE,
	glBegin,
	glClear,
	glClearColor,
	glClearDepth,
	glColor3f,
	glEnable,
	glDisable,
	glDepthMask,
	glLightfv,
	glNormal3f,
	glDepthFunc,
	glEnd,
	glCallList,
	glEndList,
	glGenLists,
	glLineWidth,
	glShadeModel,
	glLoadIdentity,
	glMatrixMode,
	glMultMatrixf,
	glNewList,
	glPopMatrix,
	glPushMatrix,
	glRotatef,
	glVertex3f,
	glViewport,
	GL_MODELVIEW,
	GL_PROJECTION,
	GL_FLAT,
)
from OpenGL.GLU import gluLookAt, gluPerspective
from PySide6.QtCore import QTimer, Qt
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtGui import QSurfaceFormat, QPainter, QFont, QColor
from dataclasses import dataclass
from pathlib import Path
from roboticstoolbox import Robot
import numpy as np
import trimesh
import xml.etree.ElementTree as ET
import sys
import time

BG_COLOR = (100 / 255,100 / 255,100 / 255 , 1)

def dyn_pos_to_rad(position, min_pos=0, max_pos=4095):
	pos = max(min_pos, min(max_pos, float(position)))
	span = max_pos - min_pos
	#if span <= 0:
	#	return 0.0
	return (pos - min_pos) / span * (2.0 * np.pi)


@dataclass
class VisualMesh:
	link_name: str
	mesh_path: str
	local_T: np.ndarray
	color: tuple


def _parse_floats(text, count, default=0.0):
	if text is None:
		return [default] * count
	values = [float(v) for v in text.strip().split()]
	if len(values) < count:
		values.extend([default] * (count - len(values)))
	return values[:count]


def _rpy_to_matrix(roll, pitch, yaw):
	cr, sr = np.cos(roll), np.sin(roll)
	cp, sp = np.cos(pitch), np.sin(pitch)
	cy, sy = np.cos(yaw), np.sin(yaw)
	Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
	Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
	Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
	return Rx @ Ry @ Rz


def _make_transform(xyz, rpy, scale):
	T = np.eye(4)
	T[:3, :3] = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])
	T[:3, 3] = xyz
	S = np.eye(4)
	S[0, 0] = scale[0]
	S[1, 1] = scale[1]
	S[2, 2] = scale[2]
	return T @ S


def _resolve_mesh_path(urdf_path, filename):
	if filename.startswith("package://"):
		filename = filename.replace("package://", "", 1)
	if Path(filename).is_absolute():
		return filename
	return str(Path(urdf_path).parent / filename)


def _parse_urdf_materials(root):
	materials = {}
	for material in root.findall("material"):
		name = material.attrib.get("name")
		if not name:
			continue
		color_elem = material.find("color")
		if color_elem is None:
			continue
		rgba = _parse_floats(color_elem.attrib.get("rgba"), 4, 1.0)
		materials[name] = tuple(rgba)
	return materials


def _parse_urdf_visuals(urdf_path):
	root = ET.parse(urdf_path).getroot()
	materials = _parse_urdf_materials(root)
	visuals = []
	for link in root.findall("link"):
		link_name = link.attrib.get("name", "")
		for visual in link.findall("visual"):
			origin = visual.find("origin")
			xyz = _parse_floats(origin.attrib.get("xyz") if origin is not None else None, 3, 0.0)
			rpy = _parse_floats(origin.attrib.get("rpy") if origin is not None else None, 3, 0.0)

			geom = visual.find("geometry")
			mesh = geom.find("mesh") if geom is not None else None
			if mesh is None:
				continue
			filename = mesh.attrib.get("filename")
			if not filename:
				continue
			scale = _parse_floats(mesh.attrib.get("scale"), 3, 1.0)
			mesh_path = _resolve_mesh_path(urdf_path, filename)

			color = (1,0,0,1)
			material = visual.find("material")
			if material is not None:
				color_elem = material.find("color")
				if color_elem is not None:
					rgba = _parse_floats(color_elem.attrib.get("rgba"), 4, 1.0)
					color = tuple(rgba)
				else:
					mat_name = material.attrib.get("name")
					if mat_name and mat_name in materials:
						color = materials[mat_name]

			local_T = _make_transform(np.array(xyz), np.array(rpy), np.array(scale))
			visuals.append(VisualMesh(link_name=link_name, mesh_path=mesh_path, local_T=local_T, color=color))
	return visuals


class RobotGLWidget(QOpenGLWidget):
	def __init__(self, is_left, parent=None):

		super().__init__(parent)
		fmt = QSurfaceFormat()
		fmt.setDepthBufferSize(24)
		fmt.setStencilBufferSize(8)
		self.setFormat(fmt)

		if is_left:
			URDF_PATH = Path(__file__).parent / "urdf" / "rh8d_l.urdf"
		else:
			URDF_PATH = Path(__file__).parent / "urdf" / "rh8d_r.urdf"

		self.robot = Robot.URDF(URDF_PATH)
		self.q = np.zeros(self.robot.n)
		self._joint_limits = getattr(self.robot, "qlim", None)
		self._start_time = time.monotonic()
		self._spin_rate = np.deg2rad(8.0)
		self._visuals = _parse_urdf_visuals(URDF_PATH)
		self._mesh_cache = {}
		self._missing_meshes = set()
		self._max_faces = 4000
		self._display_lists = {}
		self._frame_count = 0
		self._fps = 0.0
		self._last_fps_time = time.monotonic()

		print(self.robot)
		self.w_rotation = 0
		self.w_adduction = 1
		self.w_flexation = 2
		self.th_adduction = 3
		if is_left:
			self.th_flexation = [4,5,6]
			self.i_flexation = [8,9,10]
			self.m_flexation = [12,13,14]
			self.r_flexation = [16,17,18]
			self.ltl_flexation = [20,21,22]
		else:
			self.th_flexation = [4,5,6]
			self.i_flexation = [7,8,9]
			self.m_flexation = [10,11,12]
			self.r_flexation = [13,14,15]
			self.ltl_flexation = [16,17,18]

		self._timer = QTimer(self)
		self._timer.setTimerType(Qt.PreciseTimer)
		self._timer.timeout.connect(self.update)
		self._timer.start(30)

		self._tilt_yaw = 0.0
		self._tilt_pitch = 0.0
		self._zoom = 1.0
		self._dragging = False
		self._last_mouse_pos = None
		self._tilt_sensitivity = 0.3
		self._zoom_sensitivity = 0.0015
		self._min_zoom = 0.35
		self._max_zoom = 3.0
		self.setFocusPolicy(Qt.StrongFocus)

	def initializeGL(self):
		glClearColor(BG_COLOR[0], BG_COLOR[1], BG_COLOR[2], BG_COLOR[3])
		glClearDepth(1.0)
		glEnable(GL_DEPTH_TEST)
		glDepthFunc(GL_LEQUAL)
		glDisable(GL_BLEND)
		glDisable(GL_POLYGON_SMOOTH)
		glDisable(GL_LINE_SMOOTH)
		glDepthMask(True)
		glEnable(GL_LIGHTING)
		#glEnable(GL_LIGHT0)
		glEnable(GL_COLOR_MATERIAL)
		glEnable(GL_NORMALIZE)
		#glShadeModel(GL_FLAT)
		glLightfv(GL_LIGHT0, GL_POSITION, [0.6, -0.8, 0.9, 1.0])
		glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])
		glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1.0])
		glLightfv(GL_LIGHT0, GL_SPECULAR, [0.8, 0.8, 0.8, 1.0])
		glLineWidth(2.0)

	def resizeGL(self, width, height):
		if height == 0:
			height = 1
		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45.0, float(width) / float(height), 0.01, 100.0)

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
		glEnable(GL_DEPTH_TEST)
		glDepthFunc(GL_LEQUAL)
		glDisable(GL_BLEND)
		glDepthMask(True)
		glEnable(GL_LIGHTING)
		glEnable(GL_LIGHT0)
		eye_x = 0.6 * self._zoom
		eye_y = -0.8 * self._zoom
		eye_z = 0.6 * self._zoom
		gluLookAt(eye_x, eye_y, eye_z, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0)
		glRotatef(self._tilt_pitch, 1.0, 0.0, 0.0)
		glRotatef(self._tilt_yaw, 0.0, 0.0, 1.0)

		self._draw_axes(0.2)
		self._draw_robot_meshes()

		now = time.monotonic()
		self._frame_count += 1
		elapsed = now - self._last_fps_time
		if elapsed >= 0.5:
			self._fps = self._frame_count / elapsed
			self._frame_count = 0
			self._last_fps_time = now

		painter = QPainter(self)
		painter.setPen(QColor(240, 240, 240))
		painter.setFont(QFont("Sans Serif", 11))
		painter.drawText(10, 20, f"FPS: {self._fps:.1f}")
		painter.end()

	def mousePressEvent(self, event):
		if event.button() == Qt.LeftButton:
			self._dragging = True
			self._last_mouse_pos = event.position()
			event.accept()
			return
		super().mousePressEvent(event)

	def mouseMoveEvent(self, event):
		if self._dragging and self._last_mouse_pos is not None:
			pos = event.position()
			dx = pos.x() - self._last_mouse_pos.x()
			dy = pos.y() - self._last_mouse_pos.y()
			self._tilt_yaw += dx * self._tilt_sensitivity
			self._tilt_pitch += dy * self._tilt_sensitivity
			self._tilt_pitch = max(-89.0, min(89.0, self._tilt_pitch))
			self._last_mouse_pos = pos
			self.update()
			event.accept()
			return
		super().mouseMoveEvent(event)

	def mouseReleaseEvent(self, event):
		if event.button() == Qt.LeftButton:
			self._dragging = False
			self._last_mouse_pos = None
			event.accept()
			return
		super().mouseReleaseEvent(event)

	def wheelEvent(self, event):
		delta = event.angleDelta().y()
		if delta:
			self._zoom *= 1.0 - (delta * self._zoom_sensitivity)
			self._zoom = max(self._min_zoom, min(self._max_zoom, self._zoom))
			self.update()
			event.accept()
			return
		super().wheelEvent(event)

	def _draw_axes(self, scale):
		glBegin(GL_LINES)
		glColor3f(1.0, 0.2, 0.2)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(scale, 0.0, 0.0)

		glColor3f(0.2, 1.0, 0.2)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(0.0, scale, 0.0)

		glColor3f(0.2, 0.4, 1.0)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(0.0, 0.0, scale)
		glEnd()

	def _draw_local_frame(self, scale=0.04):
		glLineWidth(3.0)
		glBegin(GL_LINES)
		glColor3f(1.0, 0.1, 0.1)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(scale, 0.0, 0.0)

		glColor3f(0.1, 1.0, 0.1)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(0.0, scale, 0.0)

		glColor3f(0.1, 0.3, 1.0)
		glVertex3f(0.0, 0.0, 0.0)
		glVertex3f(0.0, 0.0, scale)
		glEnd()
		glLineWidth(2.0)

	def _draw_link_frames(self, link_T, scale=0.08):
		glDisable(GL_DEPTH_TEST)
		for T in link_T.values():
			glPushMatrix()
			glMultMatrixf(T.T.astype(np.float32))
			self._draw_local_frame(scale=scale)
			glPopMatrix()
		glEnable(GL_DEPTH_TEST)

	#def _draw_robot_chain(self):
	#	transforms = self.robot.fkine_all(self.q)
	#	points = [np.zeros(3)]
	#	for tform in transforms:
	#		try:
	#			points.append(np.array(tform.t).reshape(3))
	#		except Exception:
	#			pass
	#	glColor3f(0.95, 0.85, 0.4)
	#	glBegin(GL_LINE_STRIP)
	#	for pt in points:
	#		glVertex3f(float(pt[0]), float(pt[1]), float(pt[2]))
	#	glEnd()

	def _link_transforms(self):
		transforms = self.robot.fkine_all(self.q)
		if len(transforms) == len(self.robot.links) + 1:
			transforms = transforms[1:]
		if len(transforms) != len(self.robot.links):
			return {}
		return {link.name: np.asarray(tform.A) for link, tform in zip(self.robot.links, transforms)}

	def _load_meshes(self, mesh_path):
		if mesh_path in self._mesh_cache:
			return self._mesh_cache[mesh_path]
		if mesh_path in self._missing_meshes:
			return []
		try:
			loaded = trimesh.load(mesh_path, force="mesh")
		except Exception:
			self._missing_meshes.add(mesh_path)
			return []

		if isinstance(loaded, trimesh.Scene):
			meshes = list(loaded.geometry.values())
		else:
			meshes = [loaded]

		optimized = []
		for mesh in meshes:
			if hasattr(mesh, "faces") and len(mesh.faces) > self._max_faces:
				try:
					mesh = mesh.simplify_quadratic_decimation(self._max_faces)
				except Exception:
					pass
			optimized.append(mesh)
		meshes = optimized
		self._mesh_cache[mesh_path] = meshes
		return meshes

	def _get_display_lists(self, mesh_path):
		if mesh_path in self._display_lists:
			return self._display_lists[mesh_path]
		meshes = self._load_meshes(mesh_path)
		if not meshes:
			self._display_lists[mesh_path] = []
			return []

		lists = []
		for mesh in meshes:
			list_id = glGenLists(1)
			glNewList(list_id, GL_COMPILE)
			glBegin(GL_TRIANGLES)
			vertices = np.asarray(mesh.vertices)
			faces = np.asarray(mesh.faces)
			normals = np.asarray(mesh.face_normals) if hasattr(mesh, "face_normals") else None
			for face_index, face in enumerate(faces):
				if normals is not None and len(normals) > face_index:
					n = normals[face_index]
					glNormal3f(float(n[0]), float(n[1]), float(n[2]))
				for vidx in face:
					v = vertices[vidx]
					glVertex3f(float(v[0]), float(v[1]), float(v[2]))
			glEnd()
			glEndList()
			lists.append(list_id)

		self._display_lists[mesh_path] = lists
		return lists

	def _draw_robot_meshes(self):
		link_T = self._link_transforms()
		#angle = (time.monotonic() - self._start_time) * self._spin_rate
		global_R = _rpy_to_matrix(0.0, 0.0, 0.0)
		global_T = np.eye(4)
		global_T[:3, :3] = global_R
		#self._draw_link_frames(link_T, scale=0.08)
		for visual in self._visuals:
			if visual.link_name not in link_T:
				continue
			lists = self._get_display_lists(visual.mesh_path)
			if not lists:
				continue
			T = global_T @ link_T[visual.link_name] @ visual.local_T
			glColor3f(visual.color[0], visual.color[1], visual.color[2])
			#print(visual.color)
			glPushMatrix()
			glMultMatrixf(T.T.astype(np.float32))
			for list_id in lists:
				glCallList(list_id)
			glPopMatrix()

			#glDisable(GL_DEPTH_TEST)
			#glPushMatrix()
			#glMultMatrixf(T.T.astype(np.float32))
			#self._draw_local_frame(scale=1)
			#glPopMatrix()
			#glEnable(GL_DEPTH_TEST)

	def _randomize_configuration(self):
		if self._joint_limits is None or self._joint_limits.size == 0:
			self.q = np.random.uniform(-0.8, 0.8, size=self.robot.n)
			return
		low = np.asarray(self._joint_limits[0], dtype=float)
		high = np.asarray(self._joint_limits[1], dtype=float)
		mask = np.isfinite(low) & np.isfinite(high) & (low < high)
		q = np.zeros(self.robot.n, dtype=float)
		q[mask] = np.random.uniform(low[mask], high[mask])
		q[~mask] = np.random.uniform(-0.8, 0.8, size=np.count_nonzero(~mask))
		print(q)
		self.q = q

	def _map_to_limits(self, joint_index, position):
		if self._joint_limits is None or self._joint_limits.size == 0:
			return dyn_pos_to_rad(position)
		low = float(self._joint_limits[0][joint_index])
		high = float(self._joint_limits[1][joint_index])
		if not np.isfinite(low) or not np.isfinite(high) or low >= high:
			return dyn_pos_to_rad(position)
		norm = min(1.0, float(position) / 4095.0)
		#print(f"idx {joint_index} low {low}, high {high} norm {norm} pos {position}")
		return low + norm * (high - low)


	#fingers move 0 1 2
	def map_finger(self, state, finger):
		new_q = np.zeros(3)
		#more then a 3rd: first joint at max
		if state > 4095/ 3:
			new_q[0] = self._map_to_limits(finger[0], 4095)
		#less then a 3rd: only control first joint
		else:
			new_q[0] = self._map_to_limits(finger[0], state * 3)
		#more than 2 3rd: 2. Joint at max
		if state > 2 * 4095 / 3:
			new_q[1] = self._map_to_limits(finger[1], 4095)
		#its in the interval
		elif state > 4095 / 3:
			new_q[1] = self._map_to_limits(finger[1], (state - 4095/3) * 3)
		#its less, joint at 0
		else: 
			new_q[1] = self._map_to_limits(finger[1], 0)

		if state > 2 * 4095 / 3:
			new_q[2] = self._map_to_limits(finger[2], (state - 2*4095/3) * 3)
		else:
			new_q[2] = self._map_to_limits(finger[2], 0)
		return new_q
	
	#Thumb moves 2 1 0
	#tuned thumb ranges but not perfect yet
	def map_thumb(self, state, finger):
		new_q = np.zeros(3)
		#more then a 3rd: first joint at max
		if state > 4095/ 3:
			new_q[2] = self._map_to_limits(finger[2], 4095)
		#less then a 3rd: only control first joint
		else:
			new_q[2] = self._map_to_limits(finger[2], state * 3)
		#more than 2 3rd: 2. Joint at max
		if state > 2 * 4095 / 3:
			new_q[1] = self._map_to_limits(finger[1], 4095)
		#its in the interval
		elif state > 4095 / 3:
			new_q[1] = self._map_to_limits(finger[1], (state - 4095/3) * 3)
		#its less, joint at 0
		else: 
			new_q[1] = self._map_to_limits(finger[1], 0)

		if state > 2 * 4095 / 3:
			new_q[0] = self._map_to_limits(finger[0], (state - 2*4095/3) * 3)
		else:
			new_q[0] = self._map_to_limits(finger[0], 0)
		return new_q


	def map_HandState_to_q(self, state):
		new_q = np.zeros(self.robot.n)
		#Rotational Joints
		new_q[self.w_rotation] = self._map_to_limits(self.w_rotation, state[0])
		new_q[self.w_adduction] = self._map_to_limits(self.w_adduction, state[1])
		new_q[self.w_flexation] = self._map_to_limits(self.w_flexation, state[2])
		new_q[self.th_adduction] = self._map_to_limits(self.th_adduction,state[3])

		#Fingers
		new_q[self.th_flexation]  = self.map_thumb(state[4], self.th_flexation)  #dyn_pos_to_rad(state[4]) / 3
		new_q[self.i_flexation]   = self.map_finger(state[5], self.i_flexation)   #dyn_pos_to_rad(state[5]) / 3
		new_q[self.m_flexation]   = self.map_finger(state[6], self.m_flexation)   #dyn_pos_to_rad(state[6]) / 3
		new_q[self.r_flexation]   = self.map_finger(state[7], self.r_flexation)   #dyn_pos_to_rad(state[7]) / 3
		new_q[self.ltl_flexation] = self.map_finger(state[7], self.ltl_flexation) #dyn_pos_to_rad(state[7]) / 3

		self.q = new_q