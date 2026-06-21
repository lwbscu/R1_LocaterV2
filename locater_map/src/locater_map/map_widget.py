from __future__ import annotations

from pathlib import Path
from collections import deque
from typing import Any

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QMouseEvent, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
)

from .config_loader import resolve_resource
from .data_model import RobotFrame
from .utils_transform import dt35_ray


class FieldMapView(QGraphicsView):
    mouse_position_changed = Signal(float, float)

    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__()
        self.config = config
        self.scene_obj = QGraphicsScene(self)
        self.setScene(self.scene_obj)
        self.setRenderHints(QPainter.RenderHint.Antialiasing | QPainter.RenderHint.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setMouseTracking(True)
        self.setBackgroundBrush(QColor("#10151c"))
        self.follow_robot = bool(config["display"].get("follow_robot", True))
        self.layers = {"pos": True, "calib": True, "lidar": True, "dt35": True, "grid": True, "axes": True}
        self._paths: dict[str, QPainterPath] = {}
        self._path_items: dict[str, QGraphicsPathItem] = {}
        self._path_points: dict[str, deque[QPointF]] = {}
        self._dt35_items: list[QGraphicsItem] = []
        self._grid_items: list[QGraphicsItem] = []
        self._axis_items: list[QGraphicsItem] = []
        self._frame: RobotFrame | None = None
        self._build_scene()

    def _world_to_scene(self, x: float, y: float) -> QPointF:
        return QPointF(x, -y)

    def _build_scene(self) -> None:
        width = float(self.config["map"]["field_width_cm"])
        height = float(self.config["map"]["field_height_cm"])
        self.scene_obj.setSceneRect(QRectF(-width / 2, -height / 2, width, height))
        self._add_background(width, height)
        self._add_grid(width, height)
        self._add_paths()
        self._add_robot_items()
        self.fitInView(self.scene_obj.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def _add_background(self, width: float, height: float) -> None:
        image_path = resolve_resource(self.config, self.config["map"].get("labeled_background_image"))
        if not image_path or not image_path.exists():
            image_path = resolve_resource(self.config, self.config["map"].get("background_image"))
        if image_path and image_path.exists():
            pix = QPixmap(str(image_path))
            item = QGraphicsPixmapItem(pix)
            item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
            item.setPos(-width / 2, -height / 2)
            item.setScale(width / pix.width())
            item.setZValue(-100)
            self.scene_obj.addItem(item)
        else:
            self.scene_obj.addRect(-width / 2, -height / 2, width, height, QPen(QColor("#58606d")), QColor("#19212a"))
        self.scene_obj.addRect(-width / 2, -height / 2, width, height, QPen(QColor("#d0d7de"), 1.5))

    def _add_grid(self, width: float, height: float) -> None:
        step = float(self.config["map"].get("grid_step_cm", 50.0))
        pen = QPen(QColor(120, 130, 145, 80), 0)
        x = -width / 2
        while x <= width / 2:
            item = self.scene_obj.addLine(x, -height / 2, x, height / 2, pen)
            item.setZValue(-50)
            self._grid_items.append(item)
            x += step
        y = -height / 2
        while y <= height / 2:
            item = self.scene_obj.addLine(-width / 2, y, width / 2, y, pen)
            item.setZValue(-50)
            self._grid_items.append(item)
            y += step
        axis_pen_x = QPen(QColor("#ff6b6b"), 0)
        axis_pen_y = QPen(QColor("#4dabf7"), 0)
        x_axis = self.scene_obj.addLine(-width / 2, 0, width / 2, 0, axis_pen_x)
        y_axis = self.scene_obj.addLine(0, -height / 2, 0, height / 2, axis_pen_y)
        x_axis.setZValue(-40)
        y_axis.setZValue(-40)
        self._axis_items.extend([x_axis, y_axis])

    def _add_paths(self) -> None:
        specs = {
            "pos": QColor(0, 255, 170, 210),
            "calib": QColor(255, 192, 0, 180),
            "lidar": QColor(70, 160, 255, 190),
        }
        for name, color in specs.items():
            path = QPainterPath()
            item = QGraphicsPathItem(path)
            item.setPen(QPen(color, 2.0))
            item.setZValue(20)
            self.scene_obj.addItem(item)
            self._paths[name] = path
            self._path_items[name] = item
            self._path_points[name] = deque(maxlen=int(self.config["display"].get("max_trajectory_points", 5000)))

    def _add_robot_items(self) -> None:
        size = float(self.config["robot"].get("size_cm", 83.0))
        tex_path = resolve_resource(self.config, self.config["robot"].get("texture_path"))
        if tex_path and tex_path.exists():
            pix = QPixmap(str(tex_path))
            self.robot = QGraphicsPixmapItem(pix)
            self.robot.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
            self.robot.setOffset(-pix.width() / 2, -pix.height() / 2)
            self.robot.setScale(size / pix.width())
        else:
            self.robot = QGraphicsRectItem(-size / 2, -size / 2, size, size)
            self.robot.setBrush(QColor(80, 180, 255, 110))
            self.robot.setPen(QPen(QColor("#9cdcfe"), 0))
        self.robot.setZValue(50)
        self.scene_obj.addItem(self.robot)

        self.robot_box = QGraphicsRectItem(-size / 2, -size / 2, size, size, self.robot)
        self.robot_box.setPen(QPen(QColor("#ffffff"), 0))
        self.robot_box.setBrush(Qt.BrushStyle.NoBrush)
        self.center_dot = QGraphicsEllipseItem(-3, -3, 6, 6, self.robot)
        self.center_dot.setBrush(QColor("#ffffff"))
        self.center_dot.setPen(QPen(QColor("#10151c"), 0))
        self.front_line = QGraphicsLineItem(0, 0, size * 0.75, 0, self.robot)
        self.front_line.setPen(QPen(QColor("#00ffaa"), 0))
        self.x_axis = QGraphicsLineItem(0, 0, 0, size * 0.55, self.robot)
        self.x_axis.setPen(QPen(QColor("#ff6b6b"), 0))
        self.y_axis = QGraphicsLineItem(0, 0, size * 0.55, 0, self.robot)
        self.y_axis.setPen(QPen(QColor("#4dabf7"), 0))

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        self.layers[layer] = visible
        if layer in self._path_items:
            self._path_items[layer].setVisible(visible)
        if layer == "dt35":
            for item in self._dt35_items:
                item.setVisible(visible)
        if layer == "grid":
            for item in self._grid_items:
                item.setVisible(visible)
        if layer == "axes":
            for item in self._axis_items:
                item.setVisible(visible)

    def clear_trajectories(self) -> None:
        for name, item in self._path_items.items():
            self._paths[name] = QPainterPath()
            item.setPath(self._paths[name])
            self._path_points[name].clear()

    def update_frame(self, frame: RobotFrame) -> None:
        self._frame = frame
        point = self._world_to_scene(frame.pos_x_cm, frame.pos_y_cm)
        self.robot.setPos(point)
        texture_front = float(self.config["robot"].get("texture_front_dir_deg_in_image", 0.0))
        yaw_offset = float(self.config["robot"].get("yaw_offset_deg", 0.0))
        self.robot.setRotation((frame.pos_yaw_deg + yaw_offset - 90.0) - texture_front)
        self._append_path("pos", frame.pos_x_cm, frame.pos_y_cm)
        self._append_path("calib", frame.calib_x_cm, frame.calib_y_cm)
        if frame.lidar_valid:
            self._append_path("lidar", frame.lidar_x_cm, frame.lidar_y_cm)
        self._update_dt35(frame)
        if self.follow_robot:
            self.centerOn(point)

    def _append_path(self, name: str, x: float, y: float) -> None:
        point = self._world_to_scene(x, y)
        points = self._path_points[name]
        if points:
            last = points[-1]
            min_step = float(self.config["display"].get("trajectory_min_step_cm", 1.0))
            if (last - point).manhattanLength() < min_step:
                return
        points.append(point)

        path = QPainterPath()
        for i, item in enumerate(points):
            if i == 0:
                path.moveTo(item)
            else:
                path.lineTo(item)
        self._paths[name] = path
        self._path_items[name].setPath(path)

    def _update_dt35(self, frame: RobotFrame) -> None:
        for item in self._dt35_items:
            self.scene_obj.removeItem(item)
        self._dt35_items.clear()
        if not self.layers.get("dt35", True):
            return
        for key, distance in (("sensor_1", frame.dt35_1_mm), ("sensor_2", frame.dt35_2_mm)):
            cfg = self.config["dt35"].get(key, {})
            ray = dt35_ray(frame.pos_x_cm, frame.pos_y_cm, frame.pos_yaw_deg, cfg, distance)
            color = QColor("#00ffaa") if ray["valid"] else QColor(160, 160, 160, 140)
            pen = QPen(color, 1.5)
            if not ray["valid"]:
                pen.setStyle(Qt.PenStyle.DashLine)
            a = self._world_to_scene(float(ray["sensor_x_cm"]), float(ray["sensor_y_cm"]))
            b = self._world_to_scene(float(ray["hit_x_cm"]), float(ray["hit_y_cm"]))
            line = self.scene_obj.addLine(a.x(), a.y(), b.x(), b.y(), pen)
            dot = self.scene_obj.addEllipse(b.x() - 3, b.y() - 3, 6, 6, QPen(color), color)
            line.setZValue(35)
            dot.setZValue(36)
            line.setVisible(self.layers.get("dt35", True))
            dot.setVisible(self.layers.get("dt35", True))
            self._dt35_items.extend([line, dot])

    def wheelEvent(self, event) -> None:  # type: ignore[no-untyped-def]
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        pos = self.mapToScene(event.position().toPoint())
        self.mouse_position_changed.emit(pos.x(), -pos.y())
        super().mouseMoveEvent(event)

    def reset_view(self) -> None:
        self.fitInView(self.scene_obj.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def save_screenshot(self, path: str | Path) -> None:
        pixmap = self.grab()
        pixmap.save(str(path))
