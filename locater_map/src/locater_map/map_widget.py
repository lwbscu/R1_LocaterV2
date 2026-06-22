from __future__ import annotations

from collections import deque
from math import isfinite
from pathlib import Path
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
from .utils_transform import dt35_ray, dt35_yaw_from_frame, heading_vector_from_front_yaw


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
        self.layers = {"pos": True, "calib": True, "lidar": True, "dt35": True, "field_model": True, "grid": True, "axes": True}
        self._paths: dict[str, QPainterPath] = {}
        self._path_items: dict[str, QGraphicsPathItem] = {}
        self._path_points: dict[str, deque[QPointF]] = {}
        self._dt35_items: list[QGraphicsItem] = []
        self._field_model_items: list[QGraphicsItem] = []
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
        self._add_field_model_overlay()
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

    def _target_style(self, target_type: str) -> tuple[QPen, QColor]:
        if target_type == "ignore":
            return QPen(QColor(45, 130, 255, 210), 2.5), QColor(45, 130, 255, 35)
        if target_type == "solid_obstacle":
            return QPen(QColor(80, 255, 130, 230), 2.5), QColor(80, 255, 130, 38)
        if target_type == "blocker":
            return QPen(QColor(255, 160, 75, 210), 2.0), QColor(255, 160, 75, 35)
        return QPen(QColor(255, 65, 75, 230), 2.5), QColor(255, 65, 75, 25)

    def _add_field_model_overlay(self) -> None:
        model = self.config.get("field_model", {})
        if not bool(model.get("enabled", False)):
            return
        for item in model.get("segments", []):
            if not bool(item.get("enabled", True)):
                continue
            target_type = str(item.get("target_type", "usable_wall"))
            pen, _brush = self._target_style(target_type)
            a = self._world_to_scene(float(item["x1_cm"]), float(item["y1_cm"]))
            b = self._world_to_scene(float(item["x2_cm"]), float(item["y2_cm"]))
            line = self.scene_obj.addLine(a.x(), a.y(), b.x(), b.y(), pen)
            line.setZValue(-25)
            line.setToolTip(f"{item.get('name', 'segment')} [{target_type}]")
            self._field_model_items.append(line)

        for item in model.get("rectangles", []):
            if not bool(item.get("enabled", True)):
                continue
            target_type = str(item.get("target_type", "blocker"))
            pen, brush = self._target_style(target_type)
            cx = float(item.get("center_x_cm", 0.0))
            cy = float(item.get("center_y_cm", 0.0))
            width = float(item.get("width_cm", 0.0))
            height = float(item.get("height_cm", 0.0))
            rect = QRectF(cx - width * 0.5, -(cy + height * 0.5), width, height)
            rect_item = self.scene_obj.addRect(rect, pen, brush)
            rect_item.setZValue(-24)
            rect_item.setToolTip(f"{item.get('name', 'rect')} [{target_type}]")
            self._field_model_items.append(rect_item)

        for item in self._field_model_items:
            item.setVisible(self.layers.get("field_model", True))

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
        if layer == "field_model":
            for item in self._field_model_items:
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
        field_model = dict(self.config.get("field_model", {}))
        field_model.setdefault("field_width_cm", self.config.get("map", {}).get("field_width_cm", 1215.0))
        field_model.setdefault("field_height_cm", self.config.get("map", {}).get("field_height_cm", 1210.0))
        residual_warn = float(field_model.get("residual_warn_cm", 8.0))
        yaw_for_dt35 = dt35_yaw_from_frame(frame)
        for key, distance in (("sensor_1", frame.dt35_1_mm), ("sensor_2", frame.dt35_2_mm)):
            cfg = self.config["dt35"].get(key, {})
            ray = dt35_ray(frame.pos_x_cm, frame.pos_y_cm, yaw_for_dt35, cfg, distance, field_model)
            residual = float(ray["residual_cm"])
            target_type = str(ray.get("expected_target_type", ""))
            correction_allowed = bool(ray.get("correction_allowed", False))
            corner_ambiguous = bool(ray.get("corner_ambiguous", False))
            if ray["valid"] and not correction_allowed:
                if corner_ambiguous:
                    color = QColor("#ff4d4d")
                elif target_type == "blocker":
                    color = QColor("#ff9955")
                elif target_type == "ignore":
                    color = QColor("#b08cff")
                else:
                    color = QColor("#ffcc33")
            elif ray["valid"] and isfinite(residual) and abs(residual) > residual_warn:
                color = QColor("#ffcc33")
            else:
                color = QColor("#00ffaa") if ray["valid"] else QColor(160, 160, 160, 140)
            pen = QPen(color, 1.5)
            if not ray["valid"]:
                pen.setStyle(Qt.PenStyle.DashLine)
            a = self._world_to_scene(float(ray["sensor_x_cm"]), float(ray["sensor_y_cm"]))
            b = self._world_to_scene(float(ray["hit_x_cm"]), float(ray["hit_y_cm"]))
            line = self.scene_obj.addLine(a.x(), a.y(), b.x(), b.y(), pen)
            emitter = self.scene_obj.addEllipse(a.x() - 2.5, a.y() - 2.5, 5, 5, QPen(QColor("#ffffff")), QColor("#ffffff"))
            dot = self.scene_obj.addEllipse(b.x() - 3, b.y() - 3, 6, 6, QPen(color), color)
            tip = self._dt35_tooltip(key, ray)
            line.setToolTip(tip)
            emitter.setToolTip(tip)
            dot.setToolTip(tip)
            line.setZValue(35)
            emitter.setZValue(36)
            dot.setZValue(36)
            line.setVisible(self.layers.get("dt35", True))
            emitter.setVisible(self.layers.get("dt35", True))
            dot.setVisible(self.layers.get("dt35", True))
            self._dt35_items.extend([line, emitter, dot])

            expected_x = float(ray["expected_hit_x_cm"])
            expected_y = float(ray["expected_hit_y_cm"])
            if isfinite(expected_x) and isfinite(expected_y):
                e = self._world_to_scene(expected_x, expected_y)
                if target_type == "blocker":
                    expected_color = QColor("#ff9955")
                elif target_type == "solid_obstacle":
                    expected_color = QColor("#ffa94d")
                elif target_type == "ignore":
                    expected_color = QColor("#b08cff")
                else:
                    expected_color = QColor("#4aa3ff")
                expected_pen = QPen(expected_color, 1.0)
                expected_pen.setStyle(Qt.PenStyle.DotLine)
                expected_line = self.scene_obj.addLine(a.x(), a.y(), e.x(), e.y(), expected_pen)
                expected_dot = self.scene_obj.addEllipse(e.x() - 2, e.y() - 2, 4, 4, QPen(expected_color), expected_color)
                expected_line.setToolTip(tip)
                expected_dot.setToolTip(tip)
                expected_line.setZValue(34)
                expected_dot.setZValue(35)
                expected_line.setVisible(self.layers.get("dt35", True))
                expected_dot.setVisible(self.layers.get("dt35", True))
                self._dt35_items.extend([expected_line, expected_dot])

    def _dt35_tooltip(self, key: str, ray: dict[str, object]) -> str:
        target = str(ray.get("expected_target", "")) or "no_hit"
        target_type = str(ray.get("expected_target_type", "")) or "none"
        state = "ok" if bool(ray.get("correction_allowed", False)) else "skip"
        if bool(ray.get("corner_ambiguous", False)):
            state = "corner"
        elif target_type == "ignore":
            state = "ignore"
        expected = float(ray.get("expected_distance_cm", float("nan")))
        residual = float(ray.get("residual_cm", float("nan")))
        incidence = float(ray.get("incidence_deg", float("nan")))
        ray_yaw = float(ray.get("ray_yaw_deg", float("nan")))
        axis = self._dt35_axis(ray_yaw)
        return (
            f"{key} {state}\n"
            f"target={target} [{target_type}]\n"
            f"ray_yaw={ray_yaw:.1f}deg axis={axis}\n"
            f"expected={expected:.1f}cm residual={residual:.1f}cm incidence={incidence:.1f}deg"
        )

    def _dt35_axis(self, ray_yaw_deg: float) -> str:
        dx, dy = heading_vector_from_front_yaw(ray_yaw_deg)
        if abs(dx) >= 0.85 and abs(dy) < 0.5:
            return "world X"
        if abs(dy) >= 0.85 and abs(dx) < 0.5:
            return "world Y"
        return "world XY"

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
