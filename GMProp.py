import sys
import math
import numpy as np
import cv2
from shapely.geometry import Polygon
from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import (QPen, QBrush, QColor, QImage, QPixmap, QPainterPath, QPainter, QTransform, QPalette)
from PyQt5.QtWidgets import (QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QGraphicsPathItem,
                             QToolBar, QAction, QFileDialog, QDockWidget, QWidget, QVBoxLayout, QLabel,
                             QMessageBox, QGraphicsPixmapItem, QStatusBar, QGraphicsEllipseItem,
                             QInputDialog, QGraphicsSimpleTextItem, QPushButton, QComboBox, QSpinBox, QHBoxLayout)


class DrawingView(QGraphicsView):
    def __init__(self, scene, parent=None):
        super(DrawingView, self).__init__(scene, parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setMouseTracking(True)

        # Grid configuration
        self.grid_size = 10  # smaller unit spacing
        self.grid_pen = QPen(QColor(220,220,220))
        self.grid_pen.setWidth(1)
        self.axis_pen = QPen(Qt.darkGray)
        self.axis_pen.setWidth(2)

        # Drawing state
        self.current_path = QPainterPath()
        self.current_polygon_points = []
        self.is_drawing = False
        self.shapes = []
        self.node_items = []
        self.all_nodes = []

        self.polygon_pen = QPen(QColor("blue"))
        self.polygon_pen.setWidth(2)
        self.polygon_brush = QBrush(QColor(100, 100, 255, 50))

        self.temp_line_pen = QPen(QColor("red"))
        self.temp_line_pen.setWidth(1)

        self.temp_line_item = None
        self.current_polygon_item = None

        self.shift_pressed = False
        self.fixed_line_mode = False
        self.highlighted_node_item = None
        self.snap_radius = 10

        # Snapping to grid
        self.snap_to_grid = False

        # Coordinate text item
        self.coord_text_item = QGraphicsSimpleTextItem()
        self.coord_text_item.setBrush(QColor("black"))
        self.coord_text_item.setZValue(9999)
        self.scene().addItem(self.coord_text_item)

        # Temporary length text (for showing segment lengths while drawing)
        self.length_text_item = QGraphicsSimpleTextItem()
        self.length_text_item.setBrush(QColor("black"))
        self.length_text_item.setZValue(9999)
        self.length_text_item.setVisible(False)
        self.scene().addItem(self.length_text_item)

        # Undo stack
        self.undo_stack = []

        # Pan and zoom 
        self.setDragMode(QGraphicsView.NoDrag)
        self.middle_mouse_pressed = False
        self.last_pan_point = None

        # Mode: light by default
        self.dark_mode = False

        # Shape library mode
        self.current_shape_mode = "Polygon"  # Can be Polygon, Rectangle, Circle, Line
        self.temp_shape_points = []  # used for rectangle, circle, line

    def setShapeMode(self, mode):
        self.current_shape_mode = mode
        # Reset any ongoing drawings
        self.is_drawing = False
        self.current_polygon_points.clear()
        self.current_path = QPainterPath()
        if self.current_polygon_item:
            self.scene().removeItem(self.current_polygon_item)
            self.current_polygon_item = None
        if self.temp_line_item:
            self.scene().removeItem(self.temp_line_item)
            self.temp_line_item = None
        self.temp_shape_points.clear()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Shift:
            self.shift_pressed = True
        elif event.key() == Qt.Key_Z and (event.modifiers() & Qt.ControlModifier):
            self.undo()
        else:
            super(DrawingView, self).keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Shift:
            if self.is_drawing and len(self.current_polygon_points) > 0:
                self.fixed_line_mode = True
            self.shift_pressed = False
        super(DrawingView, self).keyReleaseEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.middle_mouse_pressed = True
            self.last_pan_point = event.pos()
            return

        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())

            # Snap to grid if enabled
            if self.snap_to_grid:
                gx = round(scene_pos.x() / self.grid_size) * self.grid_size
                gy = round(scene_pos.y() / self.grid_size) * self.grid_size
                scene_pos = QPointF(gx, gy)

            snapped_pos = self.getNodeSnap(scene_pos)
            if snapped_pos is not None:
                scene_pos = snapped_pos

            # Handle shape modes
            if self.current_shape_mode == "Polygon":
                self.handlePolygonClick(scene_pos)
            elif self.current_shape_mode == "Rectangle":
                self.handleRectangleClick(scene_pos)
            elif self.current_shape_mode == "Circle":
                self.handleCircleClick(scene_pos)
            elif self.current_shape_mode == "Line":
                self.handleLineClick(scene_pos)

        super(DrawingView, self).mousePressEvent(event)

    def handlePolygonClick(self, scene_pos):
        if not self.is_drawing:
            self.startDrawingAt(scene_pos)
            self.undo_stack.append(("start_shape", scene_pos))
        else:
            # Check if closing polygon
            if len(self.current_polygon_points) > 2:
                first_point = self.current_polygon_points[0]
                if (scene_pos - first_point).manhattanLength() < 10:
                    self.finishPolygon()
                    return

            if self.fixed_line_mode:
                scene_pos = self.fixedLengthLine(scene_pos)
                self.fixed_line_mode = False

            self.current_polygon_points.append(scene_pos)
            self.current_path.lineTo(scene_pos)
            self.current_polygon_item.setPath(self.current_path)

            self.undo_stack.append(("add_point", scene_pos))

    def handleRectangleClick(self, scene_pos):
        # First click sets first corner, second click sets opposite corner
        if len(self.temp_shape_points) == 0:
            self.temp_shape_points.append(scene_pos)
            self.is_drawing = True
        else:
            # second click
            self.temp_shape_points.append(scene_pos)
            self.finishRectangle()
            self.is_drawing = False

    def handleCircleClick(self, scene_pos):
        # First click sets center, second click sets point on circumference
        if len(self.temp_shape_points) == 0:
            self.temp_shape_points.append(scene_pos)
            self.is_drawing = True
        else:
            # second click
            self.temp_shape_points.append(scene_pos)
            self.finishCircle()
            self.is_drawing = False

    def handleLineClick(self, scene_pos):
        # First click: start, second click: end
        if len(self.temp_shape_points) == 0:
            self.temp_shape_points.append(scene_pos)
            self.is_drawing = True
        else:
            self.temp_shape_points.append(scene_pos)
            self.finishLine()
            self.is_drawing = False

    def fixedLengthLine(self, scene_pos):
        last_point = self.current_polygon_points[-1]
        dx = scene_pos.x() - last_point.x()
        dy = scene_pos.y() - last_point.y()

        if abs(dx) > abs(dy):
            length, ok = QInputDialog.getDouble(self, "Line Length", "Enter line length:", abs(dx), 0.1, 1e6, 2)
            if ok:
                if dx < 0:
                    scene_pos.setX(last_point.x() - length)
                else:
                    scene_pos.setX(last_point.x() + length)
                scene_pos.setY(last_point.y())
        else:
            length, ok = QInputDialog.getDouble(self, "Line Length", "Enter line length:", abs(dy), 0.1, 1e6, 2)
            if ok:
                if dy < 0:
                    scene_pos.setY(last_point.y() - length)
                else:
                    scene_pos.setY(last_point.y() + length)
                scene_pos.setX(last_point.x())
        return scene_pos

    def mouseMoveEvent(self, event):
        if self.middle_mouse_pressed:
            delta = event.pos() - self.last_pan_point
            self.last_pan_point = event.pos()
            self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
            self.translate(-delta.x(), -delta.y())
            return

        super(DrawingView, self).mouseMoveEvent(event)
        scene_pos = self.mapToScene(event.pos())

        # Show coordinates
        self.showCoordinates(scene_pos)

        snapped_pos = self.getNodeSnap(scene_pos)
        if snapped_pos is not None:
            self.highlightNode(snapped_pos)
        else:
            self.unhighlightNode()

        self.updateTemporaryLine(scene_pos)

    def updateTemporaryLine(self, scene_pos):
        if not self.is_drawing:
            self.length_text_item.setVisible(False)
            return

        # Show segment length while drawing
        # Depends on shape mode
        start_point = None
        if self.current_shape_mode == "Polygon" and len(self.current_polygon_points) > 0:
            start_point = self.current_polygon_points[-1]
        elif self.current_shape_mode == "Rectangle" and len(self.temp_shape_points) > 0:
            start_point = self.temp_shape_points[0]
        elif self.current_shape_mode == "Circle" and len(self.temp_shape_points) > 0:
            start_point = self.temp_shape_points[0]
        elif self.current_shape_mode == "Line" and len(self.temp_shape_points) > 0:
            start_point = self.temp_shape_points[0]

        if start_point is not None:
            current_line_length = (scene_pos - start_point).manhattanLength()
            # If shift is pressed, adjust movement for polygon mode
            if self.shift_pressed and self.current_shape_mode == "Polygon":
                dx = scene_pos.x() - start_point.x()
                dy = scene_pos.y() - start_point.y()
                if abs(dx) > abs(dy):
                    scene_pos.setY(start_point.y())
                else:
                    scene_pos.setX(start_point.x())
                current_line_length = (scene_pos - start_point).manhattanLength()

            # Remove old temp line
            if self.temp_line_item:
                self.scene().removeItem(self.temp_line_item)
                self.temp_line_item = None

            # Draw new temp line
            line_path = QPainterPath(start_point)
            line_path.lineTo(scene_pos)
            self.temp_line_item = QGraphicsPathItem(line_path)
            self.temp_line_item.setPen(self.temp_line_pen)
            self.scene().addItem(self.temp_line_item)

            # Show length text
            dist = math.hypot(scene_pos.x() - start_point.x(), scene_pos.y() - start_point.y())
            self.length_text_item.setText(f"Length: {dist:.2f}")
            self.length_text_item.setPos((start_point.x() + scene_pos.x())/2 + 10, (start_point.y() + scene_pos.y())/2)
            self.length_text_item.setVisible(True)
        else:
            self.length_text_item.setVisible(False)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.middle_mouse_pressed = False
            self.last_pan_point = None
        super(DrawingView, self).mouseReleaseEvent(event)

    def wheelEvent(self, event):
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        factor = 1.1
        if event.angleDelta().y() < 0:
            factor = 1.0 / factor
        self.scale(factor, factor)

    def startDrawingAt(self, start_pos):
        self.is_drawing = True
        self.current_polygon_points = [start_pos]
        self.current_path = QPainterPath(start_pos)

        if self.current_polygon_item:
            self.scene().removeItem(self.current_polygon_item)
        self.current_polygon_item = QGraphicsPathItem()
        self.current_polygon_item.setPen(self.polygon_pen)
        self.current_polygon_item.setBrush(self.polygon_brush)
        self.scene().addItem(self.current_polygon_item)

    def finishPolygon(self):
        if self.is_drawing and len(self.current_polygon_points) > 2:
            self.current_path.closeSubpath()
            self.current_polygon_item.setPath(self.current_path)
            poly = self.current_polygon_points[:]
            self.shapes.append(poly)
            self.is_drawing = False
            self.addNodeMarkers(poly)
            self.undo_stack.append(("finish_shape", poly))

            self.current_polygon_points = []
            self.current_path = QPainterPath()
            self.current_polygon_item = None

            if self.temp_line_item:
                self.scene().removeItem(self.temp_line_item)
                self.temp_line_item = None

            self.length_text_item.setVisible(False)

    def finishRectangle(self):
        # Two points define a rectangle
        if len(self.temp_shape_points) == 2:
            p1 = self.temp_shape_points[0]
            p2 = self.temp_shape_points[1]
            rect_coords = [QPointF(p1.x(), p1.y()),
                           QPointF(p2.x(), p1.y()),
                           QPointF(p2.x(), p2.y()),
                           QPointF(p1.x(), p2.y())]
            self.shapes.append(rect_coords)
            self.addPolygonToScene(rect_coords)
            self.temp_shape_points.clear()

    def finishCircle(self):
        # Two points define a circle: first = center, second = point on circumference
        if len(self.temp_shape_points) == 2:
            center = self.temp_shape_points[0]
            perimeter = self.temp_shape_points[1]
            radius = math.hypot(perimeter.x() - center.x(), perimeter.y() - center.y())

            # Approximate circle as polygon
            circle_points = []
            num_segments = 64
            for i in range(num_segments):
                angle = 2*math.pi*i/num_segments
                x = center.x() + radius*math.cos(angle)
                y = center.y() + radius*math.sin(angle)
                circle_points.append(QPointF(x,y))
            self.shapes.append(circle_points)
            self.addPolygonToScene(circle_points)
            self.temp_shape_points.clear()

    def finishLine(self):
        # Two points define a line
        if len(self.temp_shape_points) == 2:
            line_coords = self.temp_shape_points
            self.shapes.append(line_coords)
            self.addPolygonToScene(line_coords, is_line=True)
            self.temp_shape_points.clear()

    def addPolygonToScene(self, coords, is_line=False):
        path = QPainterPath(coords[0])
        for p in coords[1:]:
            path.lineTo(p)
        if not is_line:
            path.closeSubpath()
        item = QGraphicsPathItem()
        item.setPen(self.polygon_pen)
        item.setBrush(self.polygon_brush if not is_line else Qt.NoBrush)
        self.scene().addItem(item)
        item.setPath(path)
        self.addNodeMarkers(coords)

        # Cleanup temporary line
        if self.temp_line_item:
            self.scene().removeItem(self.temp_line_item)
            self.temp_line_item = None
        self.length_text_item.setVisible(False)

    def addNodeMarkers(self, polygon_points):
        for pt in polygon_points:
            node = QGraphicsEllipseItem(pt.x()-3, pt.y()-3, 6, 6)
            node.setPen(QPen(Qt.black))
            node.setBrush(QBrush(Qt.red))
            self.scene().addItem(node)
            self.node_items.append(node)
            self.all_nodes.append(pt)

    def getNodeSnap(self, pos):
        for node_pos in self.all_nodes:
            if (pos - node_pos).manhattanLength() < self.snap_radius:
                return node_pos
        return None

    def highlightNode(self, pos):
        self.unhighlightNode()
        highlight_circle = QGraphicsEllipseItem(pos.x()-5, pos.y()-5, 10, 10)
        highlight_circle.setPen(QPen(Qt.green, 2))
        highlight_circle.setBrush(Qt.transparent)
        highlight_circle.setZValue(999)
        self.scene().addItem(highlight_circle)
        self.highlighted_node_item = highlight_circle

    def unhighlightNode(self):
        if self.highlighted_node_item:
            self.scene().removeItem(self.highlighted_node_item)
            self.highlighted_node_item = None

    def showCoordinates(self, pos):
        self.coord_text_item.setText(f"({pos.x():.2f}, {pos.y():.2f})")
        self.coord_text_item.setPos(pos.x()+10, pos.y()+10)
        if self.dark_mode:
            self.coord_text_item.setBrush(QColor("white"))
        else:
            self.coord_text_item.setBrush(QColor("black"))

    def undo(self):
        if not self.undo_stack:
            return
        action = self.undo_stack.pop()
        t = action[0]

        if t == "add_point":
            last_point = action[1]
            if self.is_drawing and self.current_polygon_points and self.current_polygon_points[-1] == last_point:
                self.current_polygon_points.pop()
                self.current_path = QPainterPath(self.current_polygon_points[0])
                for p in self.current_polygon_points[1:]:
                    self.current_path.lineTo(p)
                if self.current_polygon_item:
                    self.current_polygon_item.setPath(self.current_path)
                if self.temp_line_item:
                    self.scene().removeItem(self.temp_line_item)
                    self.temp_line_item = None
        elif t == "finish_shape":
            finished_points = action[1]
            if finished_points in self.shapes:
                self.shapes.remove(finished_points)
            self.rebuildScene()
        elif t == "start_shape":
            if self.is_drawing:
                self.is_drawing = False
                self.current_polygon_points = []
                self.current_path = QPainterPath()
                if self.current_polygon_item:
                    self.scene().removeItem(self.current_polygon_item)
                    self.current_polygon_item = None
                if self.temp_line_item:
                    self.scene().removeItem(self.temp_line_item)
                    self.temp_line_item = None

    def rebuildScene(self):
        self.scene().clear()

        # Re-create coord_text_item and length_text_item after clearing
        self.coord_text_item = QGraphicsSimpleTextItem()
        self.coord_text_item.setBrush(QColor("black"))
        self.coord_text_item.setZValue(9999)
        self.scene().addItem(self.coord_text_item)

        self.length_text_item = QGraphicsSimpleTextItem()
        self.length_text_item.setBrush(QColor("black"))
        self.length_text_item.setZValue(9999)
        self.length_text_item.setVisible(False)
        self.scene().addItem(self.length_text_item)

        self.node_items.clear()
        self.all_nodes.clear()

        for poly in self.shapes:
            path = QPainterPath(poly[0])
            for p in poly[1:]:
                path.lineTo(p)
            # Check if it is a line or polygon by checking if start and end differ
            if len(poly) > 2:
                path.closeSubpath()
                item = QGraphicsPathItem()
                item.setPen(self.polygon_pen)
                item.setBrush(self.polygon_brush)
                self.scene().addItem(item)
                item.setPath(path)
            else:
                item = QGraphicsPathItem()
                item.setPen(self.polygon_pen)
                item.setBrush(Qt.NoBrush)
                self.scene().addItem(item)
                item.setPath(path)
            self.addNodeMarkers(poly)

        parent = self.parent()
        if isinstance(parent, QMainWindow):
            parent.drawAxes()

    def drawBackground(self, painter, rect):
        if self.dark_mode:
            painter.fillRect(rect, QColor(30,30,30))  # dark background
            grid_color = QColor(80,80,80)
            axis_color = QColor(200,200,200)
        else:
            painter.fillRect(rect, Qt.white)
            grid_color = QColor(220,220,220)
            axis_color = Qt.darkGray

        self.grid_pen.setColor(grid_color)
        self.axis_pen.setColor(axis_color)

        left = int(math.floor(rect.left() / self.grid_size)*self.grid_size)
        top = int(math.floor(rect.top() / self.grid_size)*self.grid_size)

        painter.setPen(self.grid_pen)
        x = left
        while x < rect.right():
            painter.drawLine(int(x), int(rect.top()), int(x), int(rect.bottom()))
            x += self.grid_size

        y = top
        while y < rect.bottom():
            painter.drawLine(int(rect.left()), int(y), int(rect.right()), int(y))
            y += self.grid_size

        painter.setPen(self.axis_pen)
        painter.drawLine(int(rect.left()), 0, int(rect.right()), 0)
        painter.drawLine(0, int(rect.top()), 0, int(rect.bottom()))

        painter.setPen(QPen(Qt.red, 2))
        origin_size = 5
        painter.drawLine(-origin_size, 0, origin_size, 0)
        painter.drawLine(0, -origin_size, 0, origin_size)

    def setDarkMode(self, dark):
        self.dark_mode = dark
        self.viewport().update()
        # Update coordinate text color
        if self.dark_mode:
            self.coord_text_item.setBrush(QColor("white"))
            self.length_text_item.setBrush(QColor("white"))
        else:
            self.coord_text_item.setBrush(QColor("black"))
            self.length_text_item.setBrush(QColor("black"))

    def setSnapToGrid(self, snap):
        self.snap_to_grid = snap

    def setGridSize(self, size):
        self.grid_size = size
        self.viewport().update()


class PropertiesDock(QWidget):
    def __init__(self, parent=None):
        super(PropertiesDock, self).__init__(parent)
        self.current_polygon_points = None

        layout = QVBoxLayout()

        self.label_info = QLabel("No shape selected.")
        self.label_info.setWordWrap(True)

        self.save_button = QPushButton("Save as CSV")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.saveCSV)

        layout.addWidget(self.label_info)
        layout.addWidget(self.save_button)
        self.setLayout(layout)

        self.dark_mode = False

    def setProperties(self, polygon_points):
        self.current_polygon_points = polygon_points
        if not polygon_points:
            self.label_info.setText("No shape selected.")
            self.save_button.setEnabled(False)
            return

        coords = [(p.x(), p.y()) for p in polygon_points]
        polygon = Polygon(coords)

        area = polygon.area
        perimeter = polygon.length
        centroid = polygon.centroid
        bbox = polygon.bounds

        edges = list(zip(coords, coords[1:] + [coords[0]]))
        horizontal_count, vertical_count = 0, 0
        for (x1, y1), (x2, y2) in edges:
            dx = x2 - x1
            dy = y2 - y1
            angle = math.degrees(math.atan2(dy, dx))
            if abs(angle) < 5 or abs(angle - 180) < 5 or abs(angle + 180) < 5:
                horizontal_count += 1
            elif abs(angle - 90) < 5 or abs(angle + 90) < 5:
                vertical_count += 1

        Ix, Iy, Ixy = self.polygon_moments_of_inertia(coords)
        x_c, y_c = centroid.x, centroid.y
        area_signed = self.signed_area(coords)
        Ix_centroid = Ix - area_signed * (y_c**2)
        Iy_centroid = Iy - area_signed * (x_c**2)
        Ixy_centroid = Ixy - area_signed * x_c * y_c

        prop_text = (
            f"Area: {area:.4f}\n"
            f"Perimeter: {perimeter:.4f}\n"
            f"Centroid: ({centroid.x:.4f}, {centroid.y:.4f})\n"
            f"Bounding Box: {bbox}\n"
            f"Number of Vertices: {len(coords)}\n"
            f"Horizontal Edges: {horizontal_count}\n"
            f"Vertical Edges: {vertical_count}\n"
            f"Ix (about centroid): {Ix_centroid:.4f}\n"
            f"Iy (about centroid): {Iy_centroid:.4f}\n"
            f"Ixy (about centroid): {Ixy_centroid:.4f}"
        )
        self.label_info.setText(prop_text)
        self.save_button.setEnabled(True)
        self.updateColors()

    def saveCSV(self):
        if not self.current_polygon_points:
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Save Properties as CSV", "", "CSV Files (*.csv)")
        if filename:
            coords = [(p.x(), p.y()) for p in self.current_polygon_points]
            polygon = Polygon(coords)
            area = polygon.area
            perimeter = polygon.length
            centroid = polygon.centroid
            bbox = polygon.bounds

            edges = list(zip(coords, coords[1:] + [coords[0]]))
            horizontal_count, vertical_count = 0, 0
            for (x1, y1), (x2, y2) in edges:
                dx = x2 - x1
                dy = y2 - y1
                angle = math.degrees(math.atan2(dy, dx))
                if abs(angle) < 5 or abs(angle - 180) < 5 or abs(angle + 180) < 5:
                    horizontal_count += 1
                elif abs(angle - 90) < 5 or abs(angle + 90) < 5:
                    vertical_count += 1

            Ix, Iy, Ixy = self.polygon_moments_of_inertia(coords)
            x_c, y_c = centroid.x, centroid.y
            area_signed = self.signed_area(coords)
            Ix_centroid = Ix - area_signed * (y_c**2)
            Iy_centroid = Iy - area_signed * (x_c**2)
            Ixy_centroid = Ixy - area_signed * x_c * y_c

            with open(filename, 'w') as f:
                f.write("Property,Value\n")
                f.write(f"Area,{area}\n")
                f.write(f"Perimeter,{perimeter}\n")
                f.write(f"Centroid X,{centroid.x}\n")
                f.write(f"Centroid Y,{centroid.y}\n")
                f.write(f"BBox minx,{bbox[0]}\n")
                f.write(f"BBox miny,{bbox[1]}\n")
                f.write(f"BBox maxx,{bbox[2]}\n")
                f.write(f"BBox maxy,{bbox[3]}\n")
                f.write(f"Number of Vertices,{len(coords)}\n")
                f.write(f"Horizontal Edges,{horizontal_count}\n")
                f.write(f"Vertical Edges,{vertical_count}\n")
                f.write(f"Ix (centroid),{Ix_centroid}\n")
                f.write(f"Iy (centroid),{Iy_centroid}\n")
                f.write(f"Ixy (centroid),{Ixy_centroid}\n")

    def polygon_moments_of_inertia(self, coords):
        Ix = 0.0
        Iy = 0.0
        Ixy = 0.0
        n = len(coords)
        for i in range(n):
            x1, y1 = coords[i]
            x2, y2 = coords[(i + 1) % n]
            A = x1 * y2 - x2 * y1
            Ix += (y1**2 + y1*y2 + y2**2)*A
            Iy += (x1**2 + x1*x2 + x2**2)*A
            Ixy += (x1*y2 + 2*x1*y1 + 2*x2*y2 + x2*y1)*A
        area = self.signed_area(coords)
        Ix = Ix / 12.0
        Iy = Iy / 12.0
        Ixy = Ixy / 24.0
        return Ix, Iy, Ixy

    def signed_area(self, coords):
        A = 0
        n = len(coords)
        for i in range(n):
            x1, y1 = coords[i]
            x2, y2 = coords[(i + 1) % n]
            A += x1*y2 - x2*y1
        return A/2.0

    def setDarkMode(self, dark):
        self.dark_mode = dark
        self.updateColors()

    def updateColors(self):
        if self.dark_mode:
            self.label_info.setStyleSheet("QLabel { color: white; }")
            self.save_button.setStyleSheet("QPushButton { color: white; }")
        else:
            self.label_info.setStyleSheet("QLabel { color: black; }")
            self.save_button.setStyleSheet("")


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.setWindowTitle("Shape Drawing and Analysis Tool")

        self.scene = QGraphicsScene()
        self.view = DrawingView(self.scene)
        self.setCentralWidget(self.view)

        self.properties_dock = PropertiesDock()
        self.dock = QDockWidget("Shape Properties", self)
        self.dock.setWidget(self.properties_dock)
        self.dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dock)

        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

        self.toolbar = QToolBar("Main Toolbar")
        self.addToolBar(self.toolbar)

        self.shape_toolbar = QToolBar("Shape Library")
        self.addToolBar(self.shape_toolbar)

        self.initToolbar()
        self.initShapeToolbar()

        self.background_image_item = None
        self.statusbar.showMessage("Ready.")

        # Set initial scene rect around a smaller range
        self.scene.setSceneRect(-100,-100,200,200)
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

        self.drawAxes()

        # Default light mode
        self.dark_mode = False
        self.applyTheme()

    def initToolbar(self):
        new_action = QAction("New Shape (Polygon)", self)
        new_action.setStatusTip("Start drawing a new polygonal shape")
        new_action.triggered.connect(self.newShape)
        self.toolbar.addAction(new_action)

        finish_action = QAction("Finish Polygon", self)
        finish_action.setStatusTip("Finish the current polygon")
        finish_action.triggered.connect(self.finishShape)
        self.toolbar.addAction(finish_action)

        clear_action = QAction("Clear Canvas", self)
        clear_action.setStatusTip("Clear all shapes and background")
        clear_action.triggered.connect(self.clearCanvas)
        self.toolbar.addAction(clear_action)

        load_img_action = QAction("Load Background Image", self)
        load_img_action.setStatusTip("Load an image as background")
        load_img_action.triggered.connect(self.loadBackgroundImage)
        self.toolbar.addAction(load_img_action)

        save_img_action = QAction("Save to Image", self)
        save_img_action.setStatusTip("Export the canvas to an image file")
        save_img_action.triggered.connect(self.exportToImage)
        self.toolbar.addAction(save_img_action)

        analyze_action = QAction("Analyze Last Shape", self)
        analyze_action.setStatusTip("Show properties of the most recently completed shape")
        analyze_action.triggered.connect(self.analyzeLastShape)
        self.toolbar.addAction(analyze_action)

        # Grid Snap toggle
        snap_action = QAction("Snap to Grid", self)
        snap_action.setCheckable(True)
        snap_action.setChecked(False)
        snap_action.setStatusTip("Toggle snapping to nearest grid point")
        snap_action.triggered.connect(self.toggleSnap)
        self.toolbar.addAction(snap_action)

        # Dark mode toggle
        dark_mode_action = QAction("Dark Mode", self)
        dark_mode_action.setCheckable(True)
        dark_mode_action.setChecked(False)
        dark_mode_action.setStatusTip("Toggle dark/light mode")
        dark_mode_action.triggered.connect(self.toggleDarkMode)
        self.toolbar.addAction(dark_mode_action)

        exit_action = QAction("Exit", self)
        exit_action.setStatusTip("Exit the application")
        exit_action.triggered.connect(self.close)
        self.toolbar.addAction(exit_action)

    def initShapeToolbar(self):
        # Shape selection combo box
        shape_label = QLabel("Shape:")
        shape_combo = QComboBox()
        shape_combo.addItems(["Polygon", "Rectangle", "Circle", "Line"])
        shape_combo.currentTextChanged.connect(self.changeShapeMode)

        # Grid size adjust
        grid_label = QLabel("Grid Size:")
        grid_spin = QSpinBox()
        grid_spin.setRange(1, 1000)
        grid_spin.setValue(self.view.grid_size)
        grid_spin.valueChanged.connect(self.view.setGridSize)

        shape_layout = QHBoxLayout()
        shape_widget = QWidget()
        shape_layout.addWidget(shape_label)
        shape_layout.addWidget(shape_combo)
        shape_layout.addWidget(grid_label)
        shape_layout.addWidget(grid_spin)
        shape_widget.setLayout(shape_layout)

        self.shape_toolbar.addWidget(shape_widget)

    def changeShapeMode(self, mode):
        self.view.setShapeMode(mode)
        self.statusbar.showMessage(f"Shape mode changed to {mode}")

    def newShape(self):
        self.view.setShapeMode("Polygon")
        choice = QMessageBox.question(self, "Start New Polygon",
                                      "Do you want to pick start point by clicking on canvas?",
                                      QMessageBox.Yes | QMessageBox.No)
        if choice == QMessageBox.Yes:
            self.view.is_drawing = False
            self.view.current_polygon_points = []
            self.view.current_path = QPainterPath()
            if self.view.current_polygon_item:
                self.scene.removeItem(self.view.current_polygon_item)
                self.view.current_polygon_item = None
            if self.view.temp_line_item:
                self.scene.removeItem(self.view.temp_line_item)
                self.view.temp_line_item = None
            self.statusbar.showMessage("Click on the canvas to start drawing a new polygon.")
        else:
            x, ok1 = QInputDialog.getDouble(self, "Start X", "Enter X coordinate:", 0.0, -1e6, 1e6, 2)
            if not ok1:
                return
            y, ok2 = QInputDialog.getDouble(self, "Start Y", "Enter Y coordinate:", 0.0, -1e6, 1e6, 2)
            if not ok2:
                return
            self.view.is_drawing = False
            self.view.current_polygon_points = []
            self.view.current_path = QPainterPath()
            if self.view.current_polygon_item:
                self.scene.removeItem(self.view.current_polygon_item)
                self.view.current_polygon_item = None
            if self.view.temp_line_item:
                self.scene.removeItem(self.view.temp_line_item)
                self.view.temp_line_item = None
            self.view.startDrawingAt(QPointF(x,y))
            self.view.undo_stack.append(("start_shape", QPointF(x,y)))
            self.statusbar.showMessage("Started new polygon at given coordinates.")

    def finishShape(self):
        if self.view.current_shape_mode == "Polygon":
            self.view.finishPolygon()
            self.statusbar.showMessage("Polygon finished.")
        else:
            self.statusbar.showMessage("Not in polygon mode. Finish shape not applicable.")

    def clearCanvas(self):
        self.scene.clear()
        self.view.shapes.clear()
        self.view.is_drawing = False
        self.view.current_polygon_points = []
        self.view.current_path = QPainterPath()
        self.view.current_polygon_item = None
        self.view.node_items.clear()
        self.view.all_nodes.clear()
        self.background_image_item = None
        self.properties_dock.setProperties(None)
        self.statusbar.showMessage("Canvas cleared.")

        # Re-create coord and length items after clearing
        self.view.coord_text_item = QGraphicsSimpleTextItem()
        self.view.coord_text_item.setBrush(QColor("black"))
        self.view.coord_text_item.setZValue(9999)
        self.scene.addItem(self.view.coord_text_item)

        self.view.length_text_item = QGraphicsSimpleTextItem()
        self.view.length_text_item.setBrush(QColor("black"))
        self.view.length_text_item.setZValue(9999)
        self.view.length_text_item.setVisible(False)
        self.scene.addItem(self.view.length_text_item)

        self.scene.setSceneRect(-100,-100,200,200)
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
        self.drawAxes()

    def loadBackgroundImage(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Open Background Image", "", "Images (*.png *.jpg *.bmp)")
        if filename:
            img = cv2.imread(filename)
            if img is None:
                QMessageBox.warning(self, "Error", "Could not load image.")
                return

            height, width, channels = img.shape
            bytes_per_line = 3 * width
            qimg = QImage(img.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(qimg)

            if self.background_image_item:
                self.scene.removeItem(self.background_image_item)

            self.background_image_item = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(self.background_image_item)
            self.scene.setSceneRect(QRectF(0, 0, width, height))
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
            self.statusbar.showMessage("Background image loaded.")

    def exportToImage(self):
        if self.scene.items():
            filename, _ = QFileDialog.getSaveFileName(self, "Save Canvas As Image", "", "PNG Files (*.png);;JPEG Files (*.jpg)")
            if filename:
                rect = self.scene.sceneRect()
                image = QImage(int(rect.width()), int(rect.height()), QImage.Format_ARGB32)
                if self.view.dark_mode:
                    image.fill(QColor(30,30,30))
                else:
                    image.fill(Qt.white)
                painter = QPainter(image)
                self.scene.render(painter)
                painter.end()
                image.save(filename)
                self.statusbar.showMessage(f"Canvas exported to {filename}")
        else:
            QMessageBox.information(self, "No Content", "Nothing to save.")

    def analyzeLastShape(self):
        if not self.view.shapes:
            QMessageBox.information(self, "No Shapes", "No shape to analyze.")
            return
        last_shape_points = self.view.shapes[-1]
        self.properties_dock.setProperties(last_shape_points)
        self.statusbar.showMessage("Analyzed last shape.")

    def toggleSnap(self, checked):
        self.view.setSnapToGrid(checked)
        self.statusbar.showMessage("Snap to Grid: ON" if checked else "Snap to Grid: OFF")

    def toggleDarkMode(self, checked):
        self.dark_mode = checked
        self.view.setDarkMode(checked)
        self.properties_dock.setDarkMode(checked)
        self.applyTheme()
        self.statusbar.showMessage("Dark Mode: ON" if checked else "Dark Mode: OFF")

    def applyTheme(self):
        if self.dark_mode:
            # Dark palette
            palette = QPalette()
            palette.setColor(QPalette.Window, QColor(45,45,45))
            palette.setColor(QPalette.WindowText, Qt.white)
            palette.setColor(QPalette.Base, QColor(30,30,30))
            palette.setColor(QPalette.AlternateBase, QColor(45,45,45))
            palette.setColor(QPalette.ToolTipBase, Qt.white)
            palette.setColor(QPalette.ToolTipText, Qt.white)
            palette.setColor(QPalette.Text, Qt.white)
            palette.setColor(QPalette.Button, QColor(45,45,45))
            palette.setColor(QPalette.ButtonText, Qt.white)
            palette.setColor(QPalette.BrightText, Qt.red)
            palette.setColor(QPalette.Highlight, QColor(64,128,128))
            palette.setColor(QPalette.HighlightedText, Qt.black)

            QApplication.setPalette(palette)
            self.setStyleSheet("QToolBar {background: #2D2D2D;} QStatusBar {color: white;} QDockWidget {color: white;}")

        else:
            # Default light
            QApplication.setPalette(QApplication.style().standardPalette())
            self.setStyleSheet("")

    def drawAxes(self):
        self.view.viewport().update()

    def closeEvent(self, event):
        reply = QMessageBox.question(self, "Exit", "Are you sure you want to exit?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()
        else:
            event.ignore()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1200, 800)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
