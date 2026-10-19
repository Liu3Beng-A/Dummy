"""Small local vector icons; no files, downloads or UI side effects."""
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPen, QPixmap


def workstation_icon(name, color="#8593a6", size=18):
    pixmap = QPixmap(size * 2, size * 2)
    pixmap.setDevicePixelRatio(2)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.scale(size / 24, size / 24)
    painter.setPen(QPen(QColor(color), 1.6, Qt.PenStyle.SolidLine,
                        Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
    def line(x1, y1, x2, y2):
        painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))
    if name == "motion":
        for p in ((5,4,5,19),(5,19,20,19),(5,19,13,10),
                  (3,7,5,4),(7,7,5,4),(17,17,20,19),(17,21,20,19),
                  (9,11,13,10),(13,10,12,14)):
            line(*p)
    elif name == "points":
        for y in (6,12,18):
            painter.drawEllipse(QRectF(3,y-1,2,2)); line(9,y,21,y)
    elif name == "motor":
        for x,y in ((5,8),(12,16),(19,8)):
            line(x,3,x,y-3); line(x,y+3,x,21)
            painter.drawEllipse(QRectF(x-2.5,y-2.5,5,5))
    elif name == "gripper":
        for p in ((12,3,12,6),(5,6,19,6),(5,6,5,20),(19,6,19,20),
                  (5,20,9,20),(19,20,15,20)):
            line(*p)
    elif name == "rgb":
        pen = painter.pen()
        pen.setWidthF(2.5)
        painter.setPen(pen)
        painter.drawEllipse(QRectF(3,3,18,18))
    elif name == "settings":
        for y,x in ((6,9),(12,16),(18,7)):
            line(3,y,x-3,y);line(x+3,y,21,y)
            painter.drawEllipse(QRectF(x-2.5,y-2.5,5,5))
    elif name == "terminal":
        painter.drawRoundedRect(QRectF(3,4,18,16),2,2)
        line(6,8,9,11);line(9,11,6,14);line(12,15,17,15)
    elif name == "home":
        for p in ((3,11,12,3),(12,3,21,11),(6,9,6,21),(6,21,18,21),
                  (18,21,18,9),(10,21,10,15),(10,15,14,15),(14,15,14,21)):
            line(*p)
    elif name == "power":
        line(12,2,12,11)
        # Open at the top around the stem, matching the original power glyph.
        painter.drawArc(QRectF(3.5,3.5,17,17),135*16,270*16)
    elif name == "stop":
        painter.drawRoundedRect(QRectF(5,5,14,14),1,1)
    elif name == "chevron":
        line(6,9,12,15); line(12,15,18,9)
    elif name in ("refresh", "reset"):
        # Two separate curved strokes and arrowheads, as in the design's
        # refresh controls. Mirror the camera reset to indicate return.
        if name == "reset":
            painter.translate(24, 0)
            painter.scale(-1, 1)
        path = QPainterPath(QPointF(3, 11))
        path.cubicTo(3, 6.6, 7, 3, 11.5, 3)
        path.cubicTo(15.5, 3, 18, 5.2, 21, 8)
        painter.drawPath(path)
        line(21, 3, 21, 8); line(21, 8, 16, 8)
        path = QPainterPath(QPointF(21, 13))
        path.cubicTo(21, 17.4, 17, 21, 12.5, 21)
        path.cubicTo(8.5, 21, 6, 18.8, 3, 16)
        painter.drawPath(path)
        line(3, 21, 3, 16); line(3, 16, 8, 16)
    elif name == "send":
        for p in ((3,10,21,3),(21,3,14,21),(14,21,10,14),
                  (10,14,3,10),(10,14,21,3)):
            line(*p)
    elif name == "plus":
        line(12,4,12,20); line(4,12,20,12)
    elif name == "more":
        for x in (5,12,19):
            painter.drawEllipse(QRectF(x-1,11,2,2))
    elif name == "grid":
        painter.drawRoundedRect(QRectF(3,3,18,18),1,1)
        line(12,3,12,21);line(3,12,21,12)
    elif name == "axes":
        line(6,4,6,19);line(6,19,21,19);line(6,19,16,9)
    elif name == "layers":
        for offset in (0,5):
            for p in ((3,8,12,3),(12,3,21,8),(21,8,12,13),(12,13,3,8)):
                line(p[0],p[1]+offset,p[2],p[3]+offset)
    else:
        painter.drawEllipse(QRectF(4,4,16,16))
        line(12,7,12,12);line(12,12,16,14)
    painter.end()
    return QIcon(pixmap)
