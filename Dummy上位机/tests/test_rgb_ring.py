"""Validate diffusion and continuous Qt raster output without robot hardware."""
import math
import unittest

from PySide6.QtCore import QCoreApplication, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QImage

APPLICATION = QApplication.instance() or QApplication([])

from app.ui.rgb_ring import RgbRing, _diffuse_colors


class RgbRingTests(unittest.TestCase):
    def setUp(self):
        self.ring = RgbRing()
        self.ring.resize(320, 320)

    def tearDown(self):
        self.ring.close()
        self.ring.deleteLater()
        APPLICATION.processEvents()

    def image(self):
        APPLICATION.processEvents()
        return self.ring.grab().toImage()

    def track_color(self, image, source_position, radius=.415):
        diameter = min(520., min(self.ring.width(), self.ring.height()) - 20.)
        angle = source_position * math.tau / 24 - math.pi / 2
        dpr = image.devicePixelRatio()
        x = round((self.ring.width() / 2 + math.cos(angle) * diameter * radius) * dpr)
        y = round((self.ring.height() / 2 + math.sin(angle) * diameter * radius) * dpr)
        color = image.pixelColor(x, y)
        return color.red(), color.green(), color.blue()

    def test_invalid_frames_leave_the_previous_frame_intact(self):
        self.ring.set_frame([(128, 0, 0)] * 24)
        before = self.image()
        invalid = (None, [], [(0, 0, 0)] * 23, [(0, 0)] * 24,
                   [(256, 0, 0)] * 24, [(-1, 0, 0)] * 24,
                   [(1.5, 0, 0)] * 24, [(True, 0, 0)] * 24)
        for frame in invalid:
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    self.ring.set_frame(frame)
                self.assertEqual(self.image(), before)

    def test_single_color_has_a_continuous_ring_without_lamp_gaps(self):
        self.ring.set_frame([(0, 0, 255)] * 24)
        for size in (180, 320, 640):
            self.ring.resize(size, size)
            for theme in ("light", "dark"):
                with self.subTest(theme=theme, size=size):
                    self.ring.set_theme(theme)
                    image = self.image()
                    # Sample both source centers and every intervening arc.
                    colors = [self.track_color(image, index / 12) for index in range(288)]
                    self.assertTrue(all(blue > 240 and blue - max(red, green) > 150
                                        for red, green, blue in colors))
                    for channel in range(3):
                        levels = [rgb[channel] for rgb in colors]
                        self.assertLessEqual(max(levels) - min(levels), 4)

    def test_frame_is_copied_and_pause_does_not_dim_it(self):
        colors = [[0, 100, 200] for _ in range(24)]
        self.ring.set_frame(colors)
        before = self.image()
        colors[0][0] = 255
        self.ring.set_active(False)
        self.assertEqual(self.image(), before)
        self.ring.set_active(True)
        self.assertEqual(self.image(), before)

    def test_black_frame_shows_gray_white_cover_in_both_themes(self):
        self.ring.resize(180, 180)
        self.ring.set_caption("已关闭", "乳白扩散罩")
        for theme in ("light", "dark"):
            with self.subTest(theme=theme):
                self.ring.set_theme(theme)
                image = self.image()
                colors = [self.track_color(image, index / 4) for index in range(96)]
                lower, upper = (150, 190) if theme == "light" else (50, 90)
                self.assertTrue(all(min(rgb) >= lower and max(rgb) <= upper and
                                    max(rgb) - min(rgb) <= 25 for rgb in colors), colors)
                self.assertIn("仅表示模拟方向", self.ring.toolTip())
                self.assertNotIn("顶部标记", self.ring.toolTip())
                self.assertEqual(self.ring.minimumSizeHint().width(), 180)
                self.assertEqual(self.ring.sizeHint().width(), 320)

    def test_theme_owns_background_corners_without_native_white_leaks(self):
        for theme, expected in (("light", "#f7fafc"), ("dark", "#142031")):
            self.ring.set_theme(theme)
            image = self.image()
            self.assertEqual(image.pixelColor(0, 0).name(), expected)
            self.assertEqual(image.pixelColor(image.width() - 1, image.height() - 1).name(),
                             expected)

    def test_ring_only_has_transparent_center_and_corners_and_no_caption(self):
        ring = RgbRing(ring_only=True)
        try:
            ring.setFixedSize(144, 144)
            ring.set_caption("这段文字不应绘制", "亮度 100%")
            for theme in ("light", "dark"):
                ring.set_theme(theme)
                for color in ((0, 0, 0), (0, 255, 0)):
                    ring.set_frame([color] * 24)
                    image = QImage(144, 144, QImage.Format.Format_ARGB32_Premultiplied)
                    image.fill(Qt.GlobalColor.transparent)
                    ring.render(image)
                    for point in ((0, 0), (143, 143), (72, 72), (55, 65), (90, 83)):
                        self.assertEqual(image.pixelColor(*point).alpha(), 0)
                    self.assertGreater(image.pixelColor(72, 20).alpha(), 200)
        finally:
            ring.close()
            ring.deleteLater()

    def test_white_light_fades_toward_cover_instead_of_black(self):
        for theme in ("light", "dark"):
            self.ring.set_theme(theme)
            levels = []
            for level in (0, 5, 20, 73, 152, 255):
                self.ring.set_frame([(level,) * 3] * 24)
                rgb = self.track_color(self.image(), 0)
                levels.append(sum(rgb))
            self.assertEqual(levels, sorted(set(levels)))
            self.assertGreater(levels[0], 450 if theme == "light" else 150)
            self.assertLess(levels[0], 600 if theme == "light" else 300)
            self.assertGreater(levels[-1] - levels[0], 90)

    def test_rainbow_is_blended_without_a_top_seam(self):
        # Wide color variation deliberately exercises interpolation at stop 0/1.
        colors = []
        for index in range(24):
            hue = index / 24 * 6
            segment, fraction = int(hue), hue - int(hue)
            colors.append(tuple(round(value) for value in
                                ((255, 255 * fraction, 0), (255 * (1 - fraction), 255, 0),
                                 (0, 255, 255 * fraction), (0, 255 * (1 - fraction), 255),
                                 (255 * fraction, 0, 255), (255, 0, 255 * (1 - fraction)))[segment]))
        self.ring.set_frame(colors)
        image = self.image()
        samples = [self.track_color(image, index / 12) for index in range(288)]
        largest_jump = max(max(abs(a - b) for a, b in zip(samples[index], samples[(index + 1) % 288]))
                           for index in range(288))
        self.assertLess(largest_jump, 14)
        self.assertGreater(len(set(samples)), 180)

class DiffusionTests(unittest.TestCase):
    def test_zero_and_uniform_color_are_preserved(self):
        for color in ((0, 0, 0), (20, 100, 240), (255, 255, 255)):
            scattered = _diffuse_colors([color] * 24)
            self.assertEqual(len(scattered), 288)
            for sample in scattered:
                for actual, expected in zip(sample, color):
                    self.assertAlmostEqual(actual, expected, places=10)

    def test_isolated_source_scatter_wraps_and_preserves_average_power(self):
        colors = [(0, 0, 0)] * 24
        colors[0] = (0, 255, 0)
        scattered = _diffuse_colors(colors)
        self.assertGreater(scattered[0][1], scattered[12][1])
        self.assertGreater(scattered[12][1], scattered[24][1])
        self.assertGreater(scattered[24][1], 1)
        self.assertAlmostEqual(scattered[12][1], scattered[-12][1])
        self.assertAlmostEqual(sum(rgb[1] for rgb in scattered) / len(scattered), 255 / 24)

    def test_rotating_tail_and_orientation_survive_scattering(self):
        colors = [(0, 10, 0)] * 24
        colors[0], colors[23], colors[22] = (0, 255, 0), (0, 192, 0), (0, 74, 0)
        scattered = _diffuse_colors(colors)
        self.assertGreater(scattered[-12][1], scattered[12][1])
        self.assertGreater(scattered[-24][1], scattered[24][1])
        rotated = _diffuse_colors(colors[-6:] + colors[:-6])
        for index in range(288):
            for actual, expected in zip(rotated[(index + 72) % 288], scattered[index]):
                self.assertAlmostEqual(actual, expected, places=10)
        ring = RgbRing()
        try:
            ring.resize(320, 320)
            ring.set_frame(colors)
            image = ring.grab().toImage()
            # Tail is counterclockwise from the top, matching source 23/22.
            dpr = image.devicePixelRatio()
            x = 160 - math.sin(math.pi / 12) * 124.5
            y = 160 - math.cos(math.pi / 12) * 124.5
            left = image.pixelColor(round(x * dpr), round(y * dpr))
            right = image.pixelColor(round((320 - x) * dpr), round(y * dpr))
            self.assertGreater(left.green() - left.red(), right.green() - right.red())
        finally:
            ring.close()
            ring.deleteLater()


if __name__ == "__main__":
    unittest.main()
