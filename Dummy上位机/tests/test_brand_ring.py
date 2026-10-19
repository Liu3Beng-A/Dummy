"""Header light consumes model frames without inventing an animation source."""
import math
import unittest

from PySide6.QtCore import QCoreApplication, Qt, QTimer
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

from app.ui.brand_ring import BrandRing

APPLICATION = QApplication.instance() or QApplication([])


class BrandRingTests(unittest.TestCase):
    def setUp(self):
        self.ring = BrandRing()
        self.now = 0.

    def tearDown(self):
        self.ring.close()
        self.ring.deleteLater()
        APPLICATION.processEvents()

    def frame(self, colors, **metadata):
        self.ring.set_rgb_frame(colors, **metadata)
        self.now += .3
        self.ring.advance_rgb_frame(self.now)

    def image(self):
        image = QImage(self.ring.size(), QImage.Format.Format_ARGB32_Premultiplied)
        image.fill(Qt.GlobalColor.transparent)
        self.ring.render(image)
        return image

    def sample(self, image, source):
        side = self.ring.width()
        radius = side * 428 / 1254
        angle = source * math.tau / 24 - math.pi / 2
        return image.pixelColor(round(side / 2 + radius * math.cos(angle)),
                                round(side / 2 + radius * math.sin(angle)))

    def test_shared_frame_is_copied_and_invalid_input_is_atomic(self):
        colors = [[0, 128, 255] for _ in range(24)]
        self.frame(colors, light_on=True, effect_mode=16)
        colors[0][0] = 255
        previous = (self.ring.rgb_frame, self.ring.rgb_light_on, self.ring.rgb_effect_mode)
        self.assertEqual(previous[0][0], (0, 128, 255))
        for invalid in (None, [], [(0, 0, 0)] * 23, [(0, 0)] * 24,
                        [(256, 0, 0)] * 24, [(True, 0, 0)] * 24, [(1.5, 0, 0)] * 24):
            with self.assertRaises(ValueError):
                self.ring.set_rgb_frame(invalid, light_on=False)
            self.assertEqual((self.ring.rgb_frame, self.ring.rgb_light_on, self.ring.rgb_effect_mode), previous)
        for metadata in ({'light_on': 1}, {'effect_mode': True}, {'effect_mode': 17}):
            with self.assertRaises(ValueError):
                self.ring.set_rgb_frame([(0, 0, 0)] * 24, **metadata)
            self.assertEqual((self.ring.rgb_frame, self.ring.rgb_light_on, self.ring.rgb_effect_mode), previous)
        self.assertEqual(self.ring.findChildren(QTimer), [])

    def test_continuous_ring_preserves_size_transparency_and_light_theme_outline(self):
        for size in (56, 72):
            self.ring.setFixedSize(size, size)
            for theme in ('dark', 'light'):
                self.ring.set_theme(theme)
                for color in ((0, 0, 0), (0, 255, 0), (255, 255, 255)):
                    self.frame([color] * 24, light_on=color != (0, 0, 0))
                    image = self.image()
                    self.assertEqual(self.ring.width(), size)
                    for point in ((0, 0), (size-1, size-1), (size//2, size//2)):
                        self.assertEqual(image.pixelColor(*point).alpha(), 0)
                    for source in range(24):
                        self.assertEqual(self.sample(image, source).alpha(), 255)
                    if color == (0, 255, 0):
                        sample = self.sample(image, 0)
                        self.assertGreater(sample.green()-sample.red(), 180)
                    else:
                        sample = self.sample(image, 0)
                        self.assertLess(max(sample.getRgb()[:3])-min(sample.getRgb()[:3]), 4)

    def test_rotation_uses_clockwise_sources_and_blends_across_the_seam(self):
        for index in (0, 6, 12, 18, 23):
            frame = [(0, 0, 0)] * 24
            frame[index] = (255, 0, 0)
            self.frame(frame, light_on=True, effect_mode=8)
            image = self.image()
            head = self.sample(image, index)
            opposite = self.sample(image, (index+12)%24)
            self.assertGreater(head.red()-head.green(), 80)
            self.assertLess(abs(opposite.red()-opposite.green()), 4)
        image = self.image()
        before, after = self.sample(image, -.2), self.sample(image, .2)
        self.assertLess(max(abs(a-b) for a,b in zip(before.getRgb(),after.getRgb())), 55)

    def test_white_brightness_and_power_metadata_do_not_invent_colored_light(self):
        levels = []
        for value in (0, 5, 20, 73, 152, 255):
            self.frame([(value,)*3]*24, light_on=True)
            levels.append(self.sample(self.image(), 0).red())
        self.assertEqual(levels, sorted(set(levels)))
        self.frame([(0, 0, 0)]*24, light_on=True, effect_mode=3)
        trough = self.image()
        self.frame([(0, 0, 0)]*24, light_on=False)
        self.assertEqual(self.image(), trough)

    def test_master_tick_damps_continuously_and_snaps_only_after_a_pause(self):
        self.frame([(0, 0, 0)]*24)
        self.ring.set_rgb_frame([(255, 0, 0)]*24, light_on=True)
        self.ring.advance_rgb_frame(self.now+.03)
        first = self.ring._displayed_frame[0][0]
        self.assertGreater(first, 0)
        self.assertLess(first, 255)
        self.ring.set_rgb_frame([(255, 0, 0)]*24, light_on=True)
        self.ring.advance_rgb_frame(self.now+.06)
        self.assertGreater(self.ring._displayed_frame[0][0], first)
        self.ring.advance_rgb_frame(self.now+1.)
        self.assertEqual(self.ring._displayed_frame, [[255, 0, 0]]*24)
