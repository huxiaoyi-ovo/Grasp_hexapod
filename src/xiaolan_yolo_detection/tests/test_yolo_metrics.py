"""YOLO callback metrics count only frames replaced before consumption."""
import runpy
import threading
import unittest
from pathlib import Path

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                           'scripts/xiaolan_yolo_node.py'))
YoloNode = MODULE['YoloNode']


def callback_only_node():
    node = YoloNode.__new__(YoloNode)
    node.lock = threading.Lock()
    node.latest = None
    node.counter = 0
    node.consumed_counter = 0
    node.received = 0
    node.overwritten = 0
    return node


class YoloMetricsTest(unittest.TestCase):
    def test_only_unconsumed_latest_replacement_counts_as_overwritten(self):
        node = callback_only_node()
        node.callback(object())
        self.assertEqual(node.overwritten, 0)

        with node.lock:
            current = node.latest
            node.consumed_counter = current[0]
        node.callback(object())
        self.assertEqual(node.overwritten, 0)
        node.callback(object())
        self.assertEqual(node.overwritten, 1)

    def test_replacement_before_consumption_counts_slot_loss(self):
        node = callback_only_node()
        node.callback(object())
        node.callback(object())
        self.assertEqual(node.overwritten, 1)
        self.assertEqual(node.latest[0], 2)


if __name__ == '__main__':
    unittest.main()
