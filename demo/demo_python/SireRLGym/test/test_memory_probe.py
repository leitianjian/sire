"""Run directly with Python; no simulator, torch or training dependencies."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'memory_probe', Path(__file__).resolve().parents[1] / 'utils/memory_probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class MemoryProbeTest(unittest.TestCase):
    def test_snapshot_and_append(self):
        self.assertGreater(probe.process_memory_bytes()['rss_bytes'], 0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'memory.jsonl'
            for stage in ('before_rollout', 'after_update'):
                probe.write_memory_sample(path, 70, stage)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual([row['stage'] for row in rows],
                         ['before_rollout', 'after_update'])
        self.assertTrue(all(row['iteration'] == 70 for row in rows))
        self.assertTrue(all(row['rss_bytes'] > 0 for row in rows))


if __name__ == '__main__':
    unittest.main()
