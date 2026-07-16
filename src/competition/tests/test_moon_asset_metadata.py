#!/usr/bin/env python3
import hashlib
import json
import pathlib
import unittest
import wave


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
MODEL_DIR = PACKAGE_ROOT / 'models' / 'moon'
EXPECTED_CLASSES = [
    'satellite',
    'space_station',
    'lunar_crater',
    'lunar_rover',
    'meteorite',
    'earth',
    'lunar_soil',
    'moon',
    'rocket',
    'astronaut',
]
EXPECTED_SHA256 = 'ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34'


class MoonAssetMetadataTests(unittest.TestCase):
    def test_best_weight_size_and_sha256(self):
        path = MODEL_DIR / 'best.pt'
        self.assertEqual(14438056, path.stat().st_size)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        self.assertEqual(EXPECTED_SHA256, digest)

    def test_class_files_preserve_competition_order(self):
        classes = (MODEL_DIR / 'classes.txt').read_text(encoding='utf-8').splitlines()
        metadata = json.loads((MODEL_DIR / 'metadata.json').read_text(encoding='utf-8'))
        self.assertEqual(EXPECTED_CLASSES, classes)
        self.assertEqual(EXPECTED_CLASSES, metadata['classes'])

    def test_vendored_yolov5_runtime_is_present_without_nested_git(self):
        root = PACKAGE_ROOT / 'third_party' / 'yolov5'
        for relative_path in (
            'LICENSE',
            'models/common.py',
            'models/yolo.py',
            'utils/augmentations.py',
            'utils/general.py',
            'utils/torch_utils.py',
        ):
            self.assertTrue((root / relative_path).is_file(), relative_path)
        self.assertFalse((root / '.git').exists())

    def test_offline_result_voice_assets_are_valid_pcm_wav(self):
        root = PACKAGE_ROOT / 'voice' / 'moon'
        paths = sorted(root.glob('*.wav'))
        self.assertEqual(16, len(paths))
        for path in paths:
            with wave.open(str(path), 'rb') as wav_file:
                self.assertEqual(1, wav_file.getnchannels(), path.name)
                self.assertEqual(2, wav_file.getsampwidth(), path.name)
                self.assertGreater(wav_file.getnframes(), 0, path.name)


if __name__ == '__main__':
    unittest.main()
