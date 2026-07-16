#!/usr/bin/env python3
"""Tests for the tracked Moon model validation samples and strict runner."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
TOOLS_ROOT = REPOSITORY_ROOT / 'tools'
sys.path.insert(0, str(TOOLS_ROOT))

import validate_moon_assets as validator  # noqa: E402


PACKAGE_ROOT = REPOSITORY_ROOT / 'src' / 'competition'


class TrackedSampleValidationTests(unittest.TestCase):
    def test_tracked_manifest_covers_all_classes_and_required_negatives(self):
        summary, samples, negatives = validator.validate_sample_manifest(PACKAGE_ROOT)
        self.assertEqual(list(range(10)), sorted(samples))
        self.assertEqual(10, len(summary['positive_samples']))
        self.assertEqual({'bus.jpg', 'zidane.jpg'}, {path.name for path in negatives})
        self.assertEqual(0.70, summary['confidence_threshold'])
        self.assertEqual(0.45, summary['iou_threshold'])

    def test_manifest_class_mapping_is_exact(self):
        manifest_path = (
            PACKAGE_ROOT / 'models' / 'moon' / 'validation_samples' / 'manifest.json'
        )
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        self.assertEqual(
            validator.EXPECTED_CLASSES,
            [row['name_en'] for row in manifest['classes']],
        )
        self.assertEqual(
            validator.EXPECTED_CLASSES_CN,
            [row['name_cn'] for row in manifest['classes']],
        )

    def test_manifest_rejects_modified_class_order(self):
        manifest_path = (
            PACKAGE_ROOT / 'models' / 'moon' / 'validation_samples' / 'manifest.json'
        )
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        manifest['classes'][0], manifest['classes'][1] = (
            manifest['classes'][1],
            manifest['classes'][0],
        )
        with tempfile.TemporaryDirectory() as directory:
            modified = Path(directory) / 'manifest.json'
            modified.write_text(json.dumps(manifest), encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'class mapping mismatch'):
                validator.validate_sample_manifest(PACKAGE_ROOT, modified)

    def test_manifest_asset_cannot_escape_sample_directory(self):
        sample_root = PACKAGE_ROOT / 'models' / 'moon' / 'validation_samples'
        with self.assertRaisesRegex(RuntimeError, 'escapes sample directory'):
            validator._resolve_manifest_asset(sample_root, '../best.pt')


class StrictInferenceTests(unittest.TestCase):
    def setUp(self):
        _summary, self.samples, self.negatives = validator.validate_sample_manifest(
            PACKAGE_ROOT
        )

    @staticmethod
    def _prediction_for_filename(_runtime, image_path, confidence, iou):
        if confidence != 0.70 or iou != 0.45:
            raise AssertionError('competition thresholds were not preserved')
        name = Path(image_path).name
        if name[:2].isdigit():
            class_id = int(name[:2])
            return {
                'class_id': class_id,
                'class_name_en': validator.EXPECTED_CLASSES[class_id],
                'confidence': 0.90,
                'inference_ms': 1.0,
            }
        return {'class_id': None, 'confidence': 0.0, 'inference_ms': 1.0}

    def test_strict_inference_accepts_correct_positives_and_clean_negatives(self):
        with mock.patch.object(
            validator,
            'load_inference_runtime',
            return_value={'device': 'cpu'},
        ), mock.patch.object(
            validator,
            'infer_one',
            side_effect=self._prediction_for_filename,
        ):
            report = validator.run_inference(
                PACKAGE_ROOT,
                self.samples,
                self.negatives,
                'cpu',
            )
        self.assertEqual('passed', report['status'])
        self.assertEqual(10, len(report['class_samples']))
        self.assertEqual([], report['negative_false_positives_at_0_70'])

    def test_positive_class_mismatch_is_strict_failure(self):
        def mismatched(runtime, image_path, confidence, iou):
            prediction = self._prediction_for_filename(
                runtime, image_path, confidence, iou
            )
            if Path(image_path).name == '00_satellite.jpg':
                prediction['class_id'] = 1
            return prediction

        with mock.patch.object(
            validator,
            'load_inference_runtime',
            return_value={'device': 'cpu'},
        ), mock.patch.object(validator, 'infer_one', side_effect=mismatched):
            with self.assertRaisesRegex(RuntimeError, 'sample inference mismatch'):
                validator.run_inference(
                    PACKAGE_ROOT,
                    self.samples,
                    self.negatives,
                    'cpu',
                )

    def test_negative_false_positive_at_point_seven_is_strict_failure(self):
        def false_positive(runtime, image_path, confidence, iou):
            prediction = self._prediction_for_filename(
                runtime, image_path, confidence, iou
            )
            if Path(image_path).name == 'bus.jpg':
                return {
                    'class_id': 9,
                    'class_name_en': 'astronaut',
                    'confidence': 0.71,
                    'inference_ms': 1.0,
                }
            return prediction

        with mock.patch.object(
            validator,
            'load_inference_runtime',
            return_value={'device': 'cpu'},
        ), mock.patch.object(validator, 'infer_one', side_effect=false_positive):
            with self.assertRaisesRegex(
                RuntimeError,
                'negative sample false positive at confidence 0.70',
            ):
                validator.run_inference(
                    PACKAGE_ROOT,
                    self.samples,
                    self.negatives,
                    'cpu',
                )


if __name__ == '__main__':
    unittest.main()
