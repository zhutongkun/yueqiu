#!/usr/bin/env python3
"""Validate Moon dataset metadata and run optional local YOLOv5 inference."""

import argparse
import hashlib
import json
import os
import pathlib
import sys
import time
from collections import Counter


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
EXPECTED_CLASSES_CN = [
    '卫星',
    '空间站',
    '月坑',
    '月球车',
    '陨石',
    '地球',
    '月壤',
    '月球',
    '火箭',
    '宇航员',
]
EXPECTED_MODEL_SIZE = 14438056
EXPECTED_MODEL_SHA256 = 'ab953a754cc6ea68742d49ccf26d8b122bb771fe65aa6bd582761bdeaaa7da34'
IMAGE_SUFFIXES = frozenset(('.jpg', '.jpeg', '.png', '.bmp'))


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate_package(package_root):
    model_dir = package_root / 'models' / 'moon'
    model_path = model_dir / 'best.pt'
    classes_path = model_dir / 'classes.txt'
    metadata_path = model_dir / 'metadata.json'
    yolov5_root = package_root / 'third_party' / 'yolov5'

    if not model_path.is_file():
        raise RuntimeError('model missing: %s' % model_path)
    if model_path.stat().st_size != EXPECTED_MODEL_SIZE:
        raise RuntimeError('model size mismatch: %s' % model_path.stat().st_size)
    digest = sha256_file(model_path)
    if digest != EXPECTED_MODEL_SHA256:
        raise RuntimeError('model SHA256 mismatch: %s' % digest)

    classes = classes_path.read_text(encoding='utf-8').splitlines()
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if classes != EXPECTED_CLASSES or metadata.get('classes') != EXPECTED_CLASSES:
        raise RuntimeError('class order mismatch')
    for relative_path in (
        'models/common.py',
        'models/yolo.py',
        'utils/augmentations.py',
        'utils/general.py',
        'utils/torch_utils.py',
        'LICENSE',
    ):
        if not (yolov5_root / relative_path).is_file():
            raise RuntimeError('vendored YOLOv5 file missing: %s' % relative_path)
    if (yolov5_root / '.git').exists():
        raise RuntimeError('nested yolov5/.git must not be deployed')
    return {
        'model_path': str(model_path),
        'model_size_bytes': model_path.stat().st_size,
        'model_sha256': digest,
        'classes': classes,
        'yolov5_root': str(yolov5_root),
    }


def _resolve_manifest_asset(sample_root, relative_path):
    relative = pathlib.Path(relative_path)
    if relative.is_absolute():
        raise RuntimeError('manifest asset path must be relative: %s' % relative_path)
    root = sample_root.resolve()
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root)
    except ValueError:
        raise RuntimeError('manifest asset escapes sample directory: %s' % relative_path)
    if not candidate.is_file():
        raise RuntimeError('manifest asset missing: %s' % candidate)
    return candidate


def _validate_manifest_image(sample_root, entry):
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError('Pillow is required for sample validation: %s' % exc)

    path = _resolve_manifest_asset(sample_root, entry.get('file', ''))
    digest = sha256_file(path)
    if digest != entry.get('sha256'):
        raise RuntimeError('sample SHA256 mismatch: %s' % path)
    with Image.open(str(path)) as image:
        image.verify()
    with Image.open(str(path)) as image:
        expected_size = (int(entry.get('width', -1)), int(entry.get('height', -1)))
        if image.size != expected_size:
            raise RuntimeError(
                'sample dimensions mismatch: %s is %s, expected %s'
                % (path, image.size, expected_size)
            )
        if image.format != 'JPEG':
            raise RuntimeError('sample is not JPEG: %s' % path)
    return path, digest


def validate_sample_manifest(package_root, manifest_path=None):
    """Validate the tracked ten-class and negative inference sample set."""
    sample_root = package_root / 'models' / 'moon' / 'validation_samples'
    manifest_path = pathlib.Path(manifest_path) if manifest_path else sample_root / 'manifest.json'
    if not manifest_path.is_file():
        raise RuntimeError('sample manifest missing: %s' % manifest_path)
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('schema_version') != 1:
        raise RuntimeError('unsupported sample manifest schema')

    expected_class_rows = [
        {'id': class_id, 'name_en': name_en, 'name_cn': EXPECTED_CLASSES_CN[class_id]}
        for class_id, name_en in enumerate(EXPECTED_CLASSES)
    ]
    if manifest.get('classes') != expected_class_rows:
        raise RuntimeError('sample manifest class mapping mismatch')
    if float(manifest.get('confidence_threshold', -1.0)) != 0.70:
        raise RuntimeError('sample manifest confidence threshold must be 0.70')
    if float(manifest.get('iou_threshold', -1.0)) != 0.45:
        raise RuntimeError('sample manifest IoU threshold must be 0.45')

    positive_rows = manifest.get('positive_samples')
    negative_rows = manifest.get('negative_samples')
    if not isinstance(positive_rows, list) or not isinstance(negative_rows, list):
        raise RuntimeError('sample manifest entries must be lists')

    sample_by_class = {}
    seen_hashes = {}
    positive_summary = []
    for entry in positive_rows:
        class_id = entry.get('class_id')
        if class_id not in range(len(EXPECTED_CLASSES)):
            raise RuntimeError('positive sample class id out of range: %s' % class_id)
        if class_id in sample_by_class:
            raise RuntimeError('duplicate positive sample class id: %s' % class_id)
        if entry.get('class_name_en') != EXPECTED_CLASSES[class_id]:
            raise RuntimeError('positive sample class name mismatch: %s' % entry)
        label = entry.get('yolo_label')
        if not isinstance(label, list) or len(label) != 5 or label[0] != class_id:
            raise RuntimeError('positive sample YOLO label mismatch: %s' % entry)
        if not all(0.0 <= float(value) <= 1.0 for value in label[1:]):
            raise RuntimeError('positive sample YOLO coordinates out of range: %s' % entry)
        path, digest = _validate_manifest_image(sample_root, entry)
        if digest in seen_hashes:
            raise RuntimeError('duplicate tracked sample: %s and %s' % (path, seen_hashes[digest]))
        seen_hashes[digest] = path
        sample_by_class[class_id] = path
        positive_summary.append(
            {
                'class_id': class_id,
                'class_name_en': EXPECTED_CLASSES[class_id],
                'file': str(path),
                'sha256': digest,
            }
        )
    if set(sample_by_class) != set(range(len(EXPECTED_CLASSES))):
        raise RuntimeError('tracked samples do not cover exactly ten classes')

    negative_paths = []
    negative_summary = []
    for entry in negative_rows:
        if entry.get('expected_detection', 'missing') is not None:
            raise RuntimeError('negative sample must expect no detection: %s' % entry)
        path, digest = _validate_manifest_image(sample_root, entry)
        if digest in seen_hashes:
            raise RuntimeError('duplicate tracked sample: %s and %s' % (path, seen_hashes[digest]))
        seen_hashes[digest] = path
        negative_paths.append(path)
        negative_summary.append({'file': str(path), 'sha256': digest})
    required_negative_names = {'bus.jpg', 'zidane.jpg'}
    if not required_negative_names.issubset({path.name for path in negative_paths}):
        raise RuntimeError('tracked negatives must include bus.jpg and zidane.jpg')

    return (
        {
            'manifest_path': str(manifest_path.resolve()),
            'confidence_threshold': 0.70,
            'iou_threshold': 0.45,
            'positive_samples': positive_summary,
            'negative_samples': negative_summary,
        },
        sample_by_class,
        negative_paths,
    )


def read_label(path):
    rows = [row for row in path.read_text(encoding='utf-8').splitlines() if row.strip()]
    if not rows:
        raise RuntimeError('empty label: %s' % path)
    parsed = []
    for row in rows:
        fields = row.split()
        if len(fields) != 5:
            raise RuntimeError('invalid YOLO label row: %s' % path)
        class_id = int(fields[0])
        box = [float(value) for value in fields[1:]]
        if class_id not in range(len(EXPECTED_CLASSES)):
            raise RuntimeError('class id out of range: %s' % path)
        if not all(0.0 <= value <= 1.0 for value in box):
            raise RuntimeError('normalised box out of range: %s' % path)
        parsed.append((class_id, box))
    return parsed


def validate_dataset(dataset_root):
    try:
        from PIL import Image
    except ImportError as exc:
        raise RuntimeError('Pillow is required for dataset validation: %s' % exc)

    root_classes = dataset_root.parent / 'classes.txt'
    if root_classes.is_file():
        classes = root_classes.read_text(encoding='utf-8').splitlines()
        if classes != EXPECTED_CLASSES:
            raise RuntimeError('dataset classes.txt order mismatch')

    summary = {}
    image_hashes = {}
    sample_by_class = {}
    for split in ('train', 'val'):
        image_dir = dataset_root / 'images' / split
        label_dir = dataset_root / 'labels' / split
        images = sorted(image_dir.glob('*.jpg'))
        labels = sorted(label_dir.glob('*.txt'))
        if len(images) != len(labels):
            raise RuntimeError('%s image/label count mismatch' % split)
        class_counts = Counter()
        empty_labels = 0
        for image_path in images:
            label_path = label_dir / (image_path.stem + '.txt')
            if not label_path.is_file():
                raise RuntimeError('missing label: %s' % image_path)
            parsed = read_label(label_path)
            if not parsed:
                empty_labels += 1
            for class_id, _box in parsed:
                class_counts[class_id] += 1
                if split == 'val' and class_id not in sample_by_class:
                    sample_by_class[class_id] = image_path
            with Image.open(str(image_path)) as image:
                image.verify()
            with Image.open(str(image_path)) as image:
                if image.size != (640, 480) or image.format != 'JPEG':
                    raise RuntimeError('unexpected image format/size: %s' % image_path)
            digest = sha256_file(image_path)
            if digest in image_hashes:
                raise RuntimeError(
                    'duplicate or split leakage: %s and %s'
                    % (image_path, image_hashes[digest])
                )
            image_hashes[digest] = image_path
        orphan_labels = [
            path for path in labels if not (image_dir / (path.stem + '.jpg')).is_file()
        ]
        if orphan_labels:
            raise RuntimeError('orphan labels in %s: %s' % (split, orphan_labels[:3]))
        summary[split] = {
            'images': len(images),
            'labels': len(labels),
            'empty_labels': empty_labels,
            'class_counts': [class_counts[index] for index in range(10)],
        }
    test_images = list((dataset_root / 'images' / 'test').glob('*')) if (
        dataset_root / 'images' / 'test'
    ).is_dir() else []
    summary['test'] = {'images': len(test_images)}
    summary['total_unique_images'] = len(image_hashes)
    summary['validation_samples'] = {
        str(class_id): str(path) for class_id, path in sample_by_class.items()
    }
    if set(sample_by_class) != set(range(10)):
        raise RuntimeError('validation split does not contain every class')
    return summary, sample_by_class


def load_inference_runtime(package_root, model_path, device_setting):
    import cv2
    import numpy as np
    import torch

    yolov5_root = package_root / 'third_party' / 'yolov5'
    try:
        import ultralytics
        if not getattr(ultralytics, '__version__', ''):
            raise ImportError('ultralytics has no version metadata')
    except Exception as exc:
        raise RuntimeError(
            'a working ultralytics package is required; install it during setup '
            'with --no-deps. Runtime auto-install is forbidden: %s' % exc
        )
    os.environ['YOLOv5_AUTOINSTALL'] = 'false'
    sys.path.insert(0, str(yolov5_root))
    from models.common import DetectMultiBackend
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes
    from utils.torch_utils import select_device

    requested_device = device_setting
    if requested_device == 'auto':
        requested_device = '0' if torch.cuda.is_available() else 'cpu'
    device = select_device(requested_device)
    model = DetectMultiBackend(str(model_path), device=device, dnn=False, data=None, fp16=False)
    names = model.names
    if isinstance(names, dict):
        names = [names[index] for index in range(len(names))]
    else:
        names = list(names)
    if names != EXPECTED_CLASSES:
        raise RuntimeError('loaded model class mapping mismatch: %s' % names)
    stride = int(model.stride.max()) if hasattr(model.stride, 'max') else int(max(model.stride))
    model.warmup(imgsz=(1, 3, 640, 640))
    return {
        'cv2': cv2,
        'np': np,
        'torch': torch,
        'model': model,
        'device': device,
        'letterbox': letterbox,
        'nms': non_max_suppression,
        'scale_boxes': scale_boxes,
        'stride': stride,
    }


def infer_one(runtime, image_path, confidence_threshold, iou_threshold):
    cv2 = runtime['cv2']
    np = runtime['np']
    torch = runtime['torch']
    image = cv2.imread(str(image_path))
    if image is None:
        raise RuntimeError('OpenCV could not read %s' % image_path)
    prepared = runtime['letterbox'](
        image, new_shape=640, stride=runtime['stride'], auto=True
    )[0]
    prepared = prepared[:, :, ::-1].transpose(2, 0, 1)
    prepared = np.ascontiguousarray(prepared)
    tensor = torch.from_numpy(prepared).to(runtime['device']).float() / 255.0
    if tensor.ndimension() == 3:
        tensor = tensor.unsqueeze(0)
    started = time.perf_counter()
    with torch.no_grad():
        prediction = runtime['model'](tensor, augment=False, visualize=False)
        prediction = runtime['nms'](
            prediction,
            confidence_threshold,
            iou_threshold,
            classes=None,
            agnostic=False,
            max_det=20,
        )[0]
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    if prediction is None or not len(prediction):
        return {'class_id': None, 'confidence': 0.0, 'inference_ms': elapsed_ms}
    prediction[:, :4] = runtime['scale_boxes'](
        tensor.shape[2:], prediction[:, :4], image.shape
    ).round()
    best = max(prediction.tolist(), key=lambda row: row[4])
    return {
        'class_id': int(best[5]),
        'class_name_en': EXPECTED_CLASSES[int(best[5])],
        'confidence': float(best[4]),
        'inference_ms': elapsed_ms,
    }


def run_inference(
    package_root,
    sample_by_class,
    negative_paths,
    device,
    confidence_threshold=0.70,
    iou_threshold=0.45,
):
    model_path = package_root / 'models' / 'moon' / 'best.pt'
    runtime = load_inference_runtime(package_root, model_path, device)
    samples = []
    for class_id in range(10):
        image_path = sample_by_class[class_id]
        prediction = infer_one(
            runtime,
            image_path,
            confidence_threshold,
            iou_threshold,
        )
        prediction.update(
            {
                'image': str(image_path),
                'expected_class_id': class_id,
                'expected_class_name_en': EXPECTED_CLASSES[class_id],
            }
        )
        if prediction.get('class_id') != class_id:
            raise RuntimeError('sample inference mismatch: %s' % prediction)
        samples.append(prediction)

    negatives = []
    for image_path in sorted(negative_paths):
        prediction = infer_one(
            runtime,
            image_path,
            confidence_threshold,
            iou_threshold,
        )
        prediction['image'] = str(image_path)
        if prediction.get('class_id') is not None:
            negatives.append(prediction)
    if negatives:
        raise RuntimeError(
            'negative sample false positive at confidence %.2f: %s'
            % (confidence_threshold, negatives)
        )
    return {
        'status': 'passed',
        'device': str(runtime['device']),
        'confidence_threshold': confidence_threshold,
        'iou_threshold': iou_threshold,
        'class_samples': samples,
        'negative_false_positives_at_0_70': negatives,
    }


def collect_additional_negatives(negative_dir):
    if negative_dir is None:
        return []
    negative_dir = negative_dir.resolve()
    if not negative_dir.is_dir():
        raise RuntimeError('negative directory missing: %s' % negative_dir)
    return sorted(
        path
        for path in negative_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def parse_args():
    parser = argparse.ArgumentParser()
    default_package = pathlib.Path(__file__).resolve().parents[1] / 'src' / 'competition'
    parser.add_argument('--package-root', type=pathlib.Path, default=default_package)
    parser.add_argument('--dataset', type=pathlib.Path)
    parser.add_argument('--sample-manifest', type=pathlib.Path)
    parser.add_argument('--negative-dir', type=pathlib.Path)
    parser.add_argument('--device', default='auto')
    parser.add_argument('--run-inference', action='store_true')
    parser.add_argument('--require-inference', action='store_true')
    parser.add_argument('--require-cpu-inference', action='store_true')
    parser.add_argument('--require-gpu-inference', action='store_true')
    parser.add_argument('--output-json', type=pathlib.Path)
    return parser.parse_args()


def main():
    args = parse_args()
    package_root = args.package_root.resolve()
    report = {'package': validate_package(package_root)}
    sample_summary, samples, negative_paths = validate_sample_manifest(
        package_root,
        args.sample_manifest.resolve() if args.sample_manifest else None,
    )
    report['tracked_validation_samples'] = sample_summary
    if args.dataset is not None:
        dataset_summary, _dataset_samples = validate_dataset(args.dataset.resolve())
        report['dataset'] = dataset_summary

    negative_paths.extend(collect_additional_negatives(args.negative_dir))
    devices = []
    if args.run_inference or args.require_inference:
        devices.append(args.device)
    if args.require_cpu_inference:
        devices.append('cpu')
    if args.require_gpu_inference:
        devices.append('0')
    devices = list(dict.fromkeys(devices))

    exit_code = 0
    if devices:
        report['inference_runs'] = []
        for device in devices:
            try:
                inference_report = run_inference(
                    package_root,
                    samples,
                    negative_paths,
                    device,
                    confidence_threshold=sample_summary['confidence_threshold'],
                    iou_threshold=sample_summary['iou_threshold'],
                )
            except Exception as exc:
                inference_report = {
                    'status': 'failed',
                    'requested_device': device,
                    'reason': '%s: %s' % (type(exc).__name__, exc),
                }
                exit_code = 2
            report['inference_runs'].append(inference_report)
    else:
        report['inference'] = {
            'status': 'not_requested',
            'reason': 'use --require-inference, --require-cpu-inference, or --require-gpu-inference',
        }

    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(output + '\n', encoding='utf-8')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
