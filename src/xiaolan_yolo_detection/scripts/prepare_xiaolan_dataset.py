#!/usr/bin/env python3
"""Prepare paired YOLO splits; preview with --dry-run before writing.

Use --exclude-classes-txt only when classes.txt is labelImg metadata.
Existing source labels are never opened for writing.
"""
import argparse
import hashlib
import random
import shutil
from pathlib import Path

SPLITS = ('train', 'val', 'test')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def counts(n):
    weights = (7, 2, 1)
    result = [n * w // 10 for w in weights]
    order = sorted(range(3), key=lambda i: (-(n * weights[i] % 10), i))
    for i in order[:n - sum(result)]:
        result[i] += 1
    if n >= 3:
        for i in range(3):
            if result[i] == 0:
                donor = max(range(3), key=lambda j: result[j])
                result[donor] -= 1
                result[i] += 1
    return result


def prepare(root, dry_run=False, exclude_classes=False):
    root = root.expanduser().resolve()
    raw, source = root / 'raw', root / 'labels_raw'
    if not raw.is_dir():
        raise ValueError('raw directory missing')
    outputs = [root / kind / split for kind in ('images', 'labels') for split in SPLITS]
    for p in [raw, source, root / 'images', root / 'labels', root / 'xiaolan.yaml'] + outputs:
        if p.is_symlink():
            raise ValueError('Refusing symbolic link: {}'.format(p))
    images = sorted(p for p in raw.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if not images or any(not p.is_file() or p.is_symlink() for p in images):
        raise ValueError('No images, or unsupported image file type')
    if len({p.stem for p in images}) != len(images):
        raise ValueError('Duplicate image stems: labels would not be unique')
    existing = sorted(source.glob('*.txt'))
    if any(not p.is_file() or p.is_symlink() for p in existing):
        raise ValueError('Source labels must be regular files')
    before = {p: digest(p) for p in existing}
    image_hashes = {p.name: digest(p) for p in images}
    missing = [source / (p.stem + '.txt') for p in images if not (source / (p.stem + '.txt')).exists()]
    expected = {p.stem + '.txt' for p in images}
    metadata = {'classes.txt'} if exclude_classes and 'classes.txt' not in expected else set()
    actual = {p.name for p in existing} - metadata
    extras = actual - expected
    print('DRY-RUN' if dry_run else 'EXECUTE', flush=True)
    print('总图片数:', len(images))
    print('已存在对应标签数:', len(images) - len(missing))
    print('将创建空标签数:', len(missing))
    print('源目录全部 txt 数:', len(existing))
    print('排除的元数据:', sorted(metadata))
    for p in outputs:
        if p.exists() and not p.is_dir():
            raise ValueError('Output is not directory: {}'.format(p))
        entries = list(p.iterdir()) if p.exists() else []
        print('将清空生成目录: {} ({} 个旧条目)'.format(p, len(entries)))
    positive = [p for p in images if (source / (p.stem + '.txt')).exists() and (source / (p.stem + '.txt')).stat().st_size > 0]
    pos_names = {p.name for p in positive}
    negative = [p for p in images if p.name not in pos_names]
    random.seed(42)
    groups = {s: [] for s in SPLITS}
    for stratum in (positive, negative):
        random.shuffle(stratum)
        start = 0
        for split, n in zip(SPLITS, counts(len(stratum))):
            groups[split].extend(stratum[start:start + n])
            start += n
    if dry_run:
        print('补齐后标签数:', len(actual) + len(missing))
        if extras:
            raise ValueError('Label count/mapping mismatch; splitting blocked: {}'.format(sorted(extras)))
        for split, members in groups.items():
            pos = sum(p.name in pos_names for p in members)
            print('{}: total={} positive={} negative={}'.format(split, len(members), pos, len(members)-pos))
        return
    source.mkdir(parents=True, exist_ok=True)
    for p in missing:
        with p.open('xb'):
            pass
    labels = [p for p in source.glob('*.txt') if p.name not in metadata]
    nonempty = sum(p.stat().st_size > 0 for p in labels)
    print('新创建空标签数:', len(missing))
    print('非空标签数:', nonempty)
    print('空标签数:', len(labels) - nonempty)
    if len(labels) != len(images) or {p.name for p in labels} != expected:
        raise ValueError('图片数量/对应关系与 txt 不一致，停止划分；额外标签: {}'.format(sorted(extras)))
    for p in outputs:
        p.mkdir(parents=True, exist_ok=True)
        print('清空生成目录:', p, flush=True)
        for entry in p.iterdir():
            if entry.is_dir() and not entry.is_symlink():
                shutil.rmtree(entry)
            else:
                entry.unlink()
    seen = []
    for split, members in groups.items():
        for p in members:
            shutil.copy2(p, root / 'images' / split / p.name)
            shutil.copy2(source / (p.stem + '.txt'), root / 'labels' / split / (p.stem + '.txt'))
        dest_images = list((root / 'images' / split).iterdir())
        dest_labels = list((root / 'labels' / split).iterdir())
        if {p.stem for p in dest_images} != {p.stem for p in dest_labels} or len(dest_images) != len(dest_labels):
            raise ValueError('Unpaired output files')
        for p in dest_images:
            if digest(p) != image_hashes[p.name]:
                raise ValueError('Image copy differs: {}'.format(p))
            label = root / 'labels' / split / (p.stem + '.txt')
            if digest(label) != digest(source / label.name):
                raise ValueError('Label copy differs: {}'.format(label))
        seen.extend(p.name for p in dest_images)
    if len(seen) != len(set(seen)) or set(seen) != {p.name for p in images}:
        raise ValueError('Duplicate or missing split images')
    if any(digest(p) != h for p, h in before.items()):
        raise ValueError('Existing source label changed')
    if any(digest(p) != image_hashes[p.name] for p in images):
        raise ValueError('Source image changed')
    (root / 'xiaolan.yaml').write_text('path: {}\n\ntrain: images/train\nval: images/val\ntest: images/test\n\nnames:\n  0: xiaolan\n'.format(root), encoding='utf-8')
    print('\nTOTAL\nimages: {}\npositive: {}\nnegative: {}'.format(len(images), len(positive), len(negative)))
    for split, members in groups.items():
        pos = sum(p.name in pos_names for p in members)
        print('\n{}\ntotal: {}\npositive: {}\nnegative: {}'.format(split.upper(), len(members), pos, len(members)-pos))
    print('\nPASS: unique pairs, no duplicates or omissions, split sum matches total; source hashes unchanged.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home() / 'xiaolan_dataset')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--exclude-classes-txt', action='store_true', help='Exclude labelImg classes.txt metadata from label count')
    args = parser.parse_args()
    try:
        prepare(args.root, args.dry_run, args.exclude_classes_txt)
    except (ValueError, OSError) as exc:
        parser.exit(1, 'ERROR: {}\n'.format(exc))


if __name__ == '__main__':
    main()
