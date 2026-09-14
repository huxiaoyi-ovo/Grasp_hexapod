#!/usr/bin/env python3
"""Install JetPack 5.1 / Python 3.8 GPU libraries locally, without sudo."""
import concurrent.futures
import gzip
import hashlib
import json
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import urllib.request

BASE = 'https://repo.download.nvidia.com/jetson/common/'
PACKAGES = ('libnvinfer8', 'libnvinfer-plugin8', 'libnvonnxparsers8',
            'libnvparsers8', 'python3-libnvinfer', 'libcudnn8',
            'libcublas-11-4', 'cuda-cudart-11-4', 'cuda-nvrtc-11-4', 'libcudla-11-4')


def main():
    if platform.machine() != 'aarch64' or sys.version_info[:2] != (3, 8):
        raise SystemExit('This installer requires Jetson aarch64 and Python 3.8.')
    release = Path('/etc/nv_tegra_release').read_text()
    if '# R35 ' not in release:
        raise SystemExit('This runtime is for JetPack 5 / L4T R35 only.')
    ws = Path(__file__).resolve().parents[3]
    python = ws / '.venv/xiaolan-ort/bin/python3'
    if not python.exists():
        raise SystemExit('Create .venv/xiaolan-ort first (see README).')
    dest = ws / '.venv/jetson-gpu'
    (dest / 'debs').mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(BASE + 'dists/r35.6/main/binary-arm64/Packages.gz', timeout=60) as r:
        index = gzip.decompress(r.read()).decode()
    packages = {}
    for block in index.split('\n\n'):
        fields = dict(line.split(': ', 1) for line in block.splitlines()
                      if ': ' in line and not line.startswith(' '))
        if 'Package' in fields:
            packages[fields['Package']] = fields

    def download(name):
        entry = packages[name]
        path = dest / 'debs' / Path(entry['Filename']).name
        def valid():
            if not path.exists():
                return False
            h = hashlib.sha256()
            with path.open('rb') as f:
                for chunk in iter(lambda: f.read(1024*1024), b''):
                    h.update(chunk)
            return h.hexdigest() == entry['SHA256']
        if not valid():
            print('Downloading', name, entry['Version'], entry['Size'], 'bytes', flush=True)
            partial = path.with_suffix('.partial')
            with urllib.request.urlopen(BASE + entry['Filename'], timeout=120) as r, partial.open('wb') as f:
                while True:
                    data = r.read(1024*1024)
                    if not data:
                        break
                    f.write(data)
            partial.replace(path)
            if not valid():
                raise RuntimeError('SHA256 mismatch: ' + name)
        print('Verified', name, flush=True)
        return path

    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        paths = list(pool.map(download, PACKAGES))
    root = dest / 'root'
    root.mkdir(exist_ok=True)
    for path in paths:
        subprocess.run(['dpkg-deb', '-x', str(path), str(root)], check=True)
    (dest / 'packages.json').write_text(json.dumps({n: packages[n] for n in PACKAGES}, indent=2))
    libs = ':'.join(str(root / p) for p in (
        'usr/lib/aarch64-linux-gnu', 'usr/local/cuda-11.4/targets/aarch64-linux/lib'))
    bindings = str(root / 'usr/lib/python3.8/dist-packages')
    launcher = dest / 'bin/python3'
    launcher.parent.mkdir(exist_ok=True)
    launcher.write_text('#!/usr/bin/env bash\nset -e\n' +
        'export LD_LIBRARY_PATH=' + shlex.quote(libs) + '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n' +
        'export PYTHONPATH=' + shlex.quote(bindings) + '${PYTHONPATH:+:$PYTHONPATH}\n' +
        'exec ' + shlex.quote(str(python)) + ' "$@"\n')
    launcher.chmod(0o755)
    subprocess.run([str(launcher), '-c', 'import tensorrt as trt; print("TensorRT", trt.__version__)'], check=True)
    print('GPU runtime ready:', launcher)


if __name__ == '__main__':
    main()
