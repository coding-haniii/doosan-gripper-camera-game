"""Launch the existing mock card dispatcher with a working RGB camera."""

import argparse
from importlib.util import find_spec
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

import cv2
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
VIDEO_DEVICES = Path('/sys/class/video4linux')
OCR_RUNTIME = Path.home() / '.local/share/doosan-gripper-camera-game/ocr'


def realsense_rgb_indices(devices=VIDEO_DEVICES):
    indices = []
    for node in devices.glob('video[0-9]*'):
        try:
            index = int(node.name.removeprefix('video'))
            interface = (node / 'device' / 'interface').read_text().casefold()
        except (OSError, ValueError):
            continue
        if 'realsense' in interface and 'rgb' in interface:
            indices.append(index)
    return sorted(indices)


def can_read_rgb(index, settings):
    camera = cv2.VideoCapture(index, cv2.CAP_V4L2)
    try:
        if not camera.isOpened():
            return False
        for key, prop in (('width', cv2.CAP_PROP_FRAME_WIDTH), ('height', cv2.CAP_PROP_FRAME_HEIGHT)):
            if settings.get(key):
                camera.set(prop, settings[key])
        for _ in range(3):
            ok, frame = camera.read()
            if ok and frame is not None and frame.ndim == 3 and frame.shape[2] == 3:
                return True
        return False
    finally:
        camera.release()


def choose_camera(settings):
    for index in realsense_rgb_indices():
        if can_read_rgb(index, settings):
            print(f'[CAMERA] RealSense RGB: /dev/video{index}', flush=True)
            return index

    fallback = settings['index']
    if can_read_rgb(fallback, settings):
        print(f'[CAMERA] RealSense unavailable. Using configured camera: /dev/video{fallback}', flush=True)
        return fallback
    raise SystemExit(f'Could not read RealSense RGB or configured camera /dev/video{fallback}.')


def prepare_ocr():
    if find_spec('pytesseract') is None:
        python = OCR_RUNTIME / 'venv/bin/python'
        if python.is_file() and Path(sys.prefix) != python.parents[1]:
            os.execv(python, [str(python), '-B', str(Path(__file__).resolve()), *sys.argv[1:]])
        raise SystemExit('Missing pytesseract. Install the project requirements and pytesseract.')

    bundled = OCR_RUNTIME / 'usr/bin/tesseract'
    if not os.environ.get('TESSERACT_CMD') and not shutil.which('tesseract') and bundled.is_file():
        os.environ['TESSERACT_CMD'] = str(bundled)
        os.environ.setdefault('TESSDATA_PREFIX', str(OCR_RUNTIME / 'usr/share/tesseract-ocr/5/tessdata'))


def recognize_action_card(image, recognize, recognize_result):
    height, width = image.shape[:2]
    y, x = int(height * .32), int(width * .32)
    if not x or not y:
        return recognize(image)
    center = image[y:height-y, x:width-x]
    game, confidence, raw = recognize_result(center, languages='eng')
    if game == 'tictactoe' and confidence >= .65 and 'TIC' in re.findall('[A-Z]+', raw.upper()):
        print(f'[card] TIC recognized in center crop ({confidence:.0%})', flush=True)
        return game
    return recognize(image)


def dispatch(config_path):
    # The launcher is a file inside the game folder, so add the app's package root.
    sys.path.insert(0, str(PROJECT_ROOT))
    from robot_games import app, vision

    ocr = vision._tesseract()
    if ocr is None or 'eng' not in ocr.get_languages(config=''):
        raise SystemExit('Tesseract with English language data is required to read action cards.')

    original_recognize = app.recognize_card
    original_argv = sys.argv
    try:
        # Limit the centered TIC check to this launcher, leaving the shared dispatcher untouched.
        app.recognize_card = lambda image: recognize_action_card(
            image, original_recognize, vision.recognize_card_result,
        )
        sys.argv = [str(app.__file__), '--config', str(config_path), '--mode', 'mock', '--game', 'camera']
        app.main()
    finally:
        app.recognize_card = original_recognize
        sys.argv = original_argv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=PROJECT_ROOT / 'configs/default.yaml')
    args = parser.parse_args()
    prepare_ocr()
    with args.config.open(encoding='utf-8') as file:
        config = yaml.safe_load(file)
    config['camera']['index'] = choose_camera(config['camera'])

    with tempfile.TemporaryDirectory(prefix='robot-games-camera-') as folder:
        config_path = Path(folder) / 'camera.yaml'
        config_path.write_text(yaml.safe_dump(config), encoding='utf-8')
        dispatch(config_path)


if __name__ == '__main__':
    sys.dont_write_bytecode = True
    main()
