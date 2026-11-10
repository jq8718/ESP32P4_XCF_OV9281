"""Exercise the decoded-frame image upload switch while CDC timing continues."""

import argparse
import json

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from app import DebugWindow


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    args = parser.parse_args()
    args.no_images = False
    args.auto_connect = False
    args.smoke_seconds = 0
    application = QApplication([])
    window = DebugWindow(args)
    window.show()
    observations = {}

    def turn_off():
        observations['before'] = [window.frames_received, window.timing_total]
        window.image_button.click()

    def turn_on():
        observations['off'] = [window.frames_received, window.timing_total]
        window.image_button.click()

    def finish():
        observations['after'] = [window.frames_received, window.timing_total]
        observations['upload_enabled'] = window.upload_images.isChecked()
        print(json.dumps(observations), flush=True)
        before, off, after = (observations[name] for name in ('before', 'off', 'after'))
        passed = (before[0] > 0 and off[0] <= before[0] + 1 and off[1] > before[1]
                  and after[0] > off[0] and observations['upload_enabled'])
        window.close()
        application.exit(0 if passed else 1)

    QTimer.singleShot(200, window.toggle_connection)
    QTimer.singleShot(3500, turn_off)
    QTimer.singleShot(6000, turn_on)
    QTimer.singleShot(9500, finish)
    return application.exec()


if __name__ == '__main__':
    raise SystemExit(main())
