"""Best-effort pose publication; display failures never block serial control."""
import json
import math
import os
import time


class PosePublisher:
    def __init__(self, path, target=None, capture_angle=None):
        self.path = path
        self.target = target
        self.capture_angle = capture_angle
        self.last_write = -1
        self.pose = {}
        self.pose_time = 0
        self.state = {}
        self.remaining_s = None
        self.outcome_text = None
        self.neutral_enabled = False
        self.neutral_target = None
        self.trial_id = None
        self.trial_text = None

    def update(self, data):
        if not self.path:
            return
        if 'Pit' in data and 'Rol' in data:
            self.pose = dict(data)
            self.pose_time = time.time()
        if 'App' in data:
            self.state = dict(data)
        if 'NOn' in data:
            self.neutral_enabled = data['NOn'] == '1'
            if not self.neutral_enabled:
                self.neutral_target = None
        if 'NPit' in data and 'NRol' in data:
            candidate = [float(data['NPit']), float(data['NRol'])]
            if all(math.isfinite(v) for v in candidate):
                if 'NOn' not in data:
                    self.neutral_enabled = True  # FAST includes these only when enabled.
                if self.neutral_enabled:
                    self.neutral_target = candidate
                elif 'NOn' in data:
                    self.target = candidate
        if not self.pose or time.monotonic() - self.last_write < .1:
            return
        self.last_write = time.monotonic()
        payload = {'timestamp': self.pose_time, 'pitch': float(self.pose['Pit']),
                   'roll': float(self.pose['Rol']), 'bal': int(data.get('Bal', self.state.get('Bal', 0))),
                   'count_ms': (int(data.get('CountMs', self.state.get('CountMs', 0)))
                                if int(data.get('Bal', self.state.get('Bal', 0))) == 5 else 0),
                   'target': self.neutral_target if self.neutral_enabled and self.neutral_target else self.target,
                   'neutral_enabled': self.neutral_enabled,
                   'capture_angle': float(self.capture_angle if self.capture_angle is not None else self.state.get('CapDeg', 2)),
                   'remaining_s': self.remaining_s,
                   'outcome_text': self.outcome_text,
                   'trial_id': self.trial_id, 'trial_text': self.trial_text,
                   'speeds': [float(self.pose.get(f'V{i}', self.pose.get(f'S{i}', 0))) for i in (1, 2, 3)]}
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(json.dumps(payload), encoding='utf-8')
            os.replace(temporary, self.path)
        except OSError:
            pass


def gravity_from_degrees(pitch, roll):
    p, r = math.radians(pitch), math.radians(roll)
    return [-math.sin(r)*math.cos(p), math.sin(p), math.cos(r)*math.cos(p)]
