"""
Persistent baby/doll counting via simple IoU-based tracking.

A plain per-frame detection count resets to 0 whenever the object briefly
leaves the frame, and re-counts the same object every time it reappears.
BabyCounter instead keeps a running total of *unique* objects ever seen:
existing tracks are matched to new detections by IoU, unmatched detections
become new tracks (incrementing the total), and tracks that disappear are
kept alive for a grace period so a re-detected object is recognized as the
same one instead of a new one.
"""

from dataclasses import dataclass, field


def iou(box_a: tuple[float, float, float, float], box_b: tuple[float, float, float, float]) -> float:
    """
    Intersection-over-union of two (x1, y1, x2, y2) boxes.
    """
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union_area = area_a + area_b - inter_area

    if union_area <= 0:
        return 0.0

    return inter_area / union_area


@dataclass
class _Track:
    box: tuple[float, float, float, float]
    frames_since_seen: int = 0


@dataclass
class BabyCounter:
    """
    Tracks unique babies/dolls across frames and exposes a count that:
      - never resets to 0 just because a frame has no detections
      - does not increment when an already-tracked object is detected again
      - only increments when a detection cannot be matched to any track
        currently being remembered (i.e. a genuinely new object)

    iou_threshold:
        minimum overlap between a track's last box and a new detection
        for them to be considered the same object.
    max_frames_unseen:
        number of consecutive frames a track is kept alive without being
        matched before it is forgotten. While alive, a reappearing object
        is matched to it instead of creating a new track.
    """

    iou_threshold: float = 0.3
    max_frames_unseen: int = 30
    _tracks: list[_Track] = field(default_factory=list)
    _total_seen: int = 0

    def update(self, boxes: list[tuple[float, float, float, float]]) -> int:
        """
        Feed in the bounding boxes detected in the current frame and return
        the updated total count of unique objects seen so far.
        """
        unmatched_boxes = list(boxes)
        matched_tracks: set[int] = set()

        for box in boxes:
            best_track_idx = None
            best_iou = 0.0

            for idx, track in enumerate(self._tracks):
                if idx in matched_tracks:
                    continue

                score = iou(track.box, box)
                if score >= self.iou_threshold and score > best_iou:
                    best_iou = score
                    best_track_idx = idx

            if best_track_idx is not None:
                track = self._tracks[best_track_idx]
                track.box = box
                track.frames_since_seen = 0
                matched_tracks.add(best_track_idx)
                unmatched_boxes.remove(box)

        for box in unmatched_boxes:
            self._tracks.append(_Track(box=box))
            self._total_seen += 1

        for idx, track in enumerate(self._tracks):
            if idx not in matched_tracks:
                track.frames_since_seen += 1

        self._tracks = [t for t in self._tracks if t.frames_since_seen <= self.max_frames_unseen]

        return self._total_seen

    @property
    def count(self) -> int:
        """
        Current total count of unique objects seen so far. Read-only view
        of the running total without feeding in a new frame.
        """
        return self._total_seen

    def reset(self) -> None:
        """
        Explicitly clear all tracking state and the count, e.g. when
        starting a brand new session/video.
        """
        self._tracks = []
        self._total_seen = 0
