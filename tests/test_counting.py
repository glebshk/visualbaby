from src.drone_vision.inference.count_babies import BabyCounter, iou


def test_iou_identical_boxes_is_one():
    box = (0, 0, 10, 10)
    assert iou(box, box) == 1.0


def test_iou_disjoint_boxes_is_zero():
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


def test_count_increments_for_new_object():
    counter = BabyCounter()
    assert counter.update([(0, 0, 10, 10)]) == 1
    assert counter.update([(0, 0, 10, 10), (50, 50, 60, 60)]) == 2


def test_count_does_not_increment_for_same_object_across_frames():
    counter = BabyCounter()
    counter.update([(0, 0, 10, 10)])
    counter.update([(1, 1, 11, 11)])
    counter.update([(2, 2, 12, 12)])
    assert counter.count == 1


def test_count_does_not_reset_when_no_detections():
    counter = BabyCounter()
    counter.update([(0, 0, 10, 10)])
    assert counter.update([]) == 1
    assert counter.update([]) == 1
    assert counter.count == 1


def test_object_briefly_out_of_sight_is_not_recounted():
    counter = BabyCounter(max_frames_unseen=5)
    counter.update([(0, 0, 10, 10)])
    counter.update([])
    counter.update([])
    assert counter.update([(0, 0, 10, 10)]) == 1


def test_object_gone_too_long_is_counted_as_new_on_reappearance():
    counter = BabyCounter(max_frames_unseen=2)
    counter.update([(0, 0, 10, 10)])
    counter.update([])
    counter.update([])
    counter.update([])
    assert counter.update([(0, 0, 10, 10)]) == 2


def test_reset_clears_state():
    counter = BabyCounter()
    counter.update([(0, 0, 10, 10)])
    counter.reset()
    assert counter.count == 0
    assert counter.update([(0, 0, 10, 10)]) == 1
