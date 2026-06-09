import argparse
import random
import shutil
from pathlib import Path


def split_yolo_dataset(dataset_dir: str, val_ratio: float, test_ratio: float, seed: int) -> None:
    dataset_path = Path(dataset_dir)

    train_images = dataset_path / "train" / "images"
    train_labels = dataset_path / "train" / "labels"

    valid_images = dataset_path / "valid" / "images"
    valid_labels = dataset_path / "valid" / "labels"

    test_images = dataset_path / "test" / "images"
    test_labels = dataset_path / "test" / "labels"

    if not train_images.exists():
        raise FileNotFoundError(f"Missing train images directory: {train_images}")

    if not train_labels.exists():
        raise FileNotFoundError(f"Missing train labels directory: {train_labels}")

    valid_images.mkdir(parents=True, exist_ok=True)
    valid_labels.mkdir(parents=True, exist_ok=True)
    test_images.mkdir(parents=True, exist_ok=True)
    test_labels.mkdir(parents=True, exist_ok=True)

    image_paths = sorted(
        list(train_images.glob("*.jpg"))
        + list(train_images.glob("*.jpeg"))
        + list(train_images.glob("*.png"))
    )

    if not image_paths:
        raise RuntimeError(f"No images found in {train_images}")

    random.seed(seed)
    random.shuffle(image_paths)

    total = len(image_paths)
    test_count = int(total * test_ratio)
    val_count = int(total * val_ratio)

    test_set = image_paths[:test_count]
    val_set = image_paths[test_count:test_count + val_count]

    move_pairs(test_set, train_images, train_labels, test_images, test_labels)
    move_pairs(val_set, train_images, train_labels, valid_images, valid_labels)

    write_data_yaml(dataset_path)

    print("Dataset split completed.")
    print(f"Total images before split: {total}")
    print(f"Moved to valid: {len(val_set)}")
    print(f"Moved to test: {len(test_set)}")
    print(f"Remaining in train: {count_images(train_images)}")
    print(f"Updated: {dataset_path / 'data.yaml'}")


def move_pairs(image_paths, src_images, src_labels, dst_images, dst_labels):
    for image_path in image_paths:
        label_path = src_labels / f"{image_path.stem}.txt"

        shutil.move(str(image_path), str(dst_images / image_path.name))

        if label_path.exists():
            shutil.move(str(label_path), str(dst_labels / label_path.name))
        else:
            # Negative sample without annotations.
            # Create an empty label file so YOLO knows the image has no objects.
            (dst_labels / f"{image_path.stem}.txt").touch()


def count_images(images_dir: Path) -> int:
    return sum(
        1
        for path in images_dir.iterdir()
        if path.suffix.lower() in [".jpg", ".jpeg", ".png"]
    )


def write_data_yaml(dataset_path: Path) -> None:
    data_yaml = dataset_path / "data.yaml"

    data_yaml.write_text(
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n"
        "\n"
        "nc: 1\n"
        "names:\n"
        "  0: baby\n"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data/yolo", help="Path to YOLO dataset directory")
    parser.add_argument("--val-ratio", type=float, default=0.1, help="Validation split ratio")
    parser.add_argument("--test-ratio", type=float, default=0.1, help="Test split ratio")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    split_yolo_dataset(
        dataset_dir=args.dataset,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
