import argparse
import shutil
import zipfile
from pathlib import Path


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}


def extract_zip(zip_path: Path, extract_dir: Path) -> None:
    if not zip_path.exists():
        raise FileNotFoundError(f"Zip file not found: {zip_path}")

    extract_dir.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(zip_path, "r") as zip_ref:
        zip_ref.extractall(extract_dir)


def find_dataset_root(extracted_dir: Path) -> Path:
    """
    Roboflow exports usually contain data.yaml at the root.
    Sometimes there may be one nested folder, so we search for data.yaml.
    """
    candidates = list(extracted_dir.rglob("data.yaml"))

    if not candidates:
        raise RuntimeError(f"No data.yaml found inside {extracted_dir}")

    return candidates[0].parent


def copy_split(
    source_root: Path,
    target_root: Path,
    split_name: str,
    prefix: str,
) -> int:
    source_images = source_root / split_name / "images"
    source_labels = source_root / split_name / "labels"

    if not source_images.exists():
        return 0

    target_images = target_root / split_name / "images"
    target_labels = target_root / split_name / "labels"

    target_images.mkdir(parents=True, exist_ok=True)
    target_labels.mkdir(parents=True, exist_ok=True)

    copied_count = 0

    for image_path in sorted(source_images.iterdir()):
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        new_stem = f"{prefix}_{image_path.stem}"
        new_image_name = f"{new_stem}{image_path.suffix.lower()}"
        new_label_name = f"{new_stem}.txt"

        target_image_path = target_images / new_image_name
        target_label_path = target_labels / new_label_name

        shutil.copy2(image_path, target_image_path)

        source_label_path = source_labels / f"{image_path.stem}.txt"

        if source_label_path.exists():
            shutil.copy2(source_label_path, target_label_path)
        else:
            # Empty label file for negative samples.
            target_label_path.touch()

        copied_count += 1

    return copied_count


def write_data_yaml(target_root: Path) -> None:
    data_yaml = target_root / "data.yaml"

    data_yaml.write_text(
        "train: train/images\n"
        "val: valid/images\n"
        "test: test/images\n"
        "\n"
        "nc: 1\n"
        "names:\n"
        "  0: baby\n"
    )


def merge_exports(zip_paths: list[str], output_dir: str) -> None:
    output_root = Path(output_dir)
    temp_root = Path("data/tmp/roboflow_merge")

    if temp_root.exists():
        shutil.rmtree(temp_root)

    if output_root.exists():
        shutil.rmtree(output_root)

    output_root.mkdir(parents=True, exist_ok=True)

    total_copied = 0

    for index, zip_path_str in enumerate(zip_paths, start=1):
        zip_path = Path(zip_path_str)
        prefix = f"v{index}"

        extracted_dir = temp_root / prefix
        extract_zip(zip_path, extracted_dir)

        dataset_root = find_dataset_root(extracted_dir)

        print(f"\nProcessing {zip_path}")
        print(f"Detected dataset root: {dataset_root}")

        for split_name in ["train", "valid", "test"]:
            copied = copy_split(
                source_root=dataset_root,
                target_root=output_root,
                split_name=split_name,
                prefix=prefix,
            )

            print(f"  {split_name}: copied {copied} images")
            total_copied += copied

    write_data_yaml(output_root)

    if temp_root.exists():
        shutil.rmtree(temp_root)

    print("\nMerge completed.")
    print(f"Total copied images: {total_copied}")
    print(f"Merged dataset: {output_root}")
    print(f"Data config: {output_root / 'data.yaml'}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--zips",
        nargs="+",
        required=True,
        help="Paths to Roboflow YOLO export zip files",
    )
    parser.add_argument(
        "--output",
        default="data/yolo",
        help="Output directory for merged YOLO dataset",
    )
    args = parser.parse_args()

    merge_exports(
        zip_paths=args.zips,
        output_dir=args.output,
    )


if __name__ == "__main__":
    main()
