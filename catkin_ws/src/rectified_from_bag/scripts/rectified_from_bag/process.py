import argparse
from pathlib import Path
import subprocess


def main(bagfiles, topics):
    print([
        "roslaunch",
        "process_2021",
        "stage1_extraction.launch",
        "bagfiles:=" + ";".join([str(bag.absolute()) for bag in bagfiles]),
        f"topic_file:=" + ";".join(topics),
    ])
    subprocess.check_call([
        "roslaunch",
        "process_2021",
        "stage1_extraction.launch",
        "bagfiles:=" + ";".join([str(bag.absolute()) for bag in bagfiles]),
        f"topic_file:=" + ";".join(topics),
    ])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extracts data from the vineyard bagfiles."
    )
    parser.add_argument(
        "bagfiles",
        help="Give all bagfiles you want processed here. For example, you can"
             " use wildcards like 'python3 process_bags.py ROW9/*bag'",
        nargs="+",
        type=Path,
    )
    parser.add_argument(
        "--topics",
        help="XXXXXXX.",
        nargs="+",
    )
    args = parser.parse_args()

    main(args.bagfiles, args.topic_file)
