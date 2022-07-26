import argparse
from collections import defaultdict
import cv2
from itertools import product
import json
import numpy
from pathlib import Path
from shutil import rmtree
import sys
import time

import rosbag
import rosnode
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, Image


# Required for operation
REQUIRED = {
    "nodes": ["/raw_images/image_proc"],
    # "nodes": ["/raw_images/image_proc", "/raw_images/image_view"],
    "timeout": 10,  # seconds
}


def downstream_images_done(directories, sequence):
    '''
    Checks that downstream images (rectified, disparity) have been written.
    '''
    saved = [
        largest_sequence(directories[DISP_01_DIR], suffix=".npy") == sequence,
        largest_sequence(directories[RECT_CAM0_DIR], suffix=".png") == sequence,
        largest_sequence(directories[RECT_CAM1_DIR], suffix=".png") == sequence,
    ]
    return all(saved)


def get_current_topics():
    return [t[0] for t in rospy.get_published_topics()]


def publish_messages(publishers, messages):

    rospy.sleep(1.0)  # TODO: Debug

    # Check that we have the same number of messages in each list
    num_messages = [len(message_list) for message_list in messages.values()]
    assert all(num_messages[0] == numpy.array(num_messages))
    num_messages = num_messages[0]

    for i in range(num_messages):
        print(f"{i} ", end="", flush=True)
        for topic in CAM_TOPICS:
            # The [1] is to get the message, not the timestamp
            publishers[topic].publish(messages[topic][i][1])
            rospy.sleep(0.1)  # TODO: Debug
        start_time = time.time()
        timeout = 4  # seconds
        # while not downstream_images_done(directories, i):
        if abs(time.time() - start_time) < timeout:
            # raise RuntimeError(
            #     f"Sequence {i} was published but downstream image files"
            #     " (rectified or disparity) never appeared."
            # )
            rospy.sleep(0.25)
            stop_if_dead()
    print("")


def wait_for_required():
    '''Helper function to wait for certain nodes and topics to show up.'''
    current_nodes = rosnode.get_node_names()
    current_topics = get_current_topics()
    start_time = time.time()
    while not all([n in current_nodes for n in REQUIRED["nodes"]]) \
          or not all([t in current_topics for t in REQUIRED["topics"]]):
        if abs(time.time() - start_time) > REQUIRED["timeout"]:
            raise RuntimeError(
                "We waited, but not all required nodes and topics showed up. Required nodes:\n"
                f"{sorted(REQUIRED['nodes'])}\nFound:\n{sorted(current_nodes)}\nRequired topics:\n"
                f"{sorted(REQUIRED['topics'])}\nFound:\n{sorted(current_topics)}\n"
            )
        rospy.sleep(0.25)
        current_nodes = rosnode.get_node_names()
        current_topics = get_current_topics()
        stop_if_dead()


def stop_if_dead():
    '''Check whether the user has hit control-c and exit if so.'''
    if rospy.is_shutdown():
        sys.exit(1)


def assert_files_good(directories):
    for dirname, suffix in ((CAM_INFO_01_DIR, ".json"),
                            (DISP_01_DIR,     ".npy"),
                            (DISP_01_VIS_DIR, ".png"),
                            (CAM_INFO_23_DIR, ".json"),
                            (DISP_23_DIR,     ".npy"),
                            (DISP_23_VIS_DIR, ".png"),
                            (RECT_CAM0_DIR,   ".png"),
                            (RECT_CAM1_DIR,   ".png"),
                            (RECT_CAM2_DIR,   ".png"),
                            (RECT_CAM3_DIR,   ".png")):
        # Hardcoded number of correct files
        files = [x for x in directories[dirname].glob(f"*{suffix}")]
        assert len(files) == 7, f"{dirname} couldn't find 7 *{suffix} files"
        # For now hard-code these shapes
        if suffix in (".npy", ".png"):
            for file in files:
                if suffix == ".npy":
                    assert numpy.load(file).shape == (2048, 2448), \
                        f"{file} failed shape test"
                else:
                    assert cv2.imread(str(file)).shape == (2048, 2448, 3), \
                        f"{file} failed shape test"


def main(bagfiles, topics):

    rospy.init_node("bag_reader")
    publishers = {
        "/raw_images/image_raw":   rospy.Publisher("/raw_images/image_raw",   Image,      queue_size=10),
        "/raw_images/camera_info": rospy.Publisher("/raw_images/camera_info", CameraInfo, queue_size=10),
    }
    wait_for_required()

    # # Necessary settings
    # path_prefix =    rospy.get_param("~path_prefix")
    # stage1_dirname = rospy.get_param("~stage1_extraction_dir")
    # run_profiling =  rospy.get_param("~stage1_run_profiling")

    for i, bagfile in enumerate(bagfiles):

        for topic in topics:
            print(f"Processing {i+1}: {bagfile.name} / {topic}")

            # Given the image topic, add the camera_info topic. We need these
            # two to be paired
            assert topic.split("/")[-1] == "image_raw", \
                   "For now, the process assumes we are using image_raw topic"
            extract_topics = [
                topic,
                topic.replace("image_raw", "camera_info")
            ]

            messages = defaultdict(list)
            for (topic,
                 message,
                 message_time) in rosbag.Bag(bagfile).read_messages(EXTRACT_TOPICS):
                messages[topic].append((message_time, message))

            # Check that the messages are ordered
            timing = numpy.array([
                [msg_tuple[0].to_time() for msg_tuple in messages[key]]
                for key in extract_topics
            ])
            assert numpy.all(numpy.abs(numpy.diff(timing, axis=1)) > 0)

            # For what we can't, run it through the publishing process
            publish_messages(publishers, messages)

            # Give a little time for images to finish writing (cv2.imread calls
            # were failing otherwise)
            rospy.sleep(1.0)

            # Check that we have everything we need
            # assert_files_good(directories)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extracts data from the vineyard bagfiles."
    )
    parser.add_argument(
        "--bagfiles",
        help="Give all bagfiles you want processed here. For example, you can"
             " use wildcards like 'python3 process_bags.py ROW9/*bag'",
    )
    parser.add_argument(
        "--topics",
        help="CSV file containing XXXX.",
    )
    # Only parse known args to ignore some ROS-generated stuff
    args, _ = parser.parse_known_args()
    # The bags/topics have been separated by ; to go through the roslaunch file
    main(
        bagfiles=[Path(bag) for bag in args.bagfiles.split(";")],
        topics=[_ for _ in args.topics.split(";")]
    )
