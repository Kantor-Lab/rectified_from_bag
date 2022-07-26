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
    "nodes": ["/stage1_save_disparities",
              "/stage1_save_rectified",
              "/theia/cam0/image_proc",
              "/theia/cam1/image_proc"],
    "topics": ["/disparity_map_01", "/disparity_map_23"],
    "timeout": 10,  # seconds
}

# What to extract
IMAGE_TOPICS = [f"/theia/cam{i}/image_raw" for i in range(4)]
IMAGE_INFO_TOPICS = [f"/theia/cam{i}/camera_info" for i in range(4)]
CAM_TOPICS = IMAGE_TOPICS + IMAGE_INFO_TOPICS
EXTRACT_TOPICS = CAM_TOPICS

CAM_TYPES = {}
for _topic in IMAGE_TOPICS:
    CAM_TYPES[_topic] = Image
for _topic in IMAGE_INFO_TOPICS:
    CAM_TYPES[_topic] = CameraInfo


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


def publish_messages(publishers, directories, messages):

    rospy.sleep(1.0)  # TODO: Debug
    num_messages = len(messages[IMAGE_TOPICS[0]])

    for i in range(num_messages):
        print(f"{i} ", end="", flush=True)
        for topic in CAM_TOPICS:
            # The [1] is to get the message, not the timestamp
            publishers[topic].publish(messages[topic][i][1])
            rospy.sleep(0.1)  # TODO: Debug
        start_time = time.time()
        timeout = 40  # seconds
        while not downstream_images_done(directories, i):
            if abs(time.time() - start_time) > timeout:
                raise RuntimeError(
                    f"Sequence {i} was published but downstream image files"
                    " (rectified or disparity) never appeared."
                )
            rospy.sleep(0.25)
            stop_if_dead()
    print("")


def stop_if_dead():
    '''Check whether the user has hit control-c and exit if so.'''
    if rospy.is_shutdown():
        sys.exit(1)


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


def main(bagfiles, parameters, first_call):

    rospy.init_node("bag_reader")
    publishers = {
        topic: rospy.Publisher(topic, CAM_TYPES[topic], queue_size=10)
        for topic in IMAGE_INFO_TOPICS + IMAGE_TOPICS
    }
    wait_for_required()

    # Necessary settings
    path_prefix =    rospy.get_param("~path_prefix")
    stage1_dirname = rospy.get_param("~stage1_extraction_dir")
    run_profiling =  rospy.get_param("~stage1_run_profiling")

    # If set to true, we will produce profiling output on the code being run.
    # A snakeviz file will show up in path_prefix, with stage1_timestamp.snakeviz.
    # You can visualize snakeviz files with "snakeviz X.snakeviz"
    if run_profiling:
        from cProfile import Profile
        profiler = Profile()
        profiler.enable()

    for i, bagfile in enumerate(bagfiles):
        print(f"[Stage 1] Processing {i+1}: {bagfile.name}")

        directories = create_dirs(bag=bagfile.name,
                                  path_prefix=path_prefix,
                                  directory=stage1_dirname,
                                  set_params=True,
                                  parameters=parameters,
                                  # Wipe everything the first time through
                                  delete_shit=first_call,
                                  subdirs=[CAM_INFO_01_DIR,
                                           DISP_01_DIR,
                                           DISP_01_VIS_DIR,
                                           CAM_INFO_23_DIR,
                                           DISP_23_DIR,
                                           DISP_23_VIS_DIR,
                                           RECT_CAM0_DIR,
                                           RECT_CAM1_DIR,
                                           RECT_CAM2_DIR,
                                           RECT_CAM3_DIR,
                                           ])

        messages = defaultdict(list)
        for (topic,
             message,
             message_time) in rosbag.Bag(bagfile).read_messages(EXTRACT_TOPICS):
            messages[topic].append((message_time, message))

        # Check that we have all the images
        for key in CAM_TOPICS:
            # Brutal and hard-coded, but should be true for all our data
            assert len(messages[key]) == 7
        # For now, just check that the existing order has messages close enough
        # to each other
        timing = numpy.array([
            [msg_tuple[0].to_time() for msg_tuple in messages[key]]
            for key in CAM_TOPICS
        ])
        assert numpy.all(numpy.abs(numpy.diff(timing, axis=0)) < 0.05)

        # Save what we can
        save_available_info(directories, messages)

        # For what we can't, run it through the publishing process
        publish_messages(publishers, directories, messages)

        # Give a little time for images to finish writing (cv2.imread calls
        # were failing otherwise)
        rospy.sleep(1.0)

        # Check that we have everything we need
        assert_files_good(directories)

    if run_profiling:
        profiler.disable()
        path = Path(path_prefix).joinpath(f"stage1_{int(time.time() * 1e6)}.snakeviz")
        profiler.dump_stats(path)


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
        "--parameters",
        help="Should be a json-dumped string with the parameters used to run"
             " this extraction effort. Will be saved for traceability.",
    )
    parser.add_argument(
        "--first-call",
        help="Flag should be given True the first time extraction is called,"
             " then False. This is the mechanism to wipe the dir the first time"
             " and then add to it if there are swept parameters or something.",
    )
    # Only parse known args to ignore some ROS-generated stuff
    args, _ = parser.parse_known_args()
    # The bags have been separated by ; to go through the roslaunch file
    main(
        bagfiles=[Path(bag) for bag in args.bagfiles.split(";")],
        parameters=args.parameters,
        first_call=args.first_call=="True",
    )
