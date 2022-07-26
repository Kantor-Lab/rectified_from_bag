import argparse
from collections import defaultdict
import cv2
import numpy
from pathlib import Path
from shutil import move, rmtree
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

# Reliable publish to these topics
IMAGE_RAW = "/raw_images/image_raw"
CAM_INFO = "/raw_images/camera_info"

# Place for images to live temporarily (referenced in the launch file)
TEMP_OUT = Path("/tmp/extraction/")


def wipe_and_mkdir(path):
    """Removes a directory (if it exists) and make it fresh"""
    assert not path.is_file()
    rmtree(path, ignore_errors=True)
    path.mkdir(parents=True)


def wait_for_required():
    '''Helper function to wait for certain nodes and topics to show up.'''
    current_nodes = rosnode.get_node_names()
    start_time = time.time()
    while not all([n in current_nodes for n in REQUIRED["nodes"]]):
        if abs(time.time() - start_time) > REQUIRED["timeout"]:
            raise RuntimeError(
                "We waited, but not all required nodes and topics showed up. Required nodes:\n"
                f"{sorted(REQUIRED['nodes'])}\nFound:\n{sorted(current_nodes)}\nRequired topics:\n"
                f"{sorted(REQUIRED['topics'])}\nFound:\n{sorted(current_topics)}\n"
            )
        rospy.sleep(0.25)
        current_nodes = rosnode.get_node_names()
        stop_if_dead()


def stop_if_dead():
    '''Check whether the user has hit control-c and exit if so.'''
    if rospy.is_shutdown():
        sys.exit(1)


def publish_messages(publishers, messages):

    rospy.sleep(1.0)  # TODO: Debug

    # Check that we have the same number of messages in each list
    num_messages = [len(message_list) for message_list in messages.values()]
    assert all(num_messages[0] == numpy.array(num_messages))
    num_messages = num_messages[0]

    for i in range(num_messages):
        print(f"{i} ", end="", flush=True)
        for topic in messages.keys():
            if topic.endswith("image_raw"):
                pub_topic = IMAGE_RAW
            elif topic.endswith("camera_info"):
                pub_topic = CAM_INFO
            else:
                raise NotImplementedError("Didn't match approved topics")
            # The [1] is to get the message, not the timestamp
            publishers[pub_topic].publish(messages[topic][i][1])
            rospy.sleep(0.1)  # TODO: Debug
        start_time = time.time()
        timeout = 4  # seconds
        while not downstream_images_done(i):
            if time.time() - start_time > timeout:
                raise RuntimeError(
                    f"Sequence {i} was published but downstream image files"
                    " (rectified) never appeared."
                )
            rospy.sleep(0.1)
            stop_if_dead()
    print("")


def downstream_images_done(sequence):
    '''Checks that output images (rectified) have been written.'''
    if len(list(TEMP_OUT.glob("*png"))) > sequence:
        return True
    else:
        return False


def assert_files_good(bagdir):
    subdirs = list(sorted(bagdir.glob("*/")))
    pairs = [subdirs[i:i+2] for i in range(0, len(subdirs), 2)]
    # Check that the pairs have the same number of images
    for d1, d2 in pairs:
        assert len(list(d1.glob("*png"))) == len(list(d2.glob("*png")))
    return pairs


def main(bagfiles):

    rospy.init_node("bag_reader")
    publishers = {
        IMAGE_RAW: rospy.Publisher(IMAGE_RAW, Image,      queue_size=10),
        CAM_INFO:  rospy.Publisher(CAM_INFO,  CameraInfo, queue_size=10),
    }
    wait_for_required()

    for i, bagfile in enumerate(bagfiles):

        # Make a sourceable final destination
        bagdir = Path("/home/workspace").joinpath(bagfile.name.replace(".bag", ""))
        wipe_and_mkdir(bagdir)

        topics = [
            topic for topic in rosbag.Bag(bagfile, "r").get_type_and_topic_info()[1].keys()
            if topic.endswith("image_raw")
        ]

        for topic in topics:
            print(f"Processing {i+1}: {bagfile.name}: {topic}")

            # Make a temporary repeatable output
            wipe_and_mkdir(TEMP_OUT)

            # Given the image topic, add the camera_info topic. We need these
            # two to be paired
            extract_topics = [topic,topic.replace("image_raw", "camera_info")]

            messages = defaultdict(list)
            for (topic,
                 message,
                 message_time) in rosbag.Bag(bagfile).read_messages(extract_topics):
                messages[topic].append((message_time, message))

            # Check that the messages are ordered
            timing = numpy.array([
                [msg_tuple[0].to_time() for msg_tuple in messages[key]]
                for key in extract_topics
            ])
            assert numpy.all(numpy.diff(timing, axis=1) > 0)

            # For what we can't, run it through the publishing process
            publish_messages(publishers, messages)

            # Give a little time for images to finish writing (cv2.imread calls
            # were failing otherwise)
            rospy.sleep(0.1)

            # Remove all *.ini files (part of image_saver, don't care about
            # them)
            for ini_file in TEMP_OUT.glob("*ini"):
                ini_file.unlink()
            # Make the image numbers consistent (also part of image_saver).
            # Without this the image numbers would count up across bagfiles.
            for i, impath in enumerate(sorted(TEMP_OUT.glob("*png"))):
                move(impath, TEMP_OUT.joinpath(f'image_{i:06d}.png'))

            # Move the temporary holding dir to its final destination
            topic_path = bagdir.joinpath(topic.strip("/").replace("/", "_"))
            move(TEMP_OUT, topic_path)

        # Check that we have everything we need
        pairs = assert_files_good(bagdir)

        # Call RAFT-Stereo on files
        import ipdb; ipdb.set_trace()
        pass


if __name__ == "__main__":
    main(bagfiles=[_ for _ in Path("/home/workspace/").glob("*bag")])
