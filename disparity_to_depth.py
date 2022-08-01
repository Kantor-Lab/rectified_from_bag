"""
Tool to take disparity matrics and output colorized point clouds. Requires a
series of inputs that you should know from calibration or the /*/camera_info
topic. Specifically, will treat all *.npy files in the input directory as
disparity images.
"""


import argparse
import cv2
import numpy
import open3d
from pathlib import Path


def main(indir, outdir, imagedir, baseline, focal, cx, cy, z_min, z_max):

    rows = None
    cols = None

    # TODO: Make an option not to give the image directory
    for disp_path, impath in zip(sorted(indir.glob("*.npy")),
                                 sorted(imagedir.glob("*"))):

        disparity = numpy.load(disp_path)

        # Make matrices where the (i, j) position of a pixel is captured in
        # the row (i) and column (j) of these two matrices
        if rows is None or cols is None:
            rows = numpy.ones(disparity.shape) * \
                   numpy.array(disparity.shape[0]).reshape((-1, 1)) - cy
            cols = numpy.ones(disparity.shape) * \
                   numpy.array(disparity.shape[1]) - cx

        # depth = focal length * baseline / disparity
        # TODO: Is abs(right) here?
        stub = baseline / numpy.abs(disparity)
        z = focal * stub
        # y = baseline * (i - cy) / disparity
        # x = baseline * (j - cx) / disparity
        y = stub * rows
        x = stub * cols

        # Get the depth filtered points
        mask = numpy.logical_and(z >= z_min, z <= z_max)
        points = numpy.vstack((z[mask], y[mask], x[mask])).T

        cloud = open3d.geometry.PointCloud()
        cloud.points = open3d.utility.Vector3dVector(points)
        # TODO: Fix the image size mismatch
        if impath is not None:
            print(f"imread(impath).shape: {imread(impath).shape}")
            print(f"mask.shape: {mask.shape}")
            print(f"disparity.shape: {disparity.shape}")
            print(f"points.shape: {points.shape}")
            print(f"imread(impath)[mask].shape: {imread(impath)[mask].shape}")
            cloud.colors = open3d.utility.Vector3dVector(imread(impath)[mask])

        save_path = outdir.joinpath(disp_path.name.replace(".npy", ".pcd"))
        cloud.write_point_cloud(str(save_path))


def imread(path):
    """Handle directories or images, return as 0-1 RGB values."""
    if path.is_file():
        impath = path
    else:
        images = [_ for _ in path.glob("*png")]
        assert len(images) == 1, f"Need one image only in {path}, not {len(images)}"
        impath = images[0]
    image = cv2.cvtColor(cv2.imread(str(impath)), cv2.COLOR_BGR2RGB)
    image = image / 255
    return image


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "-b", "--baseline",
        help="Distance in meters between the two stereo cameras.",
        required=True,
        type=float,
    )
    parser.add_argument(
        "-f", "--focal-length",
        help="Focal length of the camera (pixels) from the intrinsic matrix"
             " (from calibration).",
        required=True,
        type=float,
    )
    parser.add_argument(
        "-i", "--indir",
        help="Path to draw *.npy files from as disparity images.",
        required=True,
        type=Path,
    )
    parser.add_argument(
        "-I", "--imagedir",
        help="Path to draw *.png files from as the matching color images. The"
             " images can be there as files, or there as files within folders"
             " (as RAFT desires) but in both cases the files/folders need to"
             " sort in the same order as indir.",
        required=True,
        type=Path,
    )
    parser.add_argument(
        "--min-filter",
        help="Depth from camera (m) past which we start accepting points.",
        default=0.0,
        type=float,
    )
    parser.add_argument(
        "--max-filter",
        help="Depth from camera (m) past which we start reject points.",
        default=5.0,
        type=float,
    )
    parser.add_argument(
        "-o", "--outdir",
        help="Path to save colorized point cloud (pcd) files in.",
        required=True,
        type=Path,
    )
    parser.add_argument(
        "-x", "--center-x",
        help="Center value of image (pixels) in x from the intrinsic matrix"
             " (from calibration).",
        required=True,
        type=float,
    )
    parser.add_argument(
        "-y", "--center-y",
        help="Center value of image (pixels) in y from the intrinsic matrix"
             " (from calibration).",
        required=True,
        type=float,
    )
    args = parser.parse_args()

    assert args.baseline > 0, "Baseline should be positive"
    assert args.focal_length > 0, "Focal length should be positive"
    assert args.center_x > 0, "Center-x value should be positive"
    assert args.center_y > 0, "Center-y value should be positive"

    assert args.min_filter >= 0, "Min filter value should be positive"
    assert args.max_filter > args.min_filter, "Max filter should be larger than min"

    for directory in (args.indir, args.outdir, args.imagedir):
        assert directory.is_dir(), f"{directory} is not a directory"

    return args


if __name__ == "__main__":
    args = parse_args()
    main(indir=args.indir,
         outdir=args.outdir,
         imagedir=args.imagedir,
         baseline=args.baseline,
         focal=args.focal_length,
         cx=args.center_x,
         cy=args.center_y,
         z_min=args.min_filter,
         z_max=args.max_filter)
