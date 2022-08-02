# rectified_from_bag
Take in a set of rosbags and output rectified images.

## Docker pulling/starting/running

In order to get this docker image you can pull the image like so:
```
docker pull franzericschneider/raft-from-bag:v5
```

Then this is how you start a docker container (make sure you populate the folder name with the bagfiles):
```
sudo docker container run \
    -it \
    --gpus all \
    --rm \
    --name extract \
    -v /home/exouser/Documents/<folder-name>/:/home/workspace/ \
    franzericschneider/raft-from-bag:v5
```

When you're inside the docker container then just from `process` and it will process bags will the following assumptions:
1. All the bagfile have the `.bag` ending and are in `<folder-name>/`
2. The topics are named in the form `/.../.../image_raw`
3. L/R stereo pairs of topics are sorted alphabetically into L/R pairs
4. If there are multiple stereo pairs the topic pairs are grouped together alphabetically

For example, the topics `(/cam0, /cam1, /cam2, /cam3)` are assumed to have `(cam0, /cam1)` and `(/cam2, /cam3)` as L/R stereo pairs because of their sorting order. This is a pretty harsh assumption, if it violated in the future we could fix that.

After running things, when you're done with the docker container, run `exit`.

If you have the container open in one terminal, you can open a second terminal like so:
```
sudo docker exec -it extract /bin/bash
```

## Permissions

Docker by default claims its created files as root, and I don't want to keep messing with Docker to make that not be the case. Instead, do a little post-processing. For starters, the first time you use a system you can place this function in your `~/.bash_aliases` file.

```
function claim() {
    sudo chmod -R 755 $1;
    sudo chown -R $USER $1;
    sudo chgrp -R $USER $1;
}
```

Then, after a `process` run, from outside the docker container you can run `claim /home/exouser/Documents/<folder-name>/`, or wherever your bag files were placed.

## Flattening images

Another annoyance is that RAFT wants to consume the images in a stacked folder structure. In order to unflatten the images, if you want to, you can `cd` into a folder with images and run `flatten` after pasting this code (once) into the `~/.bash_aliases` file:

```
alias flatten='for dirr in */; do mv "$dirr"image.png "${dirr::-1}".png; rm -r "$dirr"; done'
```

## Cleaning unwanted `npy`s

If you are only interested in visualization and don't need the `npy` files which contain the depth data, you can remove all `npy` files recursively from a directory by running `recurse_rm_npy` after pasting this code (once) into the `~/.bash_aliases` file:

```
alias recurse_rm_npy='find . -type f -name "*.npy" -delete'
```

## Getting depth from disparity files

There is a file called `disparity_from_depth.py`. You can get it into the Jetstream instance in a number of ways, but one easy one is to go to this link on the Jetstream: https://raw.githubusercontent.com/Kantor-Lab/rectified_from_bag/main/disparity_to_depth.py, then right click and hit "Save Page As...", then you can save it in `Downloads/`. The file takes a number of arguments, which you can also see by running `python disparity_from_depth.py --help`. Basically you need to know your baseline / focal length / pixel centers from intrinsic calibration in order to get depth. Then just specify the input/output folders.

```
usage: disparity_to_depth.py [-h] -b BASELINE -f FOCAL_LENGTH -i INDIR [-I IMAGEDIR]
                             [--min-filter MIN_FILTER] [--max-filter MAX_FILTER] -o OUTDIR -x
                             CENTER_X -y CENTER_Y

Tool to take disparity matrics and output colorized point clouds. Requires a series of inputs that
you should know from calibration or the /*/camera_info topic. Specifically, will treat all *.npy
files in the input directory as disparity images.

optional arguments:
  -h, --help            show this help message and exit
  -b BASELINE, --baseline BASELINE
                        Distance in meters between the two stereo cameras. (default: None)
  -f FOCAL_LENGTH, --focal-length FOCAL_LENGTH
                        Focal length of the camera (pixels) from the intrinsic matrix (from
                        calibration). (default: None)
  -i INDIR, --indir INDIR
                        Path to draw *.npy files from as disparity images. (default: None)
  -I IMAGEDIR, --imagedir IMAGEDIR
                        Path to draw *.png files from as the matching color images. The images can
                        be there as files, or there as files within folders (as RAFT desires) but
                        in both cases the files/folders need to sort in the same order as indir.
                        You can leave this out and the point cloud will just be uncolored.
                        (default: None)
  --min-filter MIN_FILTER
                        Depth from camera (m) past which we start accepting points. (default: 0.0)
  --max-filter MAX_FILTER
                        Depth from camera (m) past which we start reject points. (default: 5.0)
  -o OUTDIR, --outdir OUTDIR
                        Path to save colorized point cloud (ply) files in. (default: None)
  -x CENTER_X, --center-x CENTER_X
                        Center value of image (pixels) in x from the intrinsic matrix (from
                        calibration). (default: None)
  -y CENTER_Y, --center-y CENTER_Y
                        Center value of image (pixels) in y from the intrinsic matrix (from
                        calibration). (default: None)
```

#### Example of using this tool on Jetstream

Note that we are in the `~/Downloads/` folder, where I saved the file.

```
(base) exouser@suitably-promoted-cardinal:~/Downloads$ python disparity_to_depth.py \
    --baseline 0.10289 \
    --focal-length 1721.12 \
    --center-x 707.17 \
    --center-y 496.39 \
    --indir ~/Downloads/0727_Soybean_Trail1/2022-07-27-14-47-48/stereo_00/ \
    --outdir ~/Downloads/0727_Soybean_Trail1/2022-07-27-14-47-48/clouds/ \
    --imagedir ~/Downloads/0727_Soybean_Trail1/2022-07-27-14-47-48/theia_left_image_raw/
(base) exouser@suitably-promoted-cardinal:~/Downloads$ ls ~/Downloads/0727_Soybean_Trail1/2022-07-27-14-47-48/clouds/
image_000000.ply  image_000007.ply  image_000014.ply  image_000021.ply  image_000028.ply
image_000001.ply  image_000008.ply  image_000015.ply  image_000022.ply  image_000029.ply
image_000002.ply  image_000009.ply  image_000016.ply  image_000023.ply  image_000030.ply
image_000003.ply  image_000010.ply  image_000017.ply  image_000024.ply  image_000031.ply
image_000004.ply  image_000011.ply  image_000018.ply  image_000025.ply  image_000032.ply
image_000005.ply  image_000012.ply  image_000019.ply  image_000026.ply
image_000006.ply  image_000013.ply  image_000020.ply  image_000027.ply
```

#### Getting the intrinsic calibration values

There are a variety of ways to do this. You could get them at calibration time, or if you know where the calibrated values are stored, or if you open up a rosbag in python and read the `camera_info` topic values. I played back a bag taken on 7/27 and got these values which should be fairly good for data collected in Iowa:

```
P: [1721.122222835719, 0.0, 707.1728820800781, -102.8898281094086, 0.0, 1721.122222835719, 496.393856048584, 0.0, 0.0, 0.0, 1.0, 0.0]
```

That means we have
```
--focal-length 1721.12  [pixels]
--baseline 0.10289      [meters]
--center-x 707.17       [pixels]
--center-y 496.39       [pixels]
```

#### Setup steps the first time you set this up

The first time you download the tool, run the following setup installs. I haven't bothered to make a virtual environment for these because they are quite simple. We could make an environment, it just has to be saved and documented.

```
pip3 install opencv-python
pip3 install open3d
```

#### How to automate this

Assuming you know your desired intrinsic calibration values (and assuming the script is in `~/Downloads/`, paste the following code into `bash_aliases` and then just run `get_depth` in future terminals.

```
function get_depth() {
    for dirr in */
    do
        python ~/Downloads/disparity_to_depth.py \
            --baseline 0.10289 \
            --focal-length 1721.12 \
            --center-x 707.17 \
            --center-y 496.39 \
            --indir "$dirr"stereo_00/ \
            --outdir "$dirr"clouds/ \
            --imagedir "$dirr"theia_left_image_raw/
    done
}
```

#### How to visualize cloud files

I like using CloudCompare to visualize cloud files, which you can do on Jetstream OR on your own computer after rsync-ing files to your computer. CloudCompare can be installed once with `sudo apt-get -y install cloudcompare`. Personally I'd suggesting bringing the cloud files back to your own computer and viewing them there, CloudCompare may stress the remote desktop.