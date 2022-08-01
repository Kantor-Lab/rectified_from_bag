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
