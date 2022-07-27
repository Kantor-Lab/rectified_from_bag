# rectified_from_bag
Take in a set of rosbags and output rectified images.

In order to get this docker image you can pull the image like so:
```
docker pull franzericschneider/raft-from-bag:v5
```

Then this is how you start a docker container:
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
1. The topics are named in the form `/.../.../image_raw`
2. L/R stereo pairs of topics are sorted alphabetically into L/R pairs
3. If there are multiple stereo pairs the topic pairs are grouped together alphabetically

For example, the topcs `(/cam0, /cam1, /cam2, /cam3)` are assumed to have `(cam0, /cam1)` and `(/cam2, /cam3)` as L/R stereo pairs because of their sorting order. This is a pretty harsh assumption, if it violated in the future we could fix that.

If you have the container open in one terminal, you can open a second terminal like so:
```
sudo docker exec -it extract /bin/bash
```