# rectified_from_bag
Take in a set of rosbags and output rectified images

```
sudo docker container run \
    -it \
    --gpus all \
    --rm \
    --name extract \
    -v /home/eric/Downloads/ROW5/:/home/workspace/ \
    -v /home/eric/cmu_courses/rectified_from_bag/:/home/extract/ \
    franzericschneider/raft-from-bag:v0
```

This assumes that
1. The topics are named in the form `/.../.../image_raw`
2. L/R stereo pairs of topics are sorted alphabetically into L/R pairs
3. If there are multiple stereo pairs the topic pairs are grouped together alphabetically

For example, the topcs `(/cam0, /cam1, /cam2, /cam3)` are assumed to have `(cam0, /cam1)` and `(/cam2, /cam3)` as L/R stereo pairs because of their sorting order.
