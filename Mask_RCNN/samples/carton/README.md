# Commands to train Mask RCNN model for carton detection

1. open terminator
2. use Anaconda to activate environment for Mask RCNN
conda activate tf_gpu
3. Train a model with COCO
```
python3 carton.py train --dataset=~/catkin_ws/src/pickpack/Mask_RCNN/datasets/carton --weights=coco
```
4. Resume training a model that you had trained earlier
```
python3 carton.py train --dataset=~/catkin_ws/src/pickpack/Mask_RCNN/datasets/carton --weights=last
```

Train a new model starting from ImageNet weights
```
python3 carton.py train --dataset=/path/to/carton/dataset --weights=imagenet
```

The code in `carton.py` is set to train for 3K steps (30 epochs of 100 steps each), and using a batch size of 2. 
Update the schedule to fit your needs.
