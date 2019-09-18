# Commands to train Mask RCNN model for board detection

1. open terminator
2. use Anaconda to activate environment for Mask RCNN
conda activate tf_gpu
3. Train a model with COCO
```
python3 board.py train --dataset=~/catkin_ws/src/pickpack/Mask_RCNN/datasets/board --weights=coco
```
4. Resume training a model that you had trained earlier
```
python3 board.py train --dataset=~/catkin_ws/src/pickpack/Mask_RCNN/datasets/board --weights=last
```

The code in `board.py` is set to train for 3K steps (30 epochs of 100 steps each), and using a batch size of 2. 
Update the schedule to fit your needs.
