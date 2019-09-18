Commands to train Mask RCNN model for carton detection

1. open terminator
2. use Anaconda to activate environment for Mask RCNN
conda activate tf_gpu

3. python3 carton.py train --dataset=/home/qxuaj/carton_folding/Mask_RCNN/process_image/carton/carton_datasets --weights=coco


=================================
# Color Splash Example

This is an example showing the use of Mask RCNN in a real application.
We train the model to detect cartons only, and then we use the generated 
masks to keep cartons in color while changing the rest of the image to
grayscale.


[This blog post](https://engineering.matterport.com/splash-of-color-instance-segmentation-with-mask-r-cnn-and-tensorflow-7c761e238b46) describes this sample in more detail.

![Balloon Color Splash](/assets/carton_color_splash.gif)


## Installation
From the [Releases page](https://github.com/matterport/Mask_RCNN/releases) page:
1. Download `mask_rcnn_carton.h5`. Save it in the root directory of the repo (the `mask_rcnn` directory).
2. Download `carton_dataset.zip`. Expand it such that it's in the path `mask_rcnn/datasets/carton/`.

## Apply color splash using the provided weights
Apply splash effect on an image:

```bash
python3 carton.py splash --weights=/path/to/mask_rcnn/mask_rcnn_carton.h5 --image=<file name or URL>
```

Apply splash effect on a video. Requires OpenCV 3.2+:

```bash
python3 carton.py splash --weights=/path/to/mask_rcnn/mask_rcnn_carton.h5 --video=<file name or URL>
```


## Run Jupyter notebooks
Open the `inspect_carton_data.ipynb` or `inspect_carton_model.ipynb` Jupter notebooks. You can use these notebooks to explore the dataset and run through the detection pipelie step by step.

## Train the Balloon model

Train a new model starting from pre-trained COCO weights
```
python3 carton.py train --dataset=/home/qxuaj/carton_folding/Mask_RCNN/process_image/carton/carton_datasets --weights=coco
```

Resume training a model that you had trained earlier
```
python3 carton.py train --dataset=/home/qxuaj/Desktop/carton_perception/Mask_RCNN/datasets/carton --weights=last
```

Train a new model starting from ImageNet weights
```
python3 carton.py train --dataset=/path/to/carton/dataset --weights=imagenet
```

The code in `carton.py` is set to train for 3K steps (30 epochs of 100 steps each), and using a batch size of 2. 
Update the schedule to fit your needs.
