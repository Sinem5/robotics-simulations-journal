## Week 4 Scipting Real Tine Segmentation for Isaac Sim

I want to be as honest as possible this week. Unfortunately, I was unable to work as effectively as I had hoped; my output was below my expectations. I did make some mistakes, but I appreciate your patience as we move forward.

First, I demonstrated how to adjust the data, but I believe there are better methods available. The final data is presented below. I also started writing the script we need for real-time segmentation. While reviewing the training results of my custom-trained YOLO model, I found the outcomes to be unsatisfactory. Therefore, I think it might be better to use the standard YOLO model without training it first.

```

Week4/
├── final_data/          # Dataset folder
│   ├── test/            # Test dataset
│   │   ├── images/      # Test images
│   │   └── labels/      # Test labels
│   ├── train/           # Training dataset
│   │   ├── images/      # Training images
│   │   └── labels/      # Training labels
│   └── val/             # Validation dataset
│       ├── images/      # Validation images
│       └── labels/      # Validation labels
├── data.yaml            # Dataset config file
├── adjust_data.py       # Script to adjust data
└── realtime_seg_det.py  # Real-time segmentation detection script

```


In the upcoming week, I plan to add detection capabilities, as focusing solely on segmentation with the untrained model doesn’t seem sufficient. Please stay tuned for updates; this is all I can manage for now. I’ll do my best to improve. See you next week!
