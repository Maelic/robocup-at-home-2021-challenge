YOLOv4 Training on the YCB Dataset
--------------------

## 1. Installation
### 1.1. Install CUDA

Execute the script and follow isntructions:

```buildoutcfg
sh install_cuda.sh
```

Rebbot your computer:

```buildoutcfg
sudo reboot
```


### 1.2. Install CUDDN and darknet (YOLO building tool)
```buildoutcfg
sh install.sh
```

## 2. Start the training

Verify that you have a folder darknet with those files inside:
- libdarknet.so
- obj.data
- obj.names
- train.txt
- test.txt

And launch the command:
```buildoutcfg
./darknet detector train obj.data yolov4-tiny-obj.cfg yolov4-tiny.conv.29
```

If you have any issue with the installation or training please consult the [YoloV4 official repository](https://github.com/AlexeyAB/darknet).
