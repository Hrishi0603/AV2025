
# Autonomous-2024
This project is a result of a full semester course **ME 691-XIII Autonomous Vehicles** offered at IIT Gandhinagar by prof Harish PM. The project is completely student run and still under progress, there are further developments on the way. We have acheived Level 3 Autonomy on an Electric Golf cart with very less hardware changes.

There are multiple sections of the cart on which modifications were done. ![14-seater-electric-golf-cart](https://github.com/user-attachments/assets/39bc530b-ad84-4064-8a0a-6ae69f878818)

## Perception

### Hardware
One Intel RealSense d435 RGBD camera and one Lenovo FHD webcam.

### Software
The Intel RealSense d435 is used for obstacle detection and its distance estimation from the cart based on which we decide the cart's velocity. Currently we have 3 levels of velocity depending upon distance of the obstacle from the cart. The Lenovo FHD cam is used for lane detection, once we have the masked resgion of the lane for which we are using yolov11 pretrained model fine-tuned on our campus roads, we calculate the cross error betwen the masked lane's centre line and cart's centre line, we give the error in a PID controller which gives pwm commands to the motor connected to the steering wheel.
## Steering

### Hardware
The steering wheel of the cart is attached to a 12V high torque Planetary motor that is controlled via an Arduino and a Cytron 30A motor Driver(Owing to high current draw). The Motor Driver is powered through a Genx 10k 25C 3S Lipo Battery. The motor is attached to the steering wheel via a chain and sprocket mechanism which introduces a lot of backlash errors.
![image](https://github.com/user-attachments/assets/823e722e-f74f-49ef-a425-b8ad93724349)
![image](https://github.com/user-attachments/assets/fdc8665f-5bf5-4186-9278-bc28cafdc0dd)

### Software

This project implements an autonomous lane detection and steering system using a fine-tuned YOLOv11 model. The software processes real-time video feed to detect lanes, calculates the deviation between the lane centerline and the vehicle's centerline, and adjusts steering using a PID controller. The YOLOv11 model, trained on a custom campus road image dataset, effectively identifies lane boundaries even in low-light and nighttime conditions. A trapezoidal region of interest (ROI) mask is applied to focus the detection on the relevant road area.

The extracted lane mask is processed to compute the lane's centerline by averaging lane pixel positions at each vertical level of the frame. The normalized deviation from the vehicle's centerline is then passed to the PID controller, which generates a proportional, integral, and derivative output to determine the steering adjustment. The system outputs real-time feedback on error and control signals overlaid on the video feed, ensuring intuitive monitoring. This streamlined software approach ensures precise and reliable lane detection and autonomous steering control in various conditions.

---

### Implementation Images

![](https://github.com/user-attachments/assets/43384238-b416-4606-9800-697b994d8421)

![](https://github.com/user-attachments/assets/589f63b4-60e5-4805-9319-615ab07a5905)

![](https://github.com/user-attachments/assets/f240f17b-c9f2-4c80-80ce-5ccca941b410)

![](https://github.com/user-attachments/assets/c73e072d-7b10-4257-a3e7-455b14be568f)

![](https://github.com/user-attachments/assets/daa3668f-5069-48f4-883b-da5cb0f14cb3)







## Braking
### Hardware
We have bypassed the brake cables attached to the pedal and connected it to a 12v dc motor , wound the brake cable over it.
### Software
Used a cytron 10A motor driver and an arduino to control the brake motor. When an obstacle is detected and is within the cart's lane, the arduino sends a pwm to the brake motor on which the brake cable is wound. 
## Localization
### Hardware
Used an Arduino UNO R3 and a Neo 6mv2 GPS Module. The accuracy of this GPS module is close to 2.5~3.5 m while the GPS is fixed to atleast 5 satellites for trilateration.![image](https://github.com/user-attachments/assets/b914471f-1ac3-40b9-9c89-c2872a840783)

### Software
So our aim is to run the cart autonomously anywhere inside our campus. For that we need to know where is it located.
One of the difficulties was to get a map of the campus roads but thanks to OpenStreetMaps - ( an open source maps library) that made our jobs much easier. OpenStreetMaps comes with **osmnx** - a library that u can install directly via pip to your python and start using their functionalities like projection and map inference. ![image](https://github.com/user-attachments/assets/0b734ccb-e23b-4299-ae07-0e699680841a)


First we got the map of interest from their Website( Essentially an xml file that contains maps and its lat,long projections). Simply download the map and next we used a library called **networkx** to convert that map to a Graph of nodes and edges. That helps us in applying any path finding algorithm easily like djikstra or A*. ![image](https://github.com/user-attachments/assets/bddeb112-dd67-4628-8732-1265e47f4673)


Next we have a map of the campus roads visible through a tkinter gui window on which we select the Start and end node and a* algorithm returns a path(Red) that the cart can traverse. Meanwhile we can also show our current location on the map(Blue) by projecting the latitude, longitude values coming from the GPS Module back to the map by **pyproj** library. 

![WhatsApp Image 2024-11-15 at 17 50 44(1)](https://github.com/user-attachments/assets/8e77a232-152f-4668-973d-db460994d7e4)



### Interfacing the GPS Module

Although the gps module works okayish outside, it fails to work accurately when there is something obstructing its view (eg. Trees, Buildings). For most part the accuracy is close to 3 meters.
The GPS module prints the info in NMEA format that needs to be parsed. 

**$GPGGA**,110617.00,41XX.XXXXX,N,00831.54761,W,1,05,2.68,129.0,M,50.1,M,,*42

**110617** – represents the time at which the fix location was taken, 11:06:17 UTC

**41XX.XXXXX,N** – latitude 41 deg XX.XXXXX’ N

**00831.54761,W** – Longitude 008 deg 31.54761′ W

**2.68**– Horizontal dilution of position

**129.0, M** – Altitude, in meters above the sea level

Thankfully there are libraries that parse the data for u (**pynmea2** in python and **TinyGPS++** in Arduino Library Manager)

The cart then follows the path and starts and stops at the respective start and end node points.

## Acceleration

The cart accelerates on a 5v logic circuit. The speed is changed by pressing the throttle potentiometer. 
We bypassed the throttle cables and connected them to an arduino to send pwm to control speed digitally. Higher PWM = Higher speed.
We also implemented a trapizoidal velocity profiling method to increase & decrease the velocity of the cart when an obstacle is detected.



## Power Electronics
![image](https://github.com/user-attachments/assets/5f02721d-c644-43b5-9c12-58185abe1ccf)


# YOLO11n-seg Lane Detection with Custom Dataset

## Features
- Download and use a custom dataset of **700 images** from RoboFlow.
- Train a YOLO11n-seg segmentation model with specified parameters.
- Perform inference on test images using the trained model.
- Save the predicted images to a specified directory.

---

## Prerequisites

1. Python 3.8 or above.
2. Installed libraries:
   - `ultralytics`
   - `cv2` (OpenCV)
   - `matplotlib`
   - `os`
   - `roboflow`
3. GPU support is recommended for training.

---

## Dataset

The dataset is automatically downloaded from RoboFlow using the provided API key and project details.

1. Replace `KKtkzf9xscdAPBPBf4bT` with your RoboFlow API key if needed.
2. The dataset is downloaded and prepared for YOLO training format.

---

## Training the Model

The model is trained using the following parameters:
- **Epochs:** 50
- **Image size:** 416x416
- **Batch size:** 2
- **Initial learning rate:** 0.0001

Modify the training parameters as needed:
```python
results = model.train(
    data='E:/Autonomus Veh/yolo_road/Lane-Detection-2-1/data.yaml',
    epochs=50,
    imgsz=416,
    batch=2,
    lr0=1e-4,
    name='roboflow_yolo11_custom',
    val=True,
)
```

---

## Directory Structure

```
project/
│
├── yolo11n-seg.pt               # YOLO11n model weights
├── dataset/                     # Custom dataset downloaded from RoboFlow
│   ├── train/                   # Training data
│   ├── val/                     # Validation data
│   └── test/                    # Test data
├── predicted_images/            # Directory for storing inference results
└── main.py                      # Script for training and testing the model
```

---

# Open Issues and Improvement

  - Shifting Chain sprocket mechanism to something more reliable (Linear Actuator or Meshed Gears)
  - Tune PID of the steering to better Gain values
  - Use more accurate GPS Module to have less than a meter accuracy - important for autonomous turns
  - Integrate Safety mechanism/Manual Override into the cart from passenger side handles.
  - Integrate both the lane Detection code and object detection/acceleration into one single code
  - More robust breaking mechanism needs to be devised
  - Obstacle ***avoidance***
    


