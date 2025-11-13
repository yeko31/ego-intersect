# EGOcentric-INTERSECTion 

This repository contains the full implementation of our **vision-based navigation and intersection detection project** using Webots digital twins, along with real-world deployment on the **Crazyflie nano-drone**.  
The project leverages **deep learning models**, **simulation environments**, and **automated data collection** to explore robust navigation strategies.

---

## 🚁 Real-World Deployment: Crazyflie Nano-Drone

Beyond simulation, we deployed the navigation pipeline on a **Crazyflie nano-drone**.  

The workflow consists of:
1. **Streaming camera frames** from the onboard Crazyflie camera.  
2. **Real-time processing** on an external computer using the trained CNN models.  
3. **Automated control commands** sent back to the drone for navigation.  

<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/real_world.gif" width="600">

- The system successfully detects intersections and navigates autonomously in a test environment.  
- Data is logged and categorized into the standard **left_right_forward folder format** for further analysis.

---

## 🧠 Grad-CAM Analysis

To understand and visualize the decision-making of our CNN models, we used **Grad-CAM** (Gradient-weighted Class Activation Mapping).  
Grad-CAM highlights regions of the input image that contribute most to the network’s predictions, providing insight into **which visual features the network focuses on** during navigation and intersection detection.


### 🔹 Models Analyzed
- **ResNet50**  
  High-capacity model used for detailed feature extraction. Grad-CAM analysis shows strong attention on **lane markings and intersection cues**.  

- **MobileNetV2**  
  Lightweight model suitable for real-time deployment on resource-limited hardware. Grad-CAM highlights key road features while maintaining efficiency.  

## 📊 Results

### 🔹 Multi Class Classification
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/multi_class.jpg" width="500">

  
### 🔹 Multi Label Classification
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/multi_label.jpg" width="500">

### 🔹 Decomposition of labels
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/orthogonal_study.jpg" width="500">


## 📂 File Highlights

- **webots/** – All the scripts used in webots experiments.
- **real_world/** – All the scripts used in real world experiments.  
- **gradCAM/** – Contains Grad-CAM script which was used to get results for ResNet50 and MobileNetV2.  
- **test_script/** – Scripts of the unseen data validation.  
- **Videos/** – Simulation and real-world demo videos.  


---
![Visitors](https://visitor-badge.laobi.icu/badge?page_id=yeko31.ego-intersect)  
⭐ If you find this project useful, please give it a star!

## 🔗 Notes

- Grad-CAM results are crucial for **interpreting model behavior** and validating that the network focuses on meaningful road and intersection features.  
- Real-world experiments demonstrate that the **simulation-to-reality pipeline works reliably**, bridging the gap between Webots digital twins and physical hardware.

---

> 🎥 More simulation and real-world demo videos can be found in the `Videos/` folder.
