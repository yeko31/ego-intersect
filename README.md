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
## 🕹️ Webots Simulation Platform
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/2.webm.gif" width="400">
For more details, please visit the Webots <a href="https://github.com/yeko31/ego-intersect/tree/main/webots">page</a>.

## 🗂️ Dataset is available at this <a href="https://huggingface.co/datasets/yeko31/ego-intersect">page</a>.

## 🧠 Grad-CAM Analysis

To understand and visualize the decision-making of our CNN models, we used **Grad-CAM** (Gradient-weighted Class Activation Mapping).  
Grad-CAM highlights regions of the input image that contribute most to the network’s predictions, providing insight into **which visual features the network focuses on** during navigation and intersection detection.


### 🔹 Models Analyzed
- **ResNet50**  
  High-capacity model used for detailed feature extraction. Grad-CAM analysis shows strong attention on **lane markings and intersection cues**.  

- **MobileNetV2**  
  Lightweight model suitable for real-time deployment on resource-limited hardware. Grad-CAM highlights key road features while maintaining efficiency.  

## 📊 Results

### 🔹 Multi Class(5) Classification
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/multi_class.jpg" width="500">

  
### 🔹 Multi Label Classification
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/multi_label.jpg" width="500">

### 🔹 Decomposition of labels
<img src="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/orthogonal_study.jpg" width="500">

### 🔹 Performance Comparison 
| Parameter | ResNet50 | MobileNetV2 | PULP-DroNetv3(Quantized)|
|----------|----------|----------|----------|
| Flops  | 4109470720 | 312917056  | 50445696 |
| Param size(MB) |94.06 | 8.91   | 1.28 |
| Forward/backward pass size (MB):   | 4173.73   |1709.60  | 136.48  |

### 🔹 Model Performance Comparison on Test Data(RNet50-L2)

| Parameter    | ResNet50(FP-32) | ResNet50-Static Quantized(Int-8) |
|--------------|----------|---------------------------|
| Hamming Loss | 0.2500     |    0.2533(+1.32%)                       |
| Precision    | 0.6667     |        0.6633(-0.51%)                   |
| Recall       | 0.9362     |          0.9362                 |
| F1-score     | 0.7788     |            0.7765(-0.3%)               |
| Model_size(MB)   | 94.06    |    24.1(-74.4%)                     |





## 📂 File Highlights

- **webots/** – All the scripts used in webots experiments.
- **real_world/** – All the scripts used in real world experiments.  
- **gradCAM/** – Contains Grad-CAM script which was used to get results for ResNet50 and MobileNetV2.  
- **test_script/** – Scripts of the unseen data validation.  
- **Videos/** – Simulation and real-world demo videos.
- **Trained Models** - To access all the trained models. Please visit this <a href="https://huggingface.co/yeko31/ego-intersect/tree/main">page</a>.
- **Supplemantary Materials** - To access, please visit this <a href="https://github.com/yeko31/ego-intersect/raw/main/webots/Videos/">page</a>.


---
![Visitors](https://visitor-badge.laobi.icu/badge?page_id=yeko31.ego-intersect)  
⭐ If you find this project useful, please give it a star!

## 🔗 Notes

- Grad-CAM results are crucial for **interpreting model behavior** and validating that the network focuses on meaningful road and intersection features.  
- Real-world experiments demonstrate that the **simulation-to-reality pipeline works reliably**, bridging the gap between Webots digital twins and physical hardware.

---

> 🎥 More simulation and real-world demo videos can be found in the `Videos/` folder.
