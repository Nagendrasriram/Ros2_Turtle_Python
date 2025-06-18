Of course! Here's your final `README.md` with the **Google Drive link** to demo videos added under a new section called **🎥 Demo**:

---

````markdown
# 🤖 ROS2 Mobile Control Interface

<p align="center">
  <a href="https://docs.ros.org/en/humble/index.html">
    <img src="https://img.shields.io/badge/ROS2-Humble-blueviolet?logo=ros&logoColor=white" alt="ROS2">
  </a>
  <a href="https://choosealicense.com/licenses/mit/">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License">
  </a>
  <img src="https://img.shields.io/badge/Python-3.8%2B-blue?logo=python" alt="Python">
  <img src="https://img.shields.io/badge/Mobile%20Control-Gyroscope%20%2B%20Buttons-orange" alt="Mobile Control">
</p>


> 📱 **Control your TurtleSim from your mobile phone using Gyroscope and Touch Buttons!**  
> 🌐 Real-time control via Flask + Socket.IO with ROS2 bridge.

---

## ⚙️ Requirements

- 🐢 ROS2 (**Humble** or **Foxy**)
- 🐍 Python 3.8+
- 📱 Smartphone with Google Chrome
- 📶 Both PC and Phone on the **same Wi-Fi**

---

## 🚀 Installation

### 📥 Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Nagendrasriram/Ros2_Turtle_Python.git
cd ~/ros2_ws
colcon build --packages-select ros2_mobile_control
source install/setup.bash
````

### 📦 Install Python Dependencies

```bash
pip install flask flask-socketio eventlet
```

---

## 🕹️ How to Use

### 1. 🐢 Launch TurtleSim (Optional)

```bash
ros2 run turtlesim turtlesim_node
```

### 2. 🌐 Start Teleoperation Server

```bash
python3 -m ros2_mobile_control.main
```

Expected output:

```
🌍 Server running at http://192.168.1.42:5000
```

### 3. 📱 Open on Your Phone Browser

* Type the IP shown in your terminal.
* Use your phone's **gyroscope** or **touch buttons** to move the turtle!

---

## ✨ Features

* 🔄 **Real-time bidirectional communication**
* 📲 **Gyroscope-based tilt control**
* 🕹️ **Touch-based directional control (↑ ↓ ← →)**
* ⚙️ **ROS2 `cmd_vel` publisher integration**
* 🔌 Extensible for camera, voice, or gesture inputs

---

## 📌 Notes

* **Enable motion sensor access** in mobile Chrome:

  ```
  chrome://settings/content/siteDetails
  ```
* Both devices must be on the **same Wi-Fi**
* Run this before launching anything:

  ```bash
  source install/setup.bash
  ```

---

## 🎥 Demo

Watch how this project works in real-time here:
📂 [Google Drive Demo Videos](https://drive.google.com/drive/folders/181P38CHZwnOGETsMg1nkkosRpqWNzF1P)

---

## 🙌 Contributing

Pull requests, suggestions, and stars ⭐ are welcome!
Let’s improve real-time mobile teleoperation together 🚀

---

## 📄 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 Author

Built with 💙 by [Nagendra Sriram](https://github.com/Nagendrasriram)

---

🛠️ Powered by ROS2, Flask, WebSockets, and smartphone sensors!


```
