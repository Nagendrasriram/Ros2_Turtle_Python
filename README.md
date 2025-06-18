
---

# 🤖 ROS2 Mobile Control Interface

**Control your TurtleSim from your mobile phone using Gyroscope and Touch Buttons!**  
Real-time control via Flask and Socket.IO with ROS2 integration.

---

## 📋 About the Project

This project enables mobile-based teleoperation of a ROS2 Turtle (like `turtlesim`) using your smartphone’s:
- Gyroscope sensor (tilt-based control)
- Touch buttons (up, down, left, right)

It uses a Flask web server and WebSocket communication to send commands in real time to a ROS2 node.


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
