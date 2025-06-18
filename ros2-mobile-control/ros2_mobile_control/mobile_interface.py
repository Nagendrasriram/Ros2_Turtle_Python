from flask import Flask, request, render_template_string
from flask_socketio import SocketIO, emit
import threading

class MobileInterface:
    def __init__(self, ros2_controller):
        self.ros2_controller = ros2_controller
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.setup_routes()
        ros2_controller.set_mobile_interface(self)  #
    
    def setup_routes(self):
        HTML = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>ROS2 Turtle Control</title>
            <style>
                body { font-family: Arial, sans-serif; text-align: center; margin-top: 30px; }
                button {
                    padding: 16px 30px;
                    font-size: 18px;
                    margin: 10px;
                    border-radius: 10px;
                    border: none;
                    background-color: #4CAF50;
                    color: white;
                    cursor: pointer;
                }
                button:hover {
                    background-color: #45a049;
                }
                .btn-grid {
                    display: inline-grid;
                    grid-template-columns: auto auto auto;
                    grid-gap: 10px;
                    margin-top: 20px;
                }
                .mode-toggle {
                    margin: 20px;
                }
                .log {
                    font-size: 14px;
                    color: gray;
                    margin-top: 10px;
                }
            </style>
            <script src="https://cdn.socket.io/4.4.1/socket.io.min.js"></script>
            <script>
                const socket = io();

                socket.on('connect', () => {
                    console.log("✅ Connected to WebSocket server");
                    document.getElementById('status').innerText = "📶 Connected via WebSocket";
                });

                socket.on('ros_feedback', (data) => {
                    console.log("📨 ROS says:", data);
                    document.getElementById('status').innerText = "🤖 ROS: " + data.status;
                });
            </script>

            <script>
                let useTilt = false;
                let lastMove = '';

                function sendCommand(direction) {
                    fetch('/move', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/x-www-form-urlencoded',
                        },
                        body: 'direction=' + direction
                    }).then(() => {
                        document.getElementById('status').innerText = '✅ Command sent: ' + direction;
                    }).catch((err) => {
                        document.getElementById('status').innerText = '❌ Failed to send command: ' + err;
                    });
                }

                function toggleTilt() {
                    useTilt = !useTilt;
                    if (useTilt) {
                        document.getElementById('tiltBtn').innerText = "🛑 Disable Tilt Control";
                        document.getElementById('status').innerText = "Tilt control enabled.";
                        startMotion();
                    } else {
                        document.getElementById('tiltBtn').innerText = "📱 Enable Tilt Control";
                        document.getElementById('status').innerText = "Tilt control disabled.";
                        window.removeEventListener('deviceorientation', handleOrientation);
                        lastMove = '';  // Reset
                        sendCommand('stop');
                    }
                }

                function startMotion() {
                    if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
                        DeviceOrientationEvent.requestPermission()
                            .then(permissionState => {
                                if (permissionState === 'granted') {
                                    window.addEventListener('deviceorientation', handleOrientation);
                                    document.getElementById('log').innerText = "✅ Motion access granted.";
                                } else {
                                    alert('Permission denied for motion sensors.');
                                    document.getElementById('log').innerText = "❌ Permission denied.";
                                }
                            })
                            .catch(err => {
                                console.error("Permission error:", err);
                                document.getElementById('log').innerText = "❌ Error requesting permission: " + err;
                            });
                    } else {
                        // For Android or desktop
                        window.addEventListener('deviceorientation', handleOrientation);
                        document.getElementById('log').innerText = "ℹ️ Listening for orientation.";
                    }
                }

                function handleOrientation(event) {
                    if (!useTilt) return;

                    if (event.beta == null || event.gamma == null) {
                        document.getElementById('log').innerText = "⚠️ No sensor data received.";
                        return;
                    }

                    const beta = event.beta;
                    const gamma = event.gamma;
                    let move = 'stop';

                    if (beta > 30) move = 'forward';
                    else if (beta < -30) move = 'backward';
                    else if (gamma > 30) move = 'right';
                    else if (gamma < -30) move = 'left';

                    if (move !== lastMove) {
                        lastMove = move;
                        document.getElementById('status').innerText = "📲 Tilt Command: " + move + 
                            " (β: " + beta.toFixed(1) + ", γ: " + gamma.toFixed(1) + ")";
                        sendCommand(move);
                    }

                    document.getElementById('log').innerText = "β: " + beta.toFixed(1) + ", γ: " + gamma.toFixed(1);
                }

                window.addEventListener("deviceorientation", function(event) {
                    document.getElementById("log").innerText =
                    "🚨 deviceorientation triggered! β: " + event.beta + ", γ: " + event.gamma;
                });
            </script>
        </head>
        <body>
            <h1>🐢 ROS2 Turtle Controller</h1>

            <div class="mode-toggle">
                <button id="tiltBtn" onclick="toggleTilt()">📱 Enable Tilt Control</button>
            </div>

            <p id="status">Tilt control is disabled.</p>
            <p class="log" id="log">No tilt data yet.</p>

            <div class="btn-grid">
                <div></div>
                <button onclick="sendCommand('forward')">⬆️ Forward</button>
                <div></div>

                <button onclick="sendCommand('left')">⬅️ Left</button>
                <button onclick="sendCommand('stop')">⏹ Stop</button>
                <button onclick="sendCommand('right')">➡️ Right</button>

                <div></div>
                <button onclick="sendCommand('backward')">⬇️ Backward</button>
                <div></div>
            </div>
            <p id="status">Tilt control is disabled.</p>
            
            <p id="log" style="margin-top: 20px; color: orange;">
            🕵️ Waiting for deviceorientation event...
            </p>


        </body>
        </html>
        """

        @self.app.route('/', methods=['GET'])
        def index():
            return render_template_string(HTML)

        @self.app.route('/move', methods=['POST'])
        def move():
            direction = request.form['direction']
            self.receive_command(direction)
            return '', 204

        @self.socketio.on('connect')
        def handle_connect():
            print("📡 WebSocket client connected")
            emit('ros_feedback', {'status': 'Connected to ROS2 bridge'})

    def receive_command(self, command):
        print(f"[ROS2-MobileControl] Command received: {command}")
        if command == "left":
            self.ros2_controller.move_left()
        elif command == "right":
            self.ros2_controller.move_right()
        elif command == "forward":
            self.ros2_controller.move_forward()
        elif command == "backward":
            self.ros2_controller.move_backward()
        elif command == "stop":
            self.ros2_controller.stop()
        else:
            print("Unknown command received:", command)

    def send_ros_update(self, data):
        self.socketio.emit('ros_feedback', data)

    def run_server(self):
        threading.Thread(target=lambda: self.socketio.run(self.app, host='0.0.0.0', port=5000), daemon=True).start()
