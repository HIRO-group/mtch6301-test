#!/usr/bin/env python3
"""
Thanks Claude:

MTCH6301 Touch Data Web Visualizer

This script creates a web-based visualizer for touch data from the Arduino.
It runs a Flask server and uses WebSockets for real-time data streaming.
"""

import serial
import time
import argparse
import sys
import json
import threading
from collections import defaultdict, deque
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

class TouchVisualizer:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.initialization_complete = False
        self.running = False
        self.connection_active = False
        
        # Touch tracking
        self.touch_points = {}  # touchId -> (x, y, pen_down, timestamp)
        self.touch_trails = defaultdict(lambda: deque(maxlen=50))  # touchId -> trail of points
        
        # Display dimensions (will be auto-detected)
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.display_width = None
        self.display_height = None
        
        # Statistics
        self.packet_count = 0
        self.last_update_time = time.time()
        self.packets_per_second = 0
        
        # Init messages
        self.init_messages = []
        
        # Flask app
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'mtch6301_visualizer'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        self.setup_routes()
        self.setup_socketio()
    
    def reset_data(self):
        """Reset all tracking data"""
        self.touch_points = {}
        self.touch_trails = defaultdict(lambda: deque(maxlen=50))
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')
        self.display_width = None
        self.display_height = None
        self.packet_count = 0
        self.packets_per_second = 0
        self.init_messages = []
        self.initialization_complete = False
        
    def connect_serial(self):
        """Connect to the serial port"""
        try:
            if self.serial_conn:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            self.connection_active = True
            self.reset_data()
            self.socketio.emit('connection_status', {'connected': True, 'message': 'Connected to device'})
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to {self.port}: {e}")
            self.connection_active = False
            self.socketio.emit('connection_status', {'connected': False, 'message': f'Failed to connect: {str(e)}'})
            return False
    
    def check_connection(self):
        """Check if serial connection is still active"""
        if not self.serial_conn:
            return False
        
        try:
            # Try to read from the port to check if it's still connected
            self.serial_conn.in_waiting
            return True
        except (serial.SerialException, OSError):
            print("Serial connection lost")
            self.connection_active = False
            self.serial_conn = None
            self.socketio.emit('connection_status', {'connected': False, 'message': 'Device disconnected'})
            return False
    
    def wait_for_initialization(self):
        """Wait for the Arduino to send 'Init done' message"""
        print("Waiting for Arduino initialization...")
        
        while self.running and self.connection_active:
            if not self.check_connection():
                return False
                
            try:
                if self.serial_conn.in_waiting:
                    try:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            print(f"[INIT] {line}")
                            self.init_messages.append({'message': line, 'timestamp': time.time()})
                            self.socketio.emit('init_message', {'message': line, 'timestamp': time.time()})
                            
                            if "Init done" in line or "done" in line.lower():
                                print("Initialization complete! Switching to binary data mode...")
                                self.initialization_complete = True
                                self.socketio.emit('init_complete', {'status': 'ready'})
                                return True
                                
                    except UnicodeDecodeError:
                        pass
                        
            except Exception as e:
                print(f"Error during initialization: {e}")
                
            time.sleep(0.1)
        
        return False
    
    def read_touch_packet(self):
        """Read and parse a 6-byte touch packet from serial"""
        if not self.serial_conn or not self.check_connection() or self.serial_conn.in_waiting < 6:
            return None
            
        try:
            data = self.serial_conn.read(6)
            if len(data) != 6:
                return None
                
            touch_id = data[0]
            pen_down = bool(data[1])
            x = data[2] | (data[3] << 8)
            y = data[4] | (data[5] << 8)
            
            # Update min/max for auto-scaling
            if pen_down:
                self.min_x = min(self.min_x, x)
                self.max_x = max(self.max_x, x)
                self.min_y = min(self.min_y, y)
                self.max_y = max(self.max_y, y)
                
                self.display_width = self.max_x - self.min_x if self.max_x > self.min_x else 1
                self.display_height = self.max_y - self.min_y if self.max_y > self.min_y else 1
            
            return {
                'touch_id': touch_id,
                'pen_down': pen_down,
                'x': x,
                'y': y,
                'timestamp': time.time()
            }
            
        except Exception as e:
            print(f"Error reading touch packet: {e}")
            return None
    
    def data_reader_thread(self):
        """Background thread to read serial data"""
        packet_times = deque(maxlen=100)
        reconnect_attempts = 0
        max_reconnect_attempts = 3
        reconnect_delay = 2.0
        
        while self.running:
            if not self.connection_active:
                # Attempt to reconnect
                if reconnect_attempts < max_reconnect_attempts:
                    print(f"Attempting to reconnect ({reconnect_attempts + 1}/{max_reconnect_attempts})...")
                    if self.connect_serial():
                        reconnect_attempts = 0
                        # Start initialization
                        init_thread = threading.Thread(target=self.wait_for_initialization)
                        init_thread.daemon = True
                        init_thread.start()
                    else:
                        reconnect_attempts += 1
                        time.sleep(reconnect_delay)
                else:
                    time.sleep(5.0)  # Wait longer before trying again
                    reconnect_attempts = 0
                continue
            
            if not self.initialization_complete:
                time.sleep(0.1)
                continue
            
            packet = self.read_touch_packet()
            if packet is None:
                time.sleep(0.01)
                continue
            
            self.packet_count += 1
            packet_times.append(time.time())
            
            # Calculate packets per second
            if len(packet_times) > 1:
                time_span = packet_times[-1] - packet_times[0]
                if time_span > 0:
                    self.packets_per_second = len(packet_times) / time_span
            
            touch_id = packet['touch_id']
            x, y = packet['x'], packet['y']
            pen_down = packet['pen_down']
            
            # Update touch tracking
            if pen_down:
                self.touch_points[touch_id] = (x, y, pen_down, packet['timestamp'])
                self.touch_trails[touch_id].append({'x': x, 'y': y, 'time': packet['timestamp']})
            else:
                if touch_id in self.touch_points:
                    del self.touch_points[touch_id]
                # Add final point to trail
                if len(self.touch_trails[touch_id]) > 0:
                    self.touch_trails[touch_id].append({'x': x, 'y': y, 'time': packet['timestamp']})
            
            # Clean old trail points (older than 5 seconds)
            current_time = time.time()
            for trail in self.touch_trails.values():
                while trail and current_time - trail[0]['time'] > 5.0:
                    trail.popleft()
            
            # Send data to web clients
            self.socketio.emit('touch_data', {
                'packet': packet,
                'touch_points': {tid: {'x': x, 'y': y, 'pen_down': pd} 
                               for tid, (x, y, pd, _) in self.touch_points.items()},
                'trails': {str(tid): list(trail) for tid, trail in self.touch_trails.items() if trail},
                'display_bounds': {
                    'min_x': self.min_x if self.min_x != float('inf') else 0,
                    'max_x': self.max_x if self.max_x != float('-inf') else 1024,
                    'min_y': self.min_y if self.min_y != float('inf') else 0,
                    'max_y': self.max_y if self.max_y != float('-inf') else 1024,
                    'width': self.display_width or 1024,
                    'height': self.display_height or 1024
                },
                'stats': {
                    'packet_count': self.packet_count,
                    'packets_per_second': round(self.packets_per_second, 1)
                }
            })
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template_string(HTML_TEMPLATE)
    
    def setup_socketio(self):
        @self.socketio.on('connect')
        def handle_connect():
            print('Client connected')
            emit('status', {'message': 'Connected to touch visualizer'})
            emit('connection_status', {'connected': self.connection_active, 'message': 'Device connected' if self.connection_active else 'Device disconnected'})
            # Send init messages to new client
            for msg in self.init_messages:
                emit('init_message', msg)
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('Client disconnected')
        
        @self.socketio.on('clear_logs')
        def handle_clear_logs():
            self.init_messages = []
            emit('logs_cleared')
    
    def run(self):
        """Main run loop"""
        if not self.connect_serial():
            print("Initial connection failed, but server will keep trying to connect...")
        
        self.running = True
        
        try:
            # Start data reader thread
            data_thread = threading.Thread(target=self.data_reader_thread)
            data_thread.daemon = True
            data_thread.start()
            
            # Start initialization if connected
            if self.connection_active:
                init_thread = threading.Thread(target=self.wait_for_initialization)
                init_thread.daemon = True
                init_thread.start()
            
            print("Starting web server...")
            print("Open http://localhost:5000 in your browser to view the visualization")
            
            # Run Flask app
            self.socketio.run(self.app, host='0.0.0.0', port=5000, debug=False)
            
        except KeyboardInterrupt:
            print("\nStopped by user")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            self.running = False
            if self.serial_conn:
                self.serial_conn.close()
                print("Serial connection closed")
        
        return True

# HTML Template for the web interface
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>MTCH6301 Touch Visualizer</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body {
            margin: 0;
            padding: 20px;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background-color: #f8f9fa;
            color: #212529;
            line-height: 1.5;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            color: #495057;
            margin-bottom: 30px;
            font-weight: 300;
            font-size: 2.5rem;
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-bottom: 25px;
        }
        .stat-card {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            border-left: 4px solid #007bff;
            text-align: center;
        }
        .stat-value {
            font-size: 2rem;
            font-weight: 600;
            color: #007bff;
            margin-bottom: 5px;
        }
        .stat-label {
            color: #6c757d;
            font-size: 0.9rem;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .canvas-container {
            position: relative;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            margin: 20px 0;
            overflow: hidden;
        }
        #touchCanvas {
            display: block;
            background-color: white;
        }
        .init-section {
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 25px;
            overflow: hidden;
        }
        .init-header {
            background: #495057;
            color: white;
            padding: 15px 20px;
            font-weight: 500;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .init-messages {
            max-height: 250px;
            overflow-y: auto;
            padding: 15px 20px;
            font-family: 'SF Mono', Monaco, monospace;
            font-size: 13px;
            line-height: 1.4;
        }
        .init-message {
            color: #495057;
            margin-bottom: 8px;
            padding: 4px 0;
        }
        .init-message:last-child {
            margin-bottom: 0;
        }
        .timestamp {
            color: #6c757d;
            margin-right: 8px;
        }
        .status {
            text-align: center;
            padding: 15px;
            margin-bottom: 25px;
            border-radius: 8px;
            font-weight: 500;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .status.connected {
            background-color: #d4edda;
            color: #155724;
            border-left: 4px solid #28a745;
        }
        .status.disconnected {
            background-color: #f8d7da;
            color: #721c24;
            border-left: 4px solid #dc3545;
        }
        .status.connecting {
            background-color: #fff3cd;
            color: #856404;
            border-left: 4px solid #ffc107;
        }
        .touch-info {
            position: absolute;
            top: 15px;
            right: 15px;
            background: rgba(255, 255, 255, 0.95);
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.15);
            font-size: 13px;
            max-width: 250px;
            backdrop-filter: blur(10px);
        }
        .touch-info h4 {
            margin: 0 0 10px 0;
            color: #495057;
            font-size: 14px;
            font-weight: 600;
        }
        .touch-detail {
            padding: 4px 0;
            font-family: 'SF Mono', Monaco, monospace;
        }
        .clear-btn {
            background: #6c757d;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 12px;
            transition: background-color 0.2s;
        }
        .clear-btn:hover {
            background: #5a6268;
        }
        .connection-indicator {
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .connection-indicator.connected {
            background-color: #28a745;
        }
        .connection-indicator.disconnected {
            background-color: #dc3545;
        }
        
        /* Scrollbar styling */
        .init-messages::-webkit-scrollbar {
            width: 6px;
        }
        .init-messages::-webkit-scrollbar-track {
            background: #f1f3f4;
        }
        .init-messages::-webkit-scrollbar-thumb {
            background: #dadce0;
            border-radius: 3px;
        }
        .init-messages::-webkit-scrollbar-thumb:hover {
            background: #bdc1c6;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>MTCH6301 Touch Visualizer</h1>
        
        <div class="status connecting" id="status">
            <span class="connection-indicator disconnected" id="connectionIndicator"></span>
            Connecting...
        </div>
        
        <div class="init-section" id="initSection" style="display: none;">
            <div class="init-header">
                <span>
                    <span class="connection-indicator disconnected" id="initConnectionIndicator"></span>
                    Device Messages
                </span>
                <button class="clear-btn" onclick="clearLogs()">Clear Logs</button>
            </div>
            <div class="init-messages" id="initMessages">
                <div class="init-message">Waiting for device messages...</div>
            </div>
        </div>
        
        <div class="stats">
            <div class="stat-card">
                <div class="stat-value" id="packetCount">0</div>
                <div class="stat-label">Packets</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="packetsPerSec">0</div>
                <div class="stat-label">Packets/sec</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="displaySize">Unknown</div>
                <div class="stat-label">Display Size</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="activeTouch">0</div>
                <div class="stat-label">Active Touches</div>
            </div>
        </div>
        
        <div class="canvas-container">
            <canvas id="touchCanvas" width="800" height="600"></canvas>
            <div class="touch-info" id="touchInfo">
                <h4>Touch Information</h4>
                <div id="touchDetails">No active touches</div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        const canvas = document.getElementById('touchCanvas');
        const ctx = canvas.getContext('2d');
        
        let displayBounds = {min_x: 0, max_x: 1024, min_y: 0, max_y: 1024, width: 1024, height: 1024};
        let touchPoints = {};
        let trails = {};
        let colors = ['#007bff', '#dc3545', '#28a745', '#fd7e14', '#6610f2', '#20c997', '#e83e8c', '#ffc107'];
        let isConnected = false;
        
        // Initialize canvas size
        function resizeCanvas() {
            const container = canvas.parentElement;
            canvas.width = container.clientWidth;
            canvas.height = Math.min(600, canvas.width * 0.6);
        }
        
        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();
        
        function updateConnectionStatus(connected, message) {
            const statusEl = document.getElementById('status');
            const indicatorEl = document.getElementById('connectionIndicator');
            const initIndicatorEl = document.getElementById('initConnectionIndicator');
            
            isConnected = connected;
            
            if (connected) {
                statusEl.className = 'status connected';
                statusEl.innerHTML = `<span class="connection-indicator connected"></span>${message}`;
                indicatorEl.className = 'connection-indicator connected';
                initIndicatorEl.className = 'connection-indicator connected';
            } else {
                statusEl.className = 'status disconnected';
                statusEl.innerHTML = `<span class="connection-indicator disconnected"></span>${message}`;
                indicatorEl.className = 'connection-indicator disconnected';
                initIndicatorEl.className = 'connection-indicator disconnected';
            }
        }
        
        function clearLogs() {
            socket.emit('clear_logs');
            document.getElementById('initMessages').innerHTML = '<div class="init-message">Logs cleared</div>';
        }
        
        function drawVisualization() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            if (displayBounds.width <= 1) return;
            
            // Calculate scaling
            const padding = 40;
            const scaleX = (canvas.width - 2 * padding) / displayBounds.width;
            const scaleY = (canvas.height - 2 * padding) / displayBounds.height;
            const scale = Math.min(scaleX, scaleY);
            
            const offsetX = padding + (canvas.width - 2 * padding - displayBounds.width * scale) / 2;
            const offsetY = padding + (canvas.height - 2 * padding - displayBounds.height * scale) / 2;
            
            function screenX(x) {
                return offsetX + (x - displayBounds.min_x) * scale;
            }
            
            function screenY(y) {
                return offsetY + (y - displayBounds.min_y) * scale;
            }
            
            // Draw display boundary
            ctx.strokeStyle = '#dee2e6';
            ctx.lineWidth = 2;
            ctx.strokeRect(
                offsetX, 
                offsetY, 
                displayBounds.width * scale, 
                displayBounds.height * scale
            );
            
            // Draw coordinate grid
            ctx.strokeStyle = '#f8f9fa';
            ctx.lineWidth = 1;
            for (let i = 1; i < 10; i++) {
                const x = offsetX + (displayBounds.width * scale * i / 10);
                const y = offsetY + (displayBounds.height * scale * i / 10);
                
                ctx.beginPath();
                ctx.moveTo(x, offsetY);
                ctx.lineTo(x, offsetY + displayBounds.height * scale);
                ctx.stroke();
                
                ctx.beginPath();
                ctx.moveTo(offsetX, y);
                ctx.lineTo(offsetX + displayBounds.width * scale, y);
                ctx.stroke();
            }
            
            // Draw trails
            Object.entries(trails).forEach(([touchId, trail], index) => {
                if (trail.length < 2) return;
                
                const color = colors[parseInt(touchId) % colors.length];
                ctx.strokeStyle = color;
                
                for (let i = 1; i < trail.length; i++) {
                    const alpha = (i / trail.length) * 0.8 + 0.2;
                    ctx.globalAlpha = alpha;
                    ctx.lineWidth = 2 + (alpha * 2);
                    
                    ctx.beginPath();
                    ctx.moveTo(screenX(trail[i-1].x), screenY(trail[i-1].y));
                    ctx.lineTo(screenX(trail[i].x), screenY(trail[i].y));
                    ctx.stroke();
                }
            });
            
            ctx.globalAlpha = 1.0;
            
            // Draw current touch points
            Object.entries(touchPoints).forEach(([touchId, point], index) => {
                const color = colors[parseInt(touchId) % colors.length];
                const x = screenX(point.x);
                const y = screenY(point.y);
                
                // Draw touch circle
                ctx.fillStyle = color;
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 2;
                
                ctx.beginPath();
                ctx.arc(x, y, 12, 0, 2 * Math.PI);
                ctx.fill();
                ctx.stroke();
                
                // Draw touch ID
                ctx.fillStyle = '#ffffff';
                ctx.font = 'bold 11px -apple-system, BlinkMacSystemFont, sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText(`${touchId}`, x, y + 3);
            });
        }
        
        function updateTouchInfo() {
            const touchDetails = document.getElementById('touchDetails');
            const activeTouches = Object.keys(touchPoints).length;
            
            document.getElementById('activeTouch').textContent = activeTouches;
            
            if (activeTouches === 0) {
                touchDetails.innerHTML = '<div style="color: #6c757d;">No active touches</div>';
            } else {
                let html = '';
                Object.entries(touchPoints).forEach(([touchId, point]) => {
                    const color = colors[parseInt(touchId) % colors.length];
                    html += `<div class="touch-detail" style="color: ${color}">
                        Touch ${touchId}: (${point.x}, ${point.y})
                    </div>`;
                });
                touchDetails.innerHTML = html;
            }
        }
        
        // Socket event handlers
        socket.on('status', (data) => {
            console.log('Status:', data.message);
        });
        
        socket.on('connection_status', (data) => {
            updateConnectionStatus(data.connected, data.message);
            if (data.connected) {
                document.getElementById('initSection').style.display = 'block';
            }
        });
        
        socket.on('init_message', (data) => {
            const messagesDiv = document.getElementById('initMessages');
            document.getElementById('initSection').style.display = 'block';
            
            const messageDiv = document.createElement('div');
            messageDiv.className = 'init-message';
            const timestamp = new Date(data.timestamp * 1000).toLocaleTimeString();
            messageDiv.innerHTML = `<span class="timestamp">${timestamp}</span>${data.message}`;
            messagesDiv.appendChild(messageDiv);
            messagesDiv.scrollTop = messagesDiv.scrollHeight;
        });
        
        socket.on('init_complete', (data) => {
            updateConnectionStatus(true, 'Ready! Touch the screen to see data.');
        });
        
        socket.on('logs_cleared', () => {
            document.getElementById('initMessages').innerHTML = '<div class="init-message">Logs cleared</div>';
        });
        
        socket.on('touch_data', (data) => {
            touchPoints = data.touch_points;
            trails = data.trails;
            displayBounds = data.display_bounds;
            
            // Update statistics
            document.getElementById('packetCount').textContent = data.stats.packet_count;
            document.getElementById('packetsPerSec').textContent = data.stats.packets_per_second;
            document.getElementById('displaySize').textContent = 
                `${displayBounds.width} × ${displayBounds.height}`;
            
            updateTouchInfo();
            drawVisualization();
        });
        
        socket.on('connect', () => {
            console.log('Connected to server');
        });
        
        socket.on('disconnect', () => {
            updateConnectionStatus(false, 'Disconnected from server');
        });
        
        // Initial draw
        drawVisualization();
    </script>
</body>
</html>
'''

def main():
    parser = argparse.ArgumentParser(description='MTCH6301 Touch Data Web Visualizer')
    parser.add_argument('port', help='Serial port (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    
    args = parser.parse_args()
    
    print("MTCH6301 Touch Data Web Visualizer")
    print("=" * 40)
    print(f"Port: {args.port}")
    print(f"Baud rate: {args.baudrate}")
    print()
    
    visualizer = TouchVisualizer(args.port, args.baudrate)
    
    if not visualizer.run():
        sys.exit(1)

if __name__ == "__main__":
    main()