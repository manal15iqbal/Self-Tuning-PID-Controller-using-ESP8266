"""
ESP8266 to Web Dashboard Bridge
Connects to ESP via serial and serves web dashboard with real-time data
"""

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, render_template, request, send_from_directory
from flask_cors import CORS
import threading
import time
import re
from datetime import datetime
from collections import deque
import json
import os

app = Flask(__name__)
CORS(app)

# ===== CONFIGURATION =====
MAX_HISTORY = 80
UPDATE_INTERVAL = 0.4
BAUDRATE = 115200
ESP_TIMEOUT = 2  # seconds to wait for ESP after connection

# ===== GLOBAL VARIABLES =====
serial_thread = None
running = False
data_lock = threading.Lock()

# Data storage
data_history = {
    'time': deque(maxlen=MAX_HISTORY),
    'kp': deque(maxlen=MAX_HISTORY),
    'ki': deque(maxlen=MAX_HISTORY),
    'kd': deque(maxlen=MAX_HISTORY),
    'speed': deque(maxlen=MAX_HISTORY),
    'error': deque(maxlen=MAX_HISTORY),
    'phase': deque(maxlen=MAX_HISTORY),  # 0=IDLE, 1=RUNNING, 2=TUNING
    'stable': deque(maxlen=MAX_HISTORY)  # 0=UNSTABLE, 1=STABLE
}

current_data = {
    # PID Values
    'kp': 0.5,
    'ki': 0.1,
    'kd': 0.05,
    
    # Motor Values
    'speed': 0.0,
    'error': 0.0,
    
    # System State
    'phase': 0,           # 0=IDLE, 1=RUNNING, 2=TUNING
    'progress': 0.0,      # 0-100%
    'stable': True,       # True/False
    
    # Status Info
    'last_update': 'Never',
    'connection_status': '🔴 DISCONNECTED',
    'data_points': 0,
    'uptime': 0.0,
    'motor_running': False,
    'tuning_active': False,
    
    # ESP Info
    'esp_connected': False,
    'esp_port': 'None'
}

# Start time for graphs
start_time = time.time()

# ===== SERIAL THREAD CLASS =====
class SerialThread(threading.Thread):
    def __init__(self, port_name):
        super().__init__()
        self.port_name = port_name
        self.baudrate = BAUDRATE
        self.running = True
        self.serial_port = None
        self.daemon = True
        self.last_data_time = time.time()
        
    def run(self):
        global current_data, running
        
        try:
            print(f"📡 Connecting to {self.port_name} at {self.baudrate} baud...")
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=1
            )
            
            # Wait for ESP to reset
            time.sleep(ESP_TIMEOUT)
            
            # Clear any existing data
            self.serial_port.reset_input_buffer()
            
            print(f"✅ Connected to ESP at {self.port_name}")
            
            with data_lock:
                current_data['esp_connected'] = True
                current_data['esp_port'] = self.port_name
                current_data['connection_status'] = f'🟢 CONNECTED to {self.port_name}'
            
            running = True
            buffer = ""
            
            while self.running and self.serial_port:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        self.last_data_time = time.time()
                        
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                process_serial_data(line)
                    
                    # Check for timeout (no data for 5 seconds)
                    if time.time() - self.last_data_time > 5:
                        with data_lock:
                            if current_data['esp_connected']:
                                current_data['connection_status'] = f'🟡 CONNECTED (No data)'
                    
                    time.sleep(0.01)
                    
                except serial.SerialException as e:
                    print(f"⚠️ Serial error: {e}")
                    self.running = False
                    break
                except Exception as e:
                    print(f"⚠️ Error in serial thread: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"❌ Failed to connect to {self.port_name}: {e}")
            with data_lock:
                current_data['esp_connected'] = False
                current_data['connection_status'] = f'🔴 Failed to connect to {self.port_name}'
        
        finally:
            self.close_port()
    
    def send_command(self, cmd):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{cmd}\n".encode())
                print(f"📤 Sent command: '{cmd}'")
                return True
            except Exception as e:
                print(f"❌ Failed to send command: {e}")
                return False
        return False
    
    def close_port(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("🔌 Serial port closed")
    
    def stop(self):
        self.running = False
        self.close_port()
        self.join(timeout=1)

# ===== DATA PROCESSING =====
def process_serial_data(data):
    """Process data received from ESP"""
    global start_time
    
    with data_lock:
        # Update timestamp
        current_time = time.time() - start_time
        current_data['last_update'] = datetime.now().strftime('%H:%M:%S')
        current_data['uptime'] = current_time
        current_data['data_points'] = len(data_history['time']) + 1
        current_data['connection_status'] = f'🟢 CONNECTED - Receiving data'
        
        # Add to history
        data_history['time'].append(current_time)
        
        # Debug: print raw data (optional)
        # print(f"📥 ESP: {data}")
        
        # Parse PID values
        if "Kp = " in data:
            try:
                value = float(data.split("Kp = ")[1].strip())
                current_data['kp'] = value
                data_history['kp'].append(value)
            except:
                data_history['kp'].append(current_data.get('kp', 0.5))
                
        elif "Ki = " in data:
            try:
                value = float(data.split("Ki = ")[1].strip())
                current_data['ki'] = value
                data_history['ki'].append(value)
            except:
                data_history['ki'].append(current_data.get('ki', 0.1))
                
        elif "Kd = " in data:
            try:
                value = float(data.split("Kd = ")[1].strip())
                current_data['kd'] = value
                data_history['kd'].append(value)
            except:
                data_history['kd'].append(current_data.get('kd', 0.05))
        else:
            # If no PID data, use last values
            data_history['kp'].append(current_data.get('kp', 0.5))
            data_history['ki'].append(current_data.get('ki', 0.1))
            data_history['kd'].append(current_data.get('kd', 0.05))
        
        # Extract sensor values
        numbers = re.findall(r"[-+]?\d*\.\d+|\d+", data)
        if numbers:
            if "RPM:" in data:
                try:
                    speed = float(numbers[0])
                    current_data['speed'] = speed
                    data_history['speed'].append(speed)
                except:
                    data_history['speed'].append(current_data.get('speed', 0))
                    
            elif "Noise=" in data:
                try:
                    error = float(numbers[0])
                    current_data['error'] = error
                    data_history['error'].append(error)
                except:
                    data_history['error'].append(current_data.get('error', 0))
        else:
            # Default values if no sensor data
            data_history['speed'].append(current_data.get('speed', 0))
            data_history['error'].append(current_data.get('error', 0))
        
        # Update system state based on ESP messages
        if "Motor STARTED" in data:
            current_data['motor_running'] = True
            current_data['tuning_active'] = False
            current_data['phase'] = 1  # RUNNING
            current_data['stable'] = True
            current_data['progress'] = 100
            
        elif "Motor STOPPED" in data:
            current_data['motor_running'] = False
            current_data['tuning_active'] = False
            current_data['phase'] = 0  # IDLE
            current_data['stable'] = True
            current_data['progress'] = 0
            
        elif "STARTING TUNING" in data or ">>> STARTING TUNING SEQUENCE" in data:
            current_data['tuning_active'] = True
            current_data['phase'] = 2  # TUNING
            current_data['stable'] = False
            current_data['progress'] = 50
            
        elif "TUNING COMPLETE" in data:
            current_data['tuning_active'] = False
            current_data['phase'] = 1  # RUNNING
            current_data['stable'] = True
            current_data['progress'] = 100
        
        # Fill phase and stable history
        data_history['phase'].append(current_data['phase'])
        data_history['stable'].append(1 if current_data['stable'] else 0)

# ===== FLASK ROUTES =====
@app.route('/')
def index():
    """Serve the main dashboard"""
    return render_template('dashboard.html')

@app.route('/api/data', methods=['GET'])
def get_data():
    """API endpoint for dashboard data"""
    with data_lock:
        return jsonify({
            'current': current_data,
            'history': {
                'time': list(data_history['time']),
                'kp': list(data_history['kp']),
                'ki': list(data_history['ki']),
                'kd': list(data_history['kd']),
                'speed': list(data_history['speed']),
                'error': list(data_history['error']),
                'phase': list(data_history['phase']),
                'stable': list(data_history['stable'])
            }
        })

@app.route('/api/control', methods=['POST'])
def control():
    """Send commands to ESP"""
    try:
        command = request.json.get('command', '').lower()
        
        if not serial_thread or not serial_thread.running:
            return jsonify({'status': 'error', 'message': 'Not connected to ESP'})
        
        if command == 'start':
            if serial_thread.send_command('s'):
                return jsonify({'status': 'success', 'message': 'Start command sent to ESP'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to send command'})
                
        elif command == 'stop':
            if serial_thread.send_command('s'):
                return jsonify({'status': 'success', 'message': 'Stop command sent to ESP'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to send command'})
                
        elif command == 'tune':
            if serial_thread.send_command('d'):
                return jsonify({'status': 'success', 'message': 'Tuning command sent to ESP'})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to send command'})
        else:
            return jsonify({'status': 'error', 'message': f'Unknown command: {command}'})
            
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/api/ports', methods=['GET'])
def get_ports():
    """Get available serial ports"""
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return jsonify({'ports': ports})

@app.route('/api/connect', methods=['POST'])
def connect_esp():
    """Connect to ESP on specified port"""
    global serial_thread
    
    # Disconnect existing connection
    if serial_thread and serial_thread.running:
        serial_thread.stop()
        serial_thread = None
        time.sleep(0.5)
    
    try:
        port = request.json.get('port')
        if not port:
            return jsonify({'status': 'error', 'message': 'No port specified'})
        
        # Create and start serial thread
        serial_thread = SerialThread(port)
        serial_thread.start()
        
        # Give it time to connect
        time.sleep(1)
        
        return jsonify({
            'status': 'success', 
            'message': f'Connecting to {port}...',
            'port': port
        })
        
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/api/disconnect', methods=['POST'])
def disconnect_esp():
    """Disconnect from ESP"""
    global serial_thread
    
    if serial_thread:
        serial_thread.stop()
        serial_thread = None
    
    with data_lock:
        current_data['esp_connected'] = False
        current_data['connection_status'] = '🔴 DISCONNECTED'
        current_data['esp_port'] = 'None'
    
    return jsonify({'status': 'success', 'message': 'Disconnected from ESP'})

@app.route('/api/status')
def get_status():
    """Get system status"""
    with data_lock:
        return jsonify({
            'esp_connected': current_data['esp_connected'],
            'esp_port': current_data['esp_port'],
            'connection_status': current_data['connection_status'],
            'data_points': current_data['data_points'],
            'uptime': current_data['uptime']
        })

# ===== STATIC FILES =====
@app.route('/static/<path:path>')
def send_static(path):
    return send_from_directory('static', path)

# ===== INITIALIZATION =====
def initialize_data():
    """Initialize with some default data for graphs"""
    for i in range(10):
        t = i * 0.5
        data_history['time'].append(t)
        data_history['kp'].append(0.5)
        data_history['ki'].append(0.1)
        data_history['kd'].append(0.05)
        data_history['speed'].append(0)
        data_history['error'].append(0)
        data_history['phase'].append(0)
        data_history['stable'].append(1)

# ===== MAIN =====
if __name__ == '__main__':
    # Initialize with some data
    initialize_data()
    
    print("=" * 60)
    print("🌉 ESP8266 to Web Dashboard Bridge")
    print("=" * 60)
    print("🚀 Features:")
    print("   • Real-time ESP data to web dashboard")
    print("   • Start/Stop/Tune commands to ESP")
    print("   • Beautiful charts with Chart.js")
    print("   • Auto-refresh data every 400ms")
    print("=" * 60)
    print("📡 Available Serial Ports:")
    ports = serial.tools.list_ports.comports()
    if ports:
        for port in ports:
            print(f"   • {port.device} - {port.description}")
    else:
        print("   No serial ports found")
    print("=" * 60)
    print("🌐 Web Dashboard: http://localhost:5000")
    print("📱 Mobile Access: http://[YOUR_IP]:5000")
    print("⚡ Press Ctrl+C to stop server")
    print("=" * 60)
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n🛑 Server stopped by user")
        if serial_thread:
            serial_thread.stop()