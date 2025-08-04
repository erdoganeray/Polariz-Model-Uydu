import sys
import time
import threading
import cv2
import numpy as np
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QGridLayout, QTextEdit
from PyQt6.QtGui import QPixmap, QSurfaceFormat, QImage
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from OpenGL.GL import *
from OpenGL.GLU import *
import math

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("PySerial kütüphanesi bulunamadı!")
    print("Lütfen şu komutu çalıştırın: pip install pyserial")

# Set OpenGL format
format = QSurfaceFormat()
format.setDepthBufferSize(24)
format.setSamples(4)
format.setVersion(2, 1)  # Using OpenGL 2.1
QSurfaceFormat.setDefaultFormat(format)

class ArduinoDataReader(QObject):
    data_received = pyqtSignal(str)
    
    def __init__(self, port='COM6', baud_rate=9600):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.running = False
        self.ser = None
    
    def start(self):
        if not SERIAL_AVAILABLE:
            print("Serial module not available. Please install pyserial.")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self.read_data)
        self.thread.daemon = True
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
    
    def read_data(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to Arduino on {self.port}")
            
            while self.running:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.data_received.emit(line)
                time.sleep(0.1)
                
        except Exception as e:
            print(f"Error: Could not connect to Arduino on {self.port}.")
            print(f"Details: {e}")

class VideoCapture(QObject):
    frame_ready = pyqtSignal(QImage)
    
    def __init__(self, camera_index=1):
        super().__init__()
        self.camera_index = camera_index
        self.running = False
        self.cap = None
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.capture_video)
        self.thread.daemon = True
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()
    
    def capture_video(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        
        if not self.cap.isOpened():
            print("Error: Could not open video device")
            return
            
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # Convert frame to QImage
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                self.frame_ready.emit(qt_image)
            time.sleep(0.03)  # ~30 FPS

class GyroWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.lastPos = None
        # Rotation angles around each axis
        self.roll = 0.0   # Rotation around X-axis
        self.pitch = 0.0  # Rotation around Y-axis
        self.yaw = 0.0    # Rotation around Z-axis (pointing to Earth center)
        
        # Enable mouse tracking
        self.setMouseTracking(True)
        
    def initializeGL(self):
        glClearColor(0.2, 0.2, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        
        # Set up lighting from above
        glLightfv(GL_LIGHT0, GL_POSITION, [0, 0, 10, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1])
        
    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h, 0.1, 100.0)
        
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, 0, -10)
        
        # Apply rotations in the correct order: Roll (X) -> Pitch (Y) -> Yaw (Z)
        glRotatef(self.roll, 1, 0, 0)   # Roll - rotation around X
        glRotatef(self.pitch, 0, 1, 0)  # Pitch - rotation around Y
        glRotatef(self.yaw, 0, 0, 1)    # Yaw - rotation around Z (Earth center)
        
        # Draw cylinder
        quad = gluNewQuadric()
        glColor3f(0.8, 0.4, 0.4)  # Pink-ish color
        
        # Draw cylinder body - oriented vertically along Z-axis
        glPushMatrix()
        glRotatef(90, 1, 0, 0)  # Orient cylinder vertically
        glTranslatef(0, 0, -1.5)  # Center the cylinder
        gluCylinder(quad, 1, 1, 3, 32, 32)
        
        # Draw top and bottom caps
        glTranslatef(0, 0, 3)
        gluDisk(quad, 0, 1, 32, 1)
        glTranslatef(0, 0, -3)
        gluDisk(quad, 0, 1, 32, 1)
        glPopMatrix()
        
        # Draw coordinate axes at cylinder center
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        # X axis (red) - Roll axis
        glColor3f(1, 0, 0)
        glVertex3f(-2, 0, 0)
        glVertex3f(2, 0, 0)
        # Y axis (green) - Pitch axis
        glColor3f(0, 1, 0)
        glVertex3f(0, -2, 0)
        glVertex3f(0, 2, 0)
        # Z axis (blue) - Yaw axis (pointing to Earth center)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, -2)
        glVertex3f(0, 0, 2)
        glEnd()
        glEnable(GL_LIGHTING)
    
    def mouseMoveEvent(self, event):
        if self.lastPos is None:
            self.lastPos = event.pos()
            return
            
        # Calculate mouse movement relative to widget center
        center = self.rect().center()
        currentPos = event.pos()
        
        # Calculate relative position from center (-1 to 1 range)
        dx = (currentPos.x() - center.x()) / (self.width() / 2)
        dy = (currentPos.y() - center.y()) / (self.height() / 2)
        
        # Update rotations based on mouse position
        # Roll (X-axis) controlled by vertical mouse movement
        self.roll = dy * 45
        # Yaw (Z-axis) controlled by horizontal mouse movement
        self.yaw = -dx * 45
        
        self.update()
        self.lastPos = currentPos
    
    def leaveEvent(self, event):
        # Reset last position when mouse leaves widget
        self.lastPos = None

class GroundStationUI(QWidget):
    def __init__(self):
        super().__init__()
        self.temp_data = []
        self.time_data = []
        self.initUI()
        
        # Initialize Arduino reader
        self.arduino_reader = ArduinoDataReader()
        self.arduino_reader.data_received.connect(self.process_arduino_data)
        self.arduino_reader.start()
        
        # Initialize video capture
        self.video_capture = VideoCapture()
        self.video_capture.frame_ready.connect(self.update_camera_view)
        self.video_capture.start()
        
        # Setup timer for updating graphs
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graphs)
        self.timer.start(1000)  # Update every second
    
    def initUI(self):
        self.setWindowTitle("Ground Station")
        self.setGeometry(100, 100, 1000, 600)
        # Set window to fullscreen
        self.showFullScreen()
        
        layout = QGridLayout()
        
        # Temperature Graph
        self.temp_fig, self.temp_ax = plt.subplots()
        self.temp_canvas = FigureCanvas(self.temp_fig)
        self.temp_ax.set_title("Temperature Graph")
        layout.addWidget(self.temp_canvas, 0, 0)
        
        # Pressure Graph
        self.pressure_fig, self.pressure_ax = plt.subplots()
        self.pressure_canvas = FigureCanvas(self.pressure_fig)
        self.pressure_ax.set_title("Pressure Graph")
        layout.addWidget(self.pressure_canvas, 1, 0)
        
        # Altitude Graph
        self.alt_fig, self.alt_ax = plt.subplots()
        self.alt_canvas = FigureCanvas(self.alt_fig)
        self.alt_ax.set_title("Altitude Graph")
        layout.addWidget(self.alt_canvas, 2, 0)
        
        # Button Layout for Manual Separation and Close
        button_layout = QVBoxLayout()
        
        # Manual Separation Button
        self.sep_button = QPushButton("Manual Separation")
        self.sep_button.clicked.connect(self.manual_separation)
        button_layout.addWidget(self.sep_button)
        
        # Close Button
        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.close)
        button_layout.addWidget(self.close_button)
        
        # Add button layout to main layout
        layout.addLayout(button_layout, 0, 1)
        
        # Log Screen
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setPlaceholderText("Log screen")
        layout.addWidget(self.log_text, 1, 1, 2, 1)
        
        # Camera View - Increased size
        self.camera_label = QLabel("Camera")
        self.camera_label.setMinimumSize(400, 300)
        self.camera_label.setPixmap(QPixmap(400, 300))
        layout.addWidget(self.camera_label, 0, 2)
        
        # Replace Gyro Label with GyroWidget
        self.gyro_widget = GyroWidget()
        self.gyro_widget.setMinimumSize(400, 300)
        layout.addWidget(self.gyro_widget, 1, 2)
        
        self.setLayout(layout)
    
    def process_arduino_data(self, data):
        try:
            # Log all data received from Arduino
            self.log_text.append(f"Arduino: {data}")
            
            # Try to convert the data to a float for the temperature graph
            try:
                # First, check if the data contains "Temperature:" format
                if "Temperature:" in data:
                    temp_value = float(data.split(":")[1].strip())
                else:
                    # If not, try to convert the entire string to a float
                    temp_value = float(data.strip())
                
                # Add the temperature value to the graph data
                self.temp_data.append(temp_value)
                self.time_data.append(len(self.time_data))
                
                # Keep only the last 60 data points (1 minute of data)
                if len(self.temp_data) > 60:
                    self.temp_data.pop(0)
                    self.time_data.pop(0)
                
                # Log the processed temperature value
                self.log_text.append(f"Temperature: {temp_value}°C")
            except ValueError:
                # If we can't convert to float, just log the data without adding to graph
                print(f"Could not convert Arduino data to temperature value: {data}")
        except Exception as e:
            print(f"Error processing Arduino data: {e}")
            self.log_text.append(f"Error processing data: {e}")
    
    def update_graphs(self):
        if self.temp_data:
            self.temp_ax.clear()
            self.temp_ax.plot(self.time_data, self.temp_data, 'r-')
            self.temp_ax.set_title("Temperature Graph")
            self.temp_ax.set_ylabel("Temperature (°C)")
            self.temp_ax.set_xlabel("Time (s)")
            self.temp_canvas.draw()
    
    def update_camera_view(self, image):
        pixmap = QPixmap.fromImage(image)
        # Scale the pixmap to fit the label while maintaining aspect ratio
        pixmap = pixmap.scaled(self.camera_label.width(), self.camera_label.height(), 
                              Qt.AspectRatioMode.KeepAspectRatio)
        self.camera_label.setPixmap(pixmap)
    
    def manual_separation(self):
        self.log_text.append("Data sent")
        print("Data sent")
        
    def keyPressEvent(self, event):
        # Add ESC key to exit fullscreen
        if event.key() == Qt.Key.Key_Escape:
            self.close()
    
    def closeEvent(self, event):
        # Clean up resources when closing the application
        self.arduino_reader.stop()
        self.video_capture.stop()
        self.timer.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GroundStationUI()
    window.show()
    sys.exit(app.exec())