#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YIS Ground Station GUI
Model Uydu Projesi - Yer İstasyonu Arayüzü
"""

import sys
import os
import time
import random
import math
import csv
from datetime import datetime
from typing import Dict, List, Tuple
import threading

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QComboBox, QTextEdit, QFrame,
    QSizePolicy, QGroupBox, QTableWidget, QTableWidgetItem, QMessageBox
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont, QPixmap, QPalette, QColor, QImage
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import pyqtgraph as pg
import numpy as np
import serial
import serial.tools.list_ports
import cv2


class CameraWidget(QLabel):
    """USB Kamera Widget Sınıfı"""
    
    def __init__(self):
        super().__init__()
        self.camera = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.camera_index = -1
        
        # Video kaydetme değişkenleri
        self.video_writer = None
        self.is_recording = False
        self.recording_start_time = None
        
        # Data klasör yolu
        self.data_dir = os.path.join(os.path.dirname(__file__), "data")
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Placeholder ayarları
        self.setText("Kamera Aranıyor...")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("""
            QLabel {
                background-color: #e9ecef;
                color: #6c757d;
                border: 2px dashed #ced4da;
                border-radius: 8px;
                font-size: 14px;
                font-weight: 500;
            }
        """)
        self.setMinimumSize(200, 150)
        self.setScaledContents(True)
        
        # Kamera arama
        self.find_usb_camera()
        
    def find_usb_camera(self):
        """USB video yakalama cihazını otomatik olarak bul"""
        # Birden fazla kamera indexini dene (0-10 arası)
        for index in range(11):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                # Kamera özelliklerini kontrol et
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.camera_index = index
                    self.camera = cap
                    self.setText(f"Kamera Bulundu (Index: {index})")
                    print(f"USB Kamera bulundu: Index {index}")
                    break
                cap.release()
        
        if self.camera_index == -1:
            self.setText("USB Kamera\nBulunamadı")
            print("USB video yakalama cihazı bulunamadı")
        else:
            self.start_camera()
    
    def start_camera(self):
        """Kamera görüntü akışını başlat"""
        if self.camera and self.camera.isOpened():
            # Kamera çözünürlüğünü ayarla
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            # Video kaydını başlat
            self.start_recording()
            
            # Timer'ı başlat (30 FPS için ~33ms)
            self.timer.start(33)
            print(f"Kamera başlatıldı: {self.camera_index}")
    
    def start_recording(self):
        """Video kaydını başlat"""
        if not self.is_recording:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_filename = os.path.join(self.data_dir, f"video_{timestamp}.mp4")
            
            # Video writer'ı başlat
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, 30.0, (640, 480))
            
            if self.video_writer and self.video_writer.isOpened():
                self.is_recording = True
                self.recording_start_time = time.time()
                print(f"Video kaydetme başlatıldı: {video_filename}")
            else:
                print("Video kaydetme başlatılamadı!")
    
    def stop_recording(self):
        """Video kaydını durdur"""
        if self.is_recording and self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            self.is_recording = False
            recording_duration = time.time() - self.recording_start_time if self.recording_start_time else 0
            print(f"Video kaydetme durduruldu. Süre: {recording_duration:.1f} saniye")
    
    def update_frame(self):
        """Kamera frame'ini güncelle"""
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret and frame is not None:
                # Video kaydı aktifse frame'i kaydet
                if self.is_recording and self.video_writer:
                    self.video_writer.write(frame)
                
                # Frame'i QLabel boyutuna uygun şekilde yeniden boyutlandır
                height, width, channel = frame.shape
                bytes_per_line = 3 * width
                
                # BGR'den RGB'ye çevir (OpenCV BGR kullanır, Qt RGB kullanır)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # QImage oluştur
                q_image = QImage(rgb_frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
                
                # QPixmap'e çevir ve label'a ata
                pixmap = QPixmap.fromImage(q_image)
                
                # Widget boyutuna göre ölçekle (aspect ratio korunarak)
                scaled_pixmap = pixmap.scaled(
                    self.size(), 
                    Qt.AspectRatioMode.KeepAspectRatio, 
                    Qt.TransformationMode.SmoothTransformation
                )
                
                self.setPixmap(scaled_pixmap)
            else:
                # Frame okunamıyorsa bağlantıyı yeniden dene
                self.reconnect_camera()
    
    def reconnect_camera(self):
        """Kamera bağlantısını yeniden kur"""
        if self.camera:
            self.camera.release()
        
        # Kısa bir beklemeden sonra yeniden bağlanmayı dene
        QTimer.singleShot(1000, self.find_usb_camera)
    
    def stop_camera(self):
        """Kamera akışını durdur"""
        if self.timer.isActive():
            self.timer.stop()
        
        # Video kaydını durdur
        self.stop_recording()
        
        if self.camera:
            self.camera.release()
            self.camera = None
        
        self.setText("Kamera\nDurduruldu")
        print("Kamera durduruldu")
    
    def closeEvent(self, event):
        """Widget kapatılırken kamerayı temizle"""
        self.stop_camera()
        super().closeEvent(event)


class SatelliteModel3D(QOpenGLWidget):
    """3D Uydu Modeli - OpenGL Widget"""
    
    def __init__(self):
        super().__init__()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.setMinimumSize(200, 200)
        
        # Mouse tracking için
        self.setMouseTracking(True)
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        
    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.98, 0.98, 0.99, 1.0)  # Açık gri-beyaz arka plan
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        
        # Işık ayarları - daha parlak
        light_pos = [3.0, 3.0, 3.0, 1.0]
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos)
        
        # Ambient ışık ekle (genel aydınlatma)
        ambient_light = [0.3, 0.3, 0.3, 1.0]
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_light)
        
        # Diffuse ışık
        diffuse_light = [0.8, 0.8, 0.8, 1.0]
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light)
        
    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, width/height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Kamerayı daha iyi pozisyona yerleştir
        glTranslatef(0.0, 0.0, -4.0)  # Biraz daha yakın
        
        # Mouse pozisyonuna göre interaktif rotasyon (daha yumuşak)
        mouse_rotation_x = self.mouse_y * 25.0  # Dikey mouse hareketi X ekseni rotasyonu
        mouse_rotation_y = self.mouse_x * 25.0  # Yatay mouse hareketi Y ekseni rotasyonu
        
        # Mouse rotasyonlarını uygula
        glRotatef(mouse_rotation_x, 1.0, 0.0, 0.0)
        glRotatef(mouse_rotation_y, 0.0, 1.0, 0.0)
        
        # Varsayılan görünüm açısı (biraz eğik)
        glRotatef(15.0, 1.0, 0.0, 0.0)  # X ekseni etrafında 15 derece
        glRotatef(25.0, 0.0, 1.0, 0.0)  # Y ekseni etrafında 25 derece
        
        # Uydu rotasyonları (pitch, roll, yaw)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.roll, 0.0, 0.0, 1.0)
        glRotatef(self.yaw, 0.0, 1.0, 0.0)
        
        # Basit uydu modeli çiz
        self.draw_satellite()
        
    def draw_satellite(self):
        """Silindir şeklinde uydu modeli çizimi"""
        # Silindir parametreleri
        radius = 0.4
        height = 1.2
        slices = 20
        
        # Ana gövde (silindir) - Koyu gri metalik (açık arka plana uygun)
        glColor3f(0.35, 0.35, 0.35)  # Daha koyu gri
        
        # Silindir yan yüzeyi
        glBegin(GL_QUADS)
        for i in range(slices):
            angle1 = 2.0 * math.pi * i / slices
            angle2 = 2.0 * math.pi * (i + 1) / slices
            
            x1 = radius * math.cos(angle1)
            z1 = radius * math.sin(angle1)
            x2 = radius * math.cos(angle2)
            z2 = radius * math.sin(angle2)
            
            # Yan yüzey
            glVertex3f(x1, -height/2, z1)
            glVertex3f(x2, -height/2, z2)
            glVertex3f(x2, height/2, z2)
            glVertex3f(x1, height/2, z1)
        glEnd()
        
        # Üst kapak - Açık gri
        glColor3f(0.7, 0.7, 0.7)  # Açık gri
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0.0, height/2, 0.0)  # Merkez
        for i in range(slices + 1):
            angle = 2.0 * math.pi * i / slices
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)
            glVertex3f(x, height/2, z)
        glEnd()
        
        # Alt kapak - Koyu gri
        glColor3f(0.2, 0.2, 0.2)  # Koyu gri
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0.0, -height/2, 0.0)  # Merkez
        for i in range(slices + 1):
            angle = 2.0 * math.pi * i / slices
            x = radius * math.cos(angle)
            z = radius * math.sin(angle)
            glVertex3f(x, -height/2, z)
        glEnd()
        
        # Koordinat sistemi eksenleri (merkez noktada)
        glDisable(GL_LIGHTING)  # Işığı kapat ki çizgiler net görünsün
        glLineWidth(3.0)  # Kalın çizgiler
        glBegin(GL_LINES)
        
        # X ekseni - Kırmızı (sağ-sol)
        glColor3f(0.9, 0.2, 0.2)  # Parlak kırmızı
        glVertex3f(-0.8, 0.0, 0.0)
        glVertex3f(0.8, 0.0, 0.0)
        
        # Y ekseni - Yeşil (yukarı-aşağı)
        glColor3f(0.2, 0.8, 0.2)  # Parlak yeşil
        glVertex3f(0.0, -0.8, 0.0)
        glVertex3f(0.0, 0.8, 0.0)
        
        # Z ekseni - Mavi (ileri-geri)
        glColor3f(0.2, 0.4, 0.9)  # Parlak mavi
        glVertex3f(0.0, 0.0, -0.8)
        glVertex3f(0.0, 0.0, 0.8)
        
        glEnd()
        glEnable(GL_LIGHTING)  # Işığı tekrar aç
        
        # Yönelim belirteci - Üstte kırmızı nokta
        glColor3f(1.0, 0.3, 0.3)  # Parlak kırmızı
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0.0, height/2 + 0.05, 0.0)  # Merkez (biraz yukarıda)
        indicator_radius = 0.08
        for i in range(9):
            angle = 2.0 * math.pi * i / 8
            x = indicator_radius * math.cos(angle)
            z = indicator_radius * math.sin(angle)
            glVertex3f(x, height/2 + 0.05, z)
        glEnd()
        
        # Yönelim çizgisi - Önü gösteren turuncu çizgi
        glColor3f(1.0, 0.6, 0.1)  # Turuncu
        glBegin(GL_QUADS)
        line_width = 0.04
        line_length = 0.25
        # Z ekseni boyunca ön tarafa uzanan turuncu çizgi
        glVertex3f(-line_width/2, height/2 - 0.08, radius)
        glVertex3f(line_width/2, height/2 - 0.08, radius)
        glVertex3f(line_width/2, height/2 - 0.08, radius + line_length)
        glVertex3f(-line_width/2, height/2 - 0.08, radius + line_length)
        glEnd()
        
        # Sağ taraf belirteci - Sarı çizgi
        glColor3f(0.9, 0.9, 0.2)  # Sarı
        glBegin(GL_QUADS)
        # X ekseni boyunca sağ tarafa uzanan sarı çizgi
        glVertex3f(radius, height/2 - 0.15, -line_width/2)
        glVertex3f(radius + line_length, height/2 - 0.15, -line_width/2)
        glVertex3f(radius + line_length, height/2 - 0.15, line_width/2)
        glVertex3f(radius, height/2 - 0.15, line_width/2)
        glEnd()
        
    def update_orientation(self, pitch, roll, yaw):
        """Uydu yönelimini güncelle"""
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.update()
    
    def enterEvent(self, event):
        """Mouse widget'a girdiğinde"""
        self.setMouseTracking(True)
        super().enterEvent(event)
    
    def leaveEvent(self, event):
        """Mouse widget'tan çıktığında - rotasyonu sıfırla"""
        self.mouse_x = 0.0
        self.mouse_y = 0.0
        self.update()
        super().leaveEvent(event)
    
    def mouseMoveEvent(self, event):
        """Mouse hareket ettiğinde - cursor yönüne göre döndür"""
        # Widget'ın merkezini al
        center_x = self.width() / 2
        center_y = self.height() / 2
        
        # Mouse pozisyonunu merkeze göre normalize et (-1 ile 1 arası)
        self.mouse_x = (event.position().x() - center_x) / center_x
        self.mouse_y = (event.position().y() - center_y) / center_y
        
        # Sınırları kontrol et
        self.mouse_x = max(-1.0, min(1.0, self.mouse_x))
        self.mouse_y = max(-1.0, min(1.0, self.mouse_y))
        
        # Güncelle
        self.update()
        super().mouseMoveEvent(event)


class TelemetryData:
    """Telemetri veri sınıfı"""
    
    def __init__(self):
        self.paket_sayisi = 0
        self.uydu_statusu = 0
        self.hata_kodu = "222222"  # Başlangıçta 6-bit hata kodu (010101)
        self.gonderme_saati = ""
        self.basinc1 = 0.00  # 2 decimal places precision
        self.basinc2 = 0.00  # 2 decimal places precision
        self.yukseklik1 = 0.00  # 2 decimal places precision
        self.yukseklik2 = 0.00  # 2 decimal places precision
        self.irtifa_farki = 0.00  # 2 decimal places precision
        self.inis_hizi = -0.00  # 2 decimal places precision
        self.sicaklik = 00.00  # Temperature with 2 decimal places precision (sent as value*100)
        self.pil_gerilimi = 0.00  # Battery voltage with 2 decimal places precision
        self.gps1_latitude = 0.000000
        self.gps1_longitude = 0.000000
        self.gps1_altitude = 0.00  # GPS altitude with 2 decimal places precision
        self.pitch = 0.00  # Pitch with 2 decimal places precision
        self.roll = 0.00  # Roll with 2 decimal places precision
        self.yaw = 0.00  # Yaw with 2 decimal places precision
        self.rhrh = "0000"
        self.iot_s1_data = 22.50  # IoT temperature with 2 decimal places precision (sent as value*100)
        self.iot_s2_data = 23.10  # IoT temperature with 2 decimal places precision (sent as value*100)
        self.takim_no = 626541  # Takım numarası (corrected value)
        
    def simulate_update(self):
        """Simüle edilmiş veri güncellemesi"""
        self.paket_sayisi += 1
        
        # Rastgele değişimler
        self.basinc1 += random.uniform(-10, 10)
        self.basinc2 += random.uniform(-10, 10)
        self.yukseklik1 += random.uniform(-1, 1)
        self.yukseklik2 += random.uniform(-1, 1)
        self.irtifa_farki = abs(self.yukseklik1 - self.yukseklik2)
        self.inis_hizi += random.uniform(-0.5, 0.5)
        self.sicaklik += random.uniform(-0.5, 0.5)
        self.pil_gerilimi = max(3.0, self.pil_gerilimi + random.uniform(-0.05, 0.05))
        
        # Yönelim verileri
        self.pitch += random.uniform(-2, 2)
        self.roll += random.uniform(-2, 2)
        self.yaw += random.uniform(-2, 2)
        
        # IoT sıcaklık verileri
        self.iot_s1_data += random.uniform(-0.3, 0.3)
        self.iot_s2_data += random.uniform(-0.3, 0.3)


class GroundStationGUI(QMainWindow):
    """Ana Yer İstasyonu GUI Sınıfı"""
    
    def __init__(self):
        super().__init__()
        self.telemetry_data = TelemetryData()
        self.start_time = time.time()  # Başlangıç zamanını kaydet
        
        # Serial communication variables
        self.serial_connection = None
        self.is_listening = False
        self.listen_thread = None
        
        # Camera variable
        self.camera_widget = None
        
        # İstatistik takibi için değişkenler
        self.first_packet_number = None  # İlk gelen paket numarası
        self.last_packet_number = 0     # Son gelen paket numarası
        self.total_received_packets = 0  # Alınan toplam paket sayısı
        self.communication_started = False  # Haberleşme başladı mı?
        self.communication_start_time = None  # Haberleşme başlangıç zamanı
        
        # CSV kaydetme için değişkenler
        self.data_dir = os.path.join(os.path.dirname(__file__), "data")
        os.makedirs(self.data_dir, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        self.csv_header_written = False
        
        self.init_ui()
        self.setup_timers()
        self.refresh_com_ports()
        
        # Başlangıç hata kodu tablosunu güncelle
        self.update_error_display()
        
        # Başlangıç istatistiklerini güncelle
        self.update_statistics_display()
        
    def init_ui(self):
        """UI kurulumu"""
        self.setWindowTitle("YIS Ground Station - Model Uydu Kontrol Merkezi")
        self.setGeometry(100, 100, 1400, 900)
        
        # Ana widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Ana layout (yatay)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(5)  # Section'lar arası boşluk azaltıldı
        main_layout.setContentsMargins(5, 5, 5, 5)  # Kenar boşlukları azaltıldı
        
        # Sol panel (grafikler + log)
        left_panel = self.create_left_panel()
        
        # Sağ panel (kamera, harita, veri, 3D model, kontroller)
        right_panel = self.create_right_panel()
        
        # Panelleri ana layout'a ekle
        main_layout.addWidget(left_panel, 2)  # Sol panel için oran
        main_layout.addWidget(right_panel, 2)  # Sağ panel için oran
        
    def create_left_panel(self):
        """Sol panel oluşturma (Grafikler + Log)"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(3)  # Grafik ve log arası boşluk azaltıldı
        left_layout.setContentsMargins(3, 3, 3, 3)  # Kenar boşlukları azaltıldı
        
        # Grafikler bölümü (3x2 grid)
        graphs_group = QGroupBox("Telemetri Grafikleri")
        graphs_layout = QGridLayout(graphs_group)
        
        # 6 grafik oluştur
        self.graphs = {}
        graph_configs = [
            ("pressure", "Basınç (Pa)", 0, 0),
            ("altitude", "Yükseklik (m)", 0, 1),
            ("descent_speed", "İniş Hızı (m/s)", 1, 0),
            ("temperature", "Atmosfer Sıcaklığı (°C)", 1, 1),
            ("altitude_diff", "İrtifa Farkı (m)", 2, 0),
            ("iot_temp", "IoT Sıcaklık (°C)", 2, 1)
        ]
        
        for graph_id, title, row, col in graph_configs:
            plot_widget = pg.PlotWidget(title=title)
            plot_widget.setMinimumSize(200, 150)
            plot_widget.setLabel('left', title)
            plot_widget.setLabel('bottom', 'Zaman (saniye)')
            plot_widget.showGrid(True, True)
            
            # Modern açık tema için grafik stilleri
            plot_widget.setBackground('#ffffff')  # Beyaz arka plan
            plot_widget.getAxis('left').setPen('#495057')  # Koyu gri eksen
            plot_widget.getAxis('bottom').setPen('#495057')
            plot_widget.getAxis('left').setTextPen('#495057')
            plot_widget.getAxis('bottom').setTextPen('#495057')
            plot_widget.setTitle(title, color='#495057', size='12pt')
            
            # Çift çizgili grafikler için farklı yapı
            if graph_id in ['pressure', 'altitude', 'iot_temp']:
                # İki ayrı çizgi ve veri dizisi
                self.graphs[graph_id] = {
                    'widget': plot_widget,
                    'data_x': [],
                    'data_y1': [],  # İlk çizgi verisi
                    'data_y2': [],  # İkinci çizgi verisi
                }
                
                # Legend ekleme ve curve oluşturma
                if graph_id == 'pressure':
                    plot_widget.addLegend()
                    # Legend için isimler curve oluşturulurken verilir
                    self.graphs[graph_id]['curve1'] = plot_widget.plot(pen={'color': '#0d6efd', 'width': 2}, name='Yük (Basınç1)')
                    self.graphs[graph_id]['curve2'] = plot_widget.plot(pen={'color': '#dc3545', 'width': 2}, name='Taşıyıcı (Basınç2)')
                elif graph_id == 'altitude':
                    plot_widget.addLegend()
                    # Legend için isimler curve oluşturulurken verilir
                    self.graphs[graph_id]['curve1'] = plot_widget.plot(pen={'color': '#0d6efd', 'width': 2}, name='Yük (Yükseklik1)')
                    self.graphs[graph_id]['curve2'] = plot_widget.plot(pen={'color': '#dc3545', 'width': 2}, name='Taşıyıcı (Yükseklik2)')
                elif graph_id == 'iot_temp':
                    plot_widget.addLegend()
                    # Legend için isimler curve oluşturulurken verilir
                    self.graphs[graph_id]['curve1'] = plot_widget.plot(pen={'color': '#0d6efd', 'width': 2}, name='IoT Sensor 1')
                    self.graphs[graph_id]['curve2'] = plot_widget.plot(pen={'color': '#dc3545', 'width': 2}, name='IoT Sensor 2')
            else:
                # Tek çizgili grafikler için mevcut yapı
                self.graphs[graph_id] = {
                    'widget': plot_widget,
                    'data_x': [],
                    'data_y': [],
                    'curve': plot_widget.plot(pen={'color': '#0d6efd', 'width': 2})  # Mavi çizgi
                }
            
            graphs_layout.addWidget(plot_widget, row, col)
        
        # Log ekranı
        log_group = QGroupBox("Sistem Logları")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMinimumHeight(150)
        self.log_text.setMaximumHeight(200)
        self.log_text.setReadOnly(True)
        # Modern log stili - açık tema uyumlu
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #f8f9fa;
                color: #198754;
                border: 1px solid #ced4da;
                border-radius: 4px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 12px;
                padding: 8px;
            }
        """)
        log_layout.addWidget(self.log_text)
        
        # Sol paneli birleştir
        left_layout.addWidget(graphs_group, 3)
        left_layout.addWidget(log_group, 1)
        
        return left_widget
        
    def create_right_panel(self):
        """Sağ panel oluşturma"""
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(3)  # Section'lar arası boşluk azaltıldı
        right_layout.setContentsMargins(3, 3, 3, 3)  # Kenar boşlukları azaltıldı
        
        # Üst: Kamera ve Harita
        top_section = self.create_camera_map_section()
        
        # Orta: Data ve 3D Model
        middle_section = self.create_data_3d_section()
        
        # Alt: COM ve Logo
        bottom_section = self.create_com_textdata_section()
        
        right_layout.addWidget(top_section, 4)
        right_layout.addWidget(middle_section, 2)
        right_layout.addWidget(bottom_section, 1)
        
        return right_widget
        
    def create_camera_map_section(self):
        """Kamera ve Harita bölümü"""
        section_widget = QWidget()
        section_layout = QHBoxLayout(section_widget)
        section_layout.setSpacing(3)  # Kamera ve harita arası boşluk azaltıldı
        section_layout.setContentsMargins(3, 3, 3, 3)  # Kenar boşlukları azaltıldı
        
        # Kamera bölümü - Gerçek USB kamera
        camera_group = QGroupBox("Kamera")
        camera_layout = QVBoxLayout(camera_group)
        
        # Gerçek kamera widget'ı oluştur
        self.camera_widget = CameraWidget()
        camera_layout.addWidget(self.camera_widget)
        
        # Harita placeholder
        map_group = QGroupBox("Harita")
        map_layout = QVBoxLayout(map_group)
        map_placeholder = QLabel("Harita Görüntüsü\n(Placeholder)")
        map_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        map_placeholder.setStyleSheet("""
            QLabel {
                background-color: #e9ecef;
                color: #6c757d;
                border: 2px dashed #ced4da;
                border-radius: 8px;
                font-size: 14px;
                font-weight: 500;
            }
        """)
        map_placeholder.setMinimumSize(200, 150)
        map_layout.addWidget(map_placeholder)
        
        section_layout.addWidget(camera_group)
        section_layout.addWidget(map_group)
        
        return section_widget
        
    def create_data_3d_section(self):
        """Data ve 3D Model bölümü"""
        section_widget = QWidget()
        section_layout = QHBoxLayout(section_widget)
        section_layout.setSpacing(3)  # Data ve 3D model arası boşluk azaltıldı
        section_layout.setContentsMargins(3, 3, 3, 3)  # Kenar boşlukları azaltıldı
        
        # Data paneli
        data_panel = self.create_data_panel()
        
        # 3D Model
        model_group = QGroupBox("3D Uydu Modeli")
        model_layout = QVBoxLayout(model_group)
        
        self.satellite_3d = SatelliteModel3D()
        model_layout.addWidget(self.satellite_3d)
        
        section_layout.addWidget(data_panel, 1)
        section_layout.addWidget(model_group, 1)
        
        return section_widget
        
    def create_data_panel(self):
        """Data paneli oluşturma"""
        data_group = QGroupBox("Kontrol ve Durum")
        data_layout = QVBoxLayout(data_group)
        
        # Telekomut bölümü
        telecommand_group = QGroupBox("Telekomut")
        telecommand_layout = QVBoxLayout(telecommand_group)
        
        # 4 adet combo box
        combo_layout = QHBoxLayout()
        self.rhrh_combos = []
        for i in range(4):
            combo = QComboBox()
            if i % 2 == 0:  # Rakam (0. ve 2. index)
                combo.addItems([str(x) for x in range(21)])  # 0-20 arası sayılar
            else:  # Harf (1. ve 3. index)
                combo.addItems(['A','M', 'F', 'N', 'R', 'G', 'B', 'P', 'Y', 'C'])  # Büyük harfler
            self.rhrh_combos.append(combo)
            combo_layout.addWidget(combo)
        
        # Telekomut gönder butonu
        self.telecommand_btn = QPushButton("RHRH: Telekomut Gönder")
        self.telecommand_btn.clicked.connect(self.send_telecommand)
        
        telecommand_layout.addLayout(combo_layout)
        telecommand_layout.addWidget(self.telecommand_btn)
        
        # Hata kodu tablosu - QLabel ile HTML table
        error_group = QGroupBox("Hata Kodu")
        error_layout = QVBoxLayout(error_group)
        
        # HTML table olarak hata kodu tablosu
        self.error_table_label = QLabel()
        self.error_table_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.error_table_label.setMinimumHeight(70)
        self.error_table_label.setMaximumHeight(70)
        
        # Test amaçlı HTML tablosu - tüm hücreler sarı
        html_table = """
        <div style="display: flex; justify-content: center; align-items: center; height: 100%;">
            <table border="1" cellpadding="8" cellspacing="0" style="border-collapse: collapse; border: 1px solid #333;">
                <tr>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 1 &nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 2 &nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 3 &nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 4 &nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 5 &nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp; 6 &nbsp;</td>
                </tr>
                <tr>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                    <td style="background-color: yellow; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>
                </tr>
            </table>
        </div>
        """
        
        self.error_table_label.setText(html_table)
        self.error_table_label.setStyleSheet("border: 1px solid #ced4da; border-radius: 4px; background-color: white;")
        
        error_layout.addWidget(self.error_table_label)
        
        data_layout.addWidget(telecommand_group)
        data_layout.addWidget(error_group)
        
        return data_group
        
    def create_com_textdata_section(self):
        """COM ve YIS Logo & Text Data bölümü"""
        section_widget = QWidget()
        section_layout = QHBoxLayout(section_widget)
        section_layout.setSpacing(3)  # COM ve logo arası boşluk azaltıldı
        section_layout.setContentsMargins(3, 3, 3, 3)  # Kenar boşlukları azaltıldı
        
        # COM Kontrol paneli
        com_group = QGroupBox("Haberleşme Kontrolü")
        com_layout = QVBoxLayout(com_group)
        
        # COM Port Selection Frame
        com_frame = QWidget()
        com_frame_layout = QGridLayout(com_frame)
        com_frame_layout.setSpacing(5)  # Grid elementleri arası boşluk
        
        # COM Port
        com_label = QLabel("COM Port:")
        self.com_combo = QComboBox()
        self.com_combo.setMinimumWidth(120)
        self.com_combo.setMinimumHeight(26)  # Minimum yükseklik
        
        # Refresh button
        self.refresh_btn = QPushButton("Yenile")
        self.refresh_btn.setMaximumWidth(60)
        self.refresh_btn.setMinimumHeight(26)  # Consistent height
        self.refresh_btn.setMaximumHeight(26)
        self.refresh_btn.clicked.connect(self.refresh_com_ports)
        
        # Baud Rate
        baud_label = QLabel("Baud Rate:")
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '115200', '230400'])
        self.baud_combo.setCurrentText('115200')
        self.baud_combo.setMaximumWidth(100)
        self.baud_combo.setMinimumHeight(26)  # Minimum yükseklik
        
        # Control Buttons
        self.start_btn = QPushButton("Başlat")
        self.start_btn.clicked.connect(self.start_listening)
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #198754;
                color: white;
                font-weight: bold;
                padding: 4px 12px;
                border-radius: 4px;
                border: none;
                font-size: 12px;
                max-height: 28px;
            }
            QPushButton:hover {
                background-color: #157347;
            }
            QPushButton:pressed {
                background-color: #146c43;
            }
        """)
        
        self.stop_btn = QPushButton("Durdur")
        self.stop_btn.clicked.connect(self.stop_listening)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                padding: 4px 12px;
                border-radius: 4px;
                border: none;
                font-size: 12px;
                max-height: 28px;
            }
            QPushButton:hover:enabled {
                background-color: #c82333;
            }
            QPushButton:pressed:enabled {
                background-color: #bd2130;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)

        # Layout arrangement
        com_frame_layout.addWidget(com_label, 0, 0)
        com_frame_layout.addWidget(self.com_combo, 0, 1)
        com_frame_layout.addWidget(self.refresh_btn, 0, 2)
        
        com_frame_layout.addWidget(baud_label, 1, 0)
        com_frame_layout.addWidget(self.baud_combo, 1, 1)
        
        # Control buttons frame
        control_frame = QWidget()
        control_layout = QHBoxLayout(control_frame)
        control_layout.setSpacing(8)  # Butonlar arası boşluk
        control_layout.setContentsMargins(0, 5, 0, 5)  # Üst-alt boşluk
        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.stop_btn)
        
        # Connection status
        self.connection_status = QLabel("Durum: Bağlantı bekleniyor")
        self.connection_status.setStyleSheet("font-weight: 500; color: #6c757d; font-size: 12px;")
        self.connection_status.setMinimumHeight(20)  # Minimum yükseklik
        
        # Manuel ayrılma butonu
        self.manual_separation_btn = QPushButton("Manuel Ayrılma")
        self.manual_separation_btn.setMinimumHeight(32)  # Consistent height
        self.manual_separation_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                padding: 6px 16px;
                border-radius: 4px;
                border: none;
                font-size: 12px;
                min-height: 30px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
            QPushButton:pressed {
                background-color: #bd2130;
            }
        """)
        self.manual_separation_btn.clicked.connect(self.manual_separation)

        # Manuel birleşme butonu
        self.manual_reunion_btn = QPushButton("Manuel Birleşme")
        self.manual_reunion_btn.setMinimumHeight(32)  # Consistent height
        self.manual_reunion_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                padding: 6px 16px;
                border-radius: 4px;
                border: none;
                font-size: 12px;
                min-height: 30px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
            QPushButton:pressed {
                background-color: #1e7e34;
            }
        """)
        self.manual_reunion_btn.clicked.connect(self.manual_reunion)
        
        # Manuel komut butonları için frame
        manual_buttons_frame = QWidget()
        manual_buttons_layout = QHBoxLayout(manual_buttons_frame)
        manual_buttons_layout.setSpacing(8)
        manual_buttons_layout.setContentsMargins(0, 0, 0, 0)
        manual_buttons_layout.addWidget(self.manual_separation_btn)
        manual_buttons_layout.addWidget(self.manual_reunion_btn)
        
        # Haberleşme alt bölümünü doldur
        com_layout.addWidget(com_frame)
        com_layout.addWidget(control_frame)
        com_layout.addWidget(self.connection_status)
        com_layout.addWidget(manual_buttons_frame)
        
        # YIS Logo ve Text Data bölümü
        logo_textdata_group = QGroupBox("YIS Logo ve Text Data")
        logo_textdata_layout = QVBoxLayout(logo_textdata_group)
        
        # Text Data sub-section
        textdata_group = QGroupBox("Text Data")
        textdata_layout = QHBoxLayout(textdata_group)  # Yatay layout yapıyoruz
        textdata_layout.setSpacing(8)  # İki bölüm arası boşluk
        textdata_layout.setContentsMargins(8, 8, 8, 8)  # İç kenar boşlukları
        
        # Sol bölüm - Durum ve Temel Bilgiler
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(4)  # Label'lar arası boşluk
        left_layout.setContentsMargins(4, 4, 4, 4)  # İç kenar boşlukları
        
        # Sol bölüm bilgileri
        self.status_label1 = QLabel("Pil Gerilimi: 0.0V (--%)")  # Default 0V
        self.status_label1.setStyleSheet("font-weight: 500; color: #495057; font-size: 11px; padding: 2px 0px;")
        
        self.status_label2 = QLabel("Statü: 0 (Uçuşa Hazır)")  # Default 0
        self.status_label2.setStyleSheet("font-weight: 500; color: #198754; font-size: 11px; padding: 2px 0px;")
        
        self.stats_total_expected = QLabel("Toplam Beklenen: --")
        self.stats_total_expected.setStyleSheet("font-weight: 500; color: #495057; font-size: 11px; padding: 1px 0px;")
        
        self.stats_total_received = QLabel("Toplam Alınan: 0")
        self.stats_total_received.setStyleSheet("font-weight: 500; color: #495057; font-size: 11px; padding: 1px 0px;")
        
        # Sol bölüm layout'ına elementleri ekle
        left_layout.addWidget(self.status_label1)
        left_layout.addWidget(self.status_label2)
        left_layout.addWidget(self.stats_total_expected)
        left_layout.addWidget(self.stats_total_received)
        left_layout.addStretch()  # Boşluk doldurma
        
        # Sağ bölüm - İstatistik Sonuçları
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(4)  # Label'lar arası boşluk
        right_layout.setContentsMargins(4, 4, 4, 4)  # İç kenar boşlukları
        
        # Sağ bölüm bilgileri
        self.stats_lost_packets = QLabel("Kayıp Paket: --")
        self.stats_lost_packets.setStyleSheet("font-weight: 500; color: #dc3545; font-size: 11px; padding: 1px 0px;")
        
        self.stats_success_rate = QLabel("Başarı Oranı: --%")
        self.stats_success_rate.setStyleSheet("font-weight: 500; color: #198754; font-size: 11px; padding: 1px 0px;")
        
        self.stats_data_rate = QLabel("Veri Hızı: 0.00 veri/sn")
        self.stats_data_rate.setStyleSheet("font-weight: 500; color: #0d6efd; font-size: 11px; padding: 1px 0px;")
        
        # Sağ bölüm layout'ına elementleri ekle
        right_layout.addWidget(self.stats_lost_packets)
        right_layout.addWidget(self.stats_success_rate)
        right_layout.addWidget(self.stats_data_rate)
        right_layout.addStretch()  # Boşluk doldurma
        
        # Ana Text Data layout'ına iki bölümü ekle
        textdata_layout.addWidget(left_widget, 1)   # Sol bölüm 1 birim
        textdata_layout.addWidget(right_widget, 1)  # Sağ bölüm 1 birim
        
        # Logo sub-section
        logo_group = QGroupBox("YIS Logo")
        logo_layout = QVBoxLayout(logo_group)
        
        self.logo_label = QLabel()
        logo_path = os.path.join(os.path.dirname(__file__), "yis_logo.png")
        
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            scaled_pixmap = pixmap.scaled(200, 150, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.logo_label.setPixmap(scaled_pixmap)
        else:
            self.logo_label.setText("YIS LOGO\n(yis_logo.png\nbulunamadı)")
            self.logo_label.setStyleSheet("""
                QLabel {
                    background-color: #e9ecef;
                    color: #6c757d;
                    border: 2px dashed #ced4da;
                    border-radius: 8px;
                    font-size: 12px;
                    font-weight: 500;
                }
            """)
            
        self.logo_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo_layout.addWidget(self.logo_label)
        
        # Text Data ve Logo'yu birleştir
        logo_textdata_layout.addWidget(textdata_group, 2)  # Text Data üstte, 2 birim (artık daha fazla yer kaplıyor)
        logo_textdata_layout.addWidget(logo_group, 1)      # Logo altta, 1 birim
        
        section_layout.addWidget(com_group, 2)  # Haberleşme kontrolü
        section_layout.addWidget(logo_textdata_group, 2)  # Logo ve Text Data bölümü
        
        return section_widget
        
    def setup_timers(self):
        """Timer'ları kurma"""
        # Veri güncelleme timer'ı (1 saniyede bir)
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_data)
        self.data_timer.start(1000)
        
        # 3D model güncelleme timer'ı (100ms)
        self.model_timer = QTimer()
        self.model_timer.timeout.connect(self.update_3d_model)
        self.model_timer.start(100)
        
    def update_data(self):
        """Verileri güncelleme - sadece simülasyon için değil gerçek veri gelirse durdur"""
        # Simülasyon kaldırıldı - sadece gerçek COM verisiyle çalışacak
        pass
        
    def update_graph(self, graph_id, x_value, y_value1, y_value2=None):
        """Grafik güncelleme - tek veya çift çizgi desteği"""
        graph = self.graphs[graph_id]
        
        if graph_id in ['pressure', 'altitude', 'iot_temp'] and y_value2 is not None:
            # Çift çizgili grafikler
            graph['data_x'].append(x_value)
            graph['data_y1'].append(y_value1)
            graph['data_y2'].append(y_value2)
            
            # Son 30 saniye verilerini tut
            while len(graph['data_x']) > 0 and graph['data_x'][-1] - graph['data_x'][0] > 30:
                graph['data_x'].pop(0)
                graph['data_y1'].pop(0)
                graph['data_y2'].pop(0)
                
            # Her iki çizgiyi güncelle
            graph['curve1'].setData(graph['data_x'], graph['data_y1'])
            graph['curve2'].setData(graph['data_x'], graph['data_y2'])
        else:
            # Tek çizgili grafikler (mevcut sistem)
            graph['data_x'].append(x_value)
            graph['data_y'].append(y_value1)
            
            # Son 30 saniye verilerini tut
            while len(graph['data_x']) > 0 and graph['data_x'][-1] - graph['data_x'][0] > 30:
                graph['data_x'].pop(0)
                graph['data_y'].pop(0)
                
            # Grafiği güncelle
            graph['curve'].setData(graph['data_x'], graph['data_y'])
        
    def update_3d_model(self):
        """3D modeli güncelleme"""
        self.satellite_3d.update_orientation(
            self.telemetry_data.pitch,
            self.telemetry_data.roll, 
            self.telemetry_data.yaw
        )
        
    def update_error_display(self):
        """Hata kodu tablosunu güncelleme - HTML tablosu ile"""
        error_code = self.telemetry_data.hata_kodu
        
        # HTML tablosu oluştur - ortalı ve çerçeveli
        html_table = '<div style="display: flex; justify-content: center; align-items: center; height: 100%;"><table border="1" cellpadding="8" cellspacing="0" style="border-collapse: collapse; border: 1px solid #333;">'
        
        # Üst satır (başlıklar)
        html_table += '<tr>'
        for i in range(6):
            html_table += f'<td style="background-color: lightgray; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; font-weight: bold; border: 1px solid #333; padding: 8px;">&nbsp; {i+1} &nbsp;</td>'
        html_table += '</tr>'
        
        # Alt satır (durum renkleri)
        html_table += '<tr>'
        for i in range(6):
            # Renk belirleme
            if error_code == "222222":
                # Default durum - veri alınmıyor, tüm hücreler sarı
                color = "yellow"
            else:
                # Gerçek veri alınıyor - hata koduna göre renklendirme
                if len(error_code) >= 6 and i < len(error_code):
                    if error_code[i] == '0':
                        color = "lime"     # 0 = normal (yeşil)
                    elif error_code[i] == '1':
                        color = "red"      # 1 = hata var (kırmızı)
                    else:
                        color = "yellow"   # Geçersiz değer (sarı)
                else:
                    color = "yellow"       # Veri eksik (sarı)
            
            html_table += f'<td style="background-color: {color}; text-align: center; min-width: 100px; max-width: 100px; width: 100px; height: 25px; border: 1px solid #333; padding: 8px;">&nbsp;</td>'
        
        html_table += '</tr>'
        html_table += '</table></div>'
        
        # HTML tablosunu label'a ata
        self.error_table_label.setText(html_table)
                
    def send_telecommand(self):
        """Telekomut gönderme"""
        rhrh_code = ''.join([combo.currentText() for combo in self.rhrh_combos])
        self.add_log_message(f"Telekomut gönderildi: RHRH={rhrh_code}")
        
        # Serial bağlantı varsa RHRH komutunu gönder
        if self.is_listening and self.serial_connection:
            try:
                # RHRH butonuna basıldığında manuel_ayrilma mutlaka 0 olmalı
                # Önce manuel ayrılma değerini sıfırla
                self.serial_connection.write(b'RESET_MANUEL\n')
                self.add_log_message("Manuel değeri 0'a sıfırlandı (RHRH komutu öncesi)")
                
                # Kısa bekleme
                time.sleep(0.05)
                
                # RHRH değerini güncelle
                rhrh_command = f'RHRH:{rhrh_code}\n'
                self.serial_connection.write(rhrh_command.encode())
                self.add_log_message(f"RHRH komutu LoRa'ya gönderildi: {rhrh_code}")
                
                # Kısa bir bekleme sonra SEND komutunu gönder
                time.sleep(0.1)
                self.serial_connection.write(b'SEND\n')
                self.add_log_message("SEND komutu LoRa'ya gönderildi (RHRH tetikleyicisi)")
            except serial.SerialException as e:
                self.add_log_message(f"RHRH/SEND komutu gönderme hatası: {str(e)}")
        else:
            self.add_log_message("UYARI: COM bağlantısı yok, komut gönderilemedi!")
        
    def manual_separation(self):
        """Manuel ayrılma komutu"""
        self.add_log_message("UYARI: Manuel ayrılma komutu gönderildi!")
        
        # Serial bağlantı varsa SEND komutunu gönder
        if self.is_listening and self.serial_connection:
            try:
                self.serial_connection.write(b'SEND\n')
                self.add_log_message("SEND komutu LoRa'ya gönderildi (Manuel ayrılma tetikleyicisi)")
            except serial.SerialException as e:
                self.add_log_message(f"SEND komutu gönderme hatası: {str(e)}")
        else:
            self.add_log_message("UYARI: COM bağlantısı yok, komut gönderilemedi!")
        
    def refresh_com_ports(self):
        """Available COM ports listesini yenile"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        
        self.com_combo.clear()
        self.com_combo.addItems(port_list)
        if port_list:
            self.com_combo.setCurrentIndex(0)
            
    def start_csv_logging(self):
        """CSV kaydetme başlat"""
        if not self.csv_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = os.path.join(self.data_dir, f"telemetri_{timestamp}.csv")
            
            try:
                self.csv_file = open(csv_filename, 'w', newline='', encoding='utf-8')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_header_written = False
                self.add_log_message(f"CSV kaydetme başlatıldı: {csv_filename}")
            except Exception as e:
                self.add_log_message(f"CSV dosyası oluşturulamadı: {str(e)}")
    
    def stop_csv_logging(self):
        """CSV kaydetme durdur"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.csv_header_written = False
            self.add_log_message("CSV kaydetme durduruldu")
    
    def write_telemetry_to_csv(self):
        """Telemetri verisini CSV'ye yaz"""
        if not self.csv_writer:
            return
        
        try:
            # Header'ı ilk seferde yaz
            if not self.csv_header_written:
                header = [
                    'paket_sayisi', 'uydu_statusu', 'hata_kodu', 'gonderme_saati',
                    'basinc1', 'basinc2', 'yukseklik1', 'yukseklik2', 'irtifa_farki',
                    'inis_hizi', 'sicaklik', 'pil_gerilimi', 'gps1_latitude', 'gps1_longitude',
                    'gps1_altitude', 'pitch', 'roll', 'yaw', 'rhrh', 'iot_s1_data', 'iot_s2_data', 'takim_no'
                ]
                self.csv_writer.writerow(header)
                self.csv_header_written = True
            
            # Veri satırını yaz
            row = [
                self.telemetry_data.paket_sayisi,
                self.telemetry_data.uydu_statusu,
                self.telemetry_data.hata_kodu,
                self.telemetry_data.gonderme_saati,
                self.telemetry_data.basinc1,
                self.telemetry_data.basinc2,
                self.telemetry_data.yukseklik1,
                self.telemetry_data.yukseklik2,
                self.telemetry_data.irtifa_farki,
                self.telemetry_data.inis_hizi,
                self.telemetry_data.sicaklik,
                self.telemetry_data.pil_gerilimi,
                self.telemetry_data.gps1_latitude,
                self.telemetry_data.gps1_longitude,
                self.telemetry_data.gps1_altitude,
                self.telemetry_data.pitch,
                self.telemetry_data.roll,
                self.telemetry_data.yaw,
                self.telemetry_data.rhrh,
                self.telemetry_data.iot_s1_data,
                self.telemetry_data.iot_s2_data,
                self.telemetry_data.takim_no
            ]
            self.csv_writer.writerow(row)
            self.csv_file.flush()  # Dosyaya anında yaz
            
        except Exception as e:
            self.add_log_message(f"CSV yazma hatası: {str(e)}")
            
    def start_listening(self):
        """COM port dinlemeyi başlat"""
        if self.com_combo.count() == 0:
            QMessageBox.critical(self, "Hata", "Lütfen bir COM port seçin!")
            return
            
        com_port = self.com_combo.currentText().split(" - ")[0]
        baud_rate = int(self.baud_combo.currentText())
        
        try:
            self.serial_connection = serial.Serial(
                port=com_port,
                baudrate=baud_rate,
                timeout=1
            )
            
            # İstatistik değişkenlerini sıfırla
            self.first_packet_number = None
            self.last_packet_number = 0
            self.total_received_packets = 0
            self.communication_started = False
            self.communication_start_time = None
            self.update_statistics_display()
            
            # CSV kaydetme başlat
            self.start_csv_logging()
            
            self.is_listening = True
            self.listen_thread = threading.Thread(target=self.listen_to_serial, daemon=True)
            self.listen_thread.start()
            
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.com_combo.setEnabled(False)
            self.baud_combo.setEnabled(False)
            self.refresh_btn.setEnabled(False)
            
            self.connection_status.setText(f"Durum: {com_port} dinleniyor...")
            self.connection_status.setStyleSheet("font-weight: 500; color: #198754; font-size: 12px;")
            self.add_log_message(f"COM port bağlantısı başarılı: {com_port} @ {baud_rate} baud")
            
        except serial.SerialException as e:
            QMessageBox.critical(self, "Bağlantı Hatası", f"COM port açılamadı: {str(e)}")
            self.add_log_message(f"COM port bağlantı hatası: {str(e)}")
            
    def stop_listening(self):
        """COM port dinlemeyi durdur"""
        self.is_listening = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        # CSV kaydetme durdur
        self.stop_csv_logging()
            
        # İstatistik değişkenlerini sıfırla
        self.first_packet_number = None
        self.last_packet_number = 0
        self.total_received_packets = 0
        self.communication_started = False
        self.update_statistics_display()
            
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.com_combo.setEnabled(True)
        self.baud_combo.setEnabled(True)
        self.refresh_btn.setEnabled(True)
        
        self.connection_status.setText("Durum: Bağlantı durduruldu")
        self.connection_status.setStyleSheet("font-weight: 500; color: #6c757d; font-size: 12px;")
        self.add_log_message("COM port bağlantısı kapatıldı")
        
    def listen_to_serial(self):
        """Serial port'u dinle ve gelen verileri logla"""
        self.add_log_message("LoRa ESP32 dinleme başladı...")
        
        while self.is_listening and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    # Read line from serial
                    data = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    
                    if data:
                        # Telemetri verisi kontrolü ve parse işlemi
                        if self.parse_telemetry_data(data):
                            self.add_log_message(f"Telemetri: {data}")
                        elif any(keyword in data for keyword in ["BTN,", "REQ,", "PRS,", "L4DATA,", "L5DATA,"]):
                            self.add_log_message(f"LoRa Verisi: {data}")
                        elif "binary" in data.lower() and ("gonderildi" in data.lower() or "alindi" in data.lower()):
                            self.add_log_message(f"LoRa İletişim: {data}")
                        else:
                            # Diğer serial mesajları
                            if len(data.strip()) > 0:
                                self.add_log_message(f"Serial: {data}")
                            
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except serial.SerialException as e:
                self.add_log_message(f"Serial okuma hatası: {str(e)}")
                break
            except Exception as e:
                self.add_log_message(f"Beklenmeyen hata: {str(e)}")
                break
                
        self.add_log_message("Serial dinleme durduruldu")
    
    def parse_telemetry_data(self, data_string):
        """Parse telemetry data and update graphs
        
        Data format from lora_2.ino:
        - basinc1, basinc2: 2 decimal places precision
        - sicaklik, iot_s1_data, iot_s2_data: 2 decimal places precision (values sent as temp*100)
        - hata_kodu: 6-bit binary string (e.g., "010101")
        """
        try:
            # Remove TEL, prefix if exists, or work with raw data
            if data_string.startswith("TEL,"):
                data_string = data_string[4:]  # Remove "TEL," prefix
            
            # Expected format after TEL,: "paket_sayisi,uydu_statusu,hata_kodu,gonderme_saati,basinc1,basinc2,yukseklik1,yukseklik2,irtifa_farki,inis_hizi,sicaklik,pil_gerilimi,gps1_latitude,gps1_longitud,gps1_altitude,pitch,roll,yaw,rhrh,iot_s1_data,iot_s2_data,takim_no"
            parts = data_string.split(',')
            
            if len(parts) >= 22:  # 22 because we removed TEL, prefix
                try:
                    # Parse the telemetry values (indexes shifted by -1 since TEL, is removed)
                    paket_sayisi = int(parts[0])
                    uydu_statusu = int(parts[1])
                    hata_kodu = parts[2]
                    gonderme_saati = parts[3]
                    basinc1 = float(parts[4])
                    basinc2 = float(parts[5])
                    yukseklik1 = float(parts[6])
                    yukseklik2 = float(parts[7])
                    irtifa_farki = float(parts[8])
                    inis_hizi = float(parts[9])
                    sicaklik = float(parts[10])
                    pil_gerilimi = float(parts[11])
                    gps1_latitude = float(parts[12])
                    gps1_longitude = float(parts[13])
                    gps1_altitude = float(parts[14])
                    pitch = float(parts[15])
                    roll = float(parts[16])
                    yaw = float(parts[17])
                    rhrh = parts[18]
                    iot_s1_data = float(parts[19])
                    iot_s2_data = float(parts[20])
                    takim_no = int(parts[21])  # Takım numarası eklendi
                    
                    # Update telemetry data
                    self.telemetry_data.paket_sayisi = paket_sayisi
                    self.telemetry_data.uydu_statusu = uydu_statusu
                    self.telemetry_data.hata_kodu = hata_kodu
                    self.telemetry_data.gonderme_saati = gonderme_saati
                    self.telemetry_data.basinc1 = basinc1
                    self.telemetry_data.basinc2 = basinc2
                    self.telemetry_data.yukseklik1 = yukseklik1
                    self.telemetry_data.yukseklik2 = yukseklik2
                    self.telemetry_data.irtifa_farki = irtifa_farki
                    self.telemetry_data.inis_hizi = inis_hizi
                    self.telemetry_data.sicaklik = sicaklik
                    self.telemetry_data.pil_gerilimi = pil_gerilimi
                    self.telemetry_data.gps1_latitude = gps1_latitude
                    self.telemetry_data.gps1_longitude = gps1_longitude
                    self.telemetry_data.gps1_altitude = gps1_altitude
                    self.telemetry_data.pitch = pitch
                    self.telemetry_data.roll = roll
                    self.telemetry_data.yaw = yaw
                    self.telemetry_data.rhrh = rhrh
                    self.telemetry_data.iot_s1_data = iot_s1_data
                    self.telemetry_data.iot_s2_data = iot_s2_data
                    self.telemetry_data.takim_no = takim_no  # Takım numarası eklendi
                    
                    # İstatistik güncellemesi
                    if not self.communication_started:
                        # İlk paket geldi, istatistik takibini başlat
                        self.first_packet_number = paket_sayisi
                        self.communication_started = True
                        self.communication_start_time = time.time()
                        self.add_log_message(f"İstatistik takibi başlatıldı. İlk paket numarası: {paket_sayisi}")
                    
                    # Paket istatistiklerini güncelle
                    self.last_packet_number = paket_sayisi
                    self.total_received_packets += 1
                    
                    # İstatistik görüntüsünü güncelle
                    self.update_statistics_display()
                    
                    # Add timestamp (relative to start time)
                    current_time = time.time() - self.start_time
                    
                    # Update graphs with real data
                    self.update_graph('pressure', current_time, basinc1, basinc2)  # Çift çizgi: Yük ve Taşıyıcı basınç
                    self.update_graph('altitude', current_time, yukseklik1, yukseklik2)  # Çift çizgi: Yük ve Taşıyıcı yükseklik
                    self.update_graph('descent_speed', current_time, inis_hizi)  # Tek çizgi
                    self.update_graph('temperature', current_time, sicaklik)  # Tek çizgi
                    self.update_graph('altitude_diff', current_time, irtifa_farki)  # Tek çizgi
                    self.update_graph('iot_temp', current_time, iot_s1_data, iot_s2_data)  # Çift çizgi: IoT1 ve IoT2
                    
                    # Telemetri verisini CSV dosyasına kaydet
                    self.write_telemetry_to_csv()
                    
                    # Update status displays
                    # Pil gerilimi: 4.2V ile 3.3V arasında yüzde hesapla
                    if pil_gerilimi <= 0:
                        # 0V veya negatif değerler için özel durum
                        self.status_label1.setText("Pil Gerilimi: 0.0V (--%)") 
                    else:
                        battery_percent = int((pil_gerilimi - 3.3) / (4.2 - 3.3) * 100)
                        battery_percent = max(0, min(100, battery_percent))  # 0-100 arası sınırla
                        self.status_label1.setText(f"Pil Gerilimi: {pil_gerilimi:.1f}V ({battery_percent}%)")
                    
                    # Uydu statusu metni
                    status_texts = {
                        0: "Uçuşa Hazır",
                        1: "Yükselme", 
                        2: "Model Uydu İniş",
                        3: "Ayrılma",
                        4: "Görev Yükü İniş",
                        5: "Kurtarma"
                    }
                    status_text = status_texts.get(uydu_statusu, f"Bilinmeyen ({uydu_statusu})")
                    self.status_label2.setText(f"Statü: {uydu_statusu} ({status_text})")
                    
                    # Hata kodu tablosunu güncelle
                    self.update_error_display()
                    
                    return True
                except ValueError as ve:
                    return False
            else:
                # Check if this looks like telemetry data (starts with a number)
                if data_string and data_string[0].isdigit() and ',' in data_string:
                    # This might be telemetry data without TEL, prefix
                    return self.parse_telemetry_data(data_string)
                return False
        except Exception as e:
            return False
        return False
        
    def manual_separation(self):
        """Manuel ayrılma komutu"""
        self.add_log_message("UYARI: Manuel ayrılma komutu gönderildi!")
        
        # Serial bağlantı varsa manuel ayrılma komutunu gönder
        if self.is_listening and self.serial_connection:
            try:
                # Manuel ayrılma/birleşme butonlarına basıldığında RHRH değeri mutlaka "0A0A" olmalı
                # Önce RHRH değerini 0A0A yap
                self.serial_connection.write(b'RHRH:0A0A\n')
                self.add_log_message("RHRH değeri 0A0A olarak ayarlandı (Manuel ayrılma öncesi)")
                
                # Kısa bekleme
                time.sleep(0.05)
                
                # Manuel ayrılma değerini 1 yap
                self.serial_connection.write(b'MANUEL_AYRILMA\n')
                self.add_log_message("Manuel ayrılma komutu LoRa'ya gönderildi (değer: 1)")
                
                # Kısa bir bekleme sonra SEND komutunu gönder
                time.sleep(0.1)
                self.serial_connection.write(b'SEND\n')
                self.add_log_message("SEND komutu LoRa'ya gönderildi (Manuel ayrılma tetikleyicisi)")
            except serial.SerialException as e:
                self.add_log_message(f"Manuel ayrılma komutu gönderme hatası: {str(e)}")
        else:
            self.add_log_message("UYARI: COM bağlantısı yok, komut gönderilemedi!")
    
    def manual_reunion(self):
        """Manuel birleşme komutu"""
        self.add_log_message("UYARI: Manuel birleşme komutu gönderildi!")
        
        # Serial bağlantı varsa manuel birleşme komutunu gönder
        if self.is_listening and self.serial_connection:
            try:
                # Manuel ayrılma/birleşme butonlarına basıldığında RHRH değeri mutlaka "0A0A" olmalı
                # Önce RHRH değerini 0A0A yap
                self.serial_connection.write(b'RHRH:0A0A\n')
                self.add_log_message("RHRH değeri 0A0A olarak ayarlandı (Manuel birleşme öncesi)")
                
                # Kısa bekleme
                time.sleep(0.05)
                
                # Manuel birleşme değerini 2 yap
                self.serial_connection.write(b'MANUEL_BIRLESME\n')
                self.add_log_message("Manuel birleşme komutu LoRa'ya gönderildi (değer: 2)")
                
                # Kısa bir bekleme sonra SEND komutunu gönder
                time.sleep(0.1)
                self.serial_connection.write(b'SEND\n')
                self.add_log_message("SEND komutu LoRa'ya gönderildi (Manuel birleşme tetikleyicisi)")
            except serial.SerialException as e:
                self.add_log_message(f"Manuel birleşme komutu gönderme hatası: {str(e)}")
        else:
            self.add_log_message("UYARI: COM bağlantısı yok, komut gönderilemedi!")
        
    def add_log_message(self, message):
        """Log mesajı ekleme"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_text.append(formatted_message)
        
        # Log'u otomatik kaydır
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
    def closeEvent(self, event):
        """Uygulama kapatılırken çağrılan fonksiyon"""
        if self.is_listening:
            self.stop_listening()
        
        # CSV kaydetme durdur
        self.stop_csv_logging()
        
        # Kamerayı kapat
        if self.camera_widget:
            self.camera_widget.stop_camera()
        
        event.accept()


    def update_statistics_display(self):
        """İstatistik görüntüsünü güncelle"""
        if not self.communication_started or self.first_packet_number is None:
            # Henüz haberleşme başlamadı
            self.stats_total_expected.setText("Toplam Beklenen: --")
            self.stats_total_received.setText(f"Toplam Alınan: {self.total_received_packets}")
            self.stats_lost_packets.setText("Kayıp Paket: --")
            self.stats_success_rate.setText("Başarı Oranı: --%")
            self.stats_data_rate.setText("Veri Hızı: 0.00 veri/sn")
        else:
            # Haberleşme başladı, istatistikleri hesapla
            expected_packets = self.last_packet_number - self.first_packet_number + 1
            lost_packets = expected_packets - self.total_received_packets
            
            if expected_packets > 0:
                success_rate = (self.total_received_packets / expected_packets) * 100
            else:
                success_rate = 0
            
            # Veri hızı hesapla (saniyede kaç veri)
            if self.communication_start_time is not None:
                elapsed_time = time.time() - self.communication_start_time
                if elapsed_time > 0:
                    data_rate = self.total_received_packets / elapsed_time
                else:
                    data_rate = 0
            else:
                data_rate = 0
            
            # Label'ları güncelle
            self.stats_total_expected.setText(f"Toplam Beklenen: {expected_packets}")
            self.stats_total_received.setText(f"Toplam Alınan: {self.total_received_packets}")
            
            # Kayıp paket sayısına göre renk ayarla
            if lost_packets > 0:
                self.stats_lost_packets.setText(f"Kayıp Paket: {lost_packets}")
                self.stats_lost_packets.setStyleSheet("font-weight: 500; color: #dc3545; font-size: 11px; padding: 1px 0px;")
            else:
                self.stats_lost_packets.setText("Kayıp Paket: 0")
                self.stats_lost_packets.setStyleSheet("font-weight: 500; color: #198754; font-size: 11px; padding: 1px 0px;")
            
            # Başarı oranına göre renk ayarla
            if success_rate >= 90:
                color = "#198754"  # Yeşil
            elif success_rate >= 70:
                color = "#ffc107"  # Sarı
            else:
                color = "#dc3545"  # Kırmızı
                
            self.stats_success_rate.setText(f"Başarı Oranı: {success_rate:.1f}%")
            self.stats_success_rate.setStyleSheet(f"font-weight: 500; color: {color}; font-size: 11px; padding: 1px 0px;")
            
            # Veri hızını göster (noktadan sonra 2 basamak)
            self.stats_data_rate.setText(f"Veri Hızı: {data_rate:.2f} veri/sn")
            self.stats_data_rate.setStyleSheet("font-weight: 500; color: #0d6efd; font-size: 11px; padding: 1px 0px;")


def main():
    """Ana fonksiyon"""
    app = QApplication(sys.argv)
    
    # Modern stil ayarı
    app.setStyle('Fusion')
    
    # Modern açık tema
    palette = QPalette()
    
    # Ana pencere renkleri
    palette.setColor(QPalette.ColorRole.Window, QColor(248, 249, 250))  # Açık gri arka plan
    palette.setColor(QPalette.ColorRole.WindowText, QColor(33, 37, 41))  # Koyu metin
    
    # Input alanları
    palette.setColor(QPalette.ColorRole.Base, QColor(255, 255, 255))  # Beyaz input arka planı
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(233, 236, 239))  # Alternatif satır rengi
    
    # Tooltip
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(33, 37, 41))
    
    # Metin renkleri
    palette.setColor(QPalette.ColorRole.Text, QColor(33, 37, 41))  # Ana metin
    
    # Buton renkleri
    palette.setColor(QPalette.ColorRole.Button, QColor(233, 236, 239))  # Açık gri buton
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(33, 37, 41))  # Koyu buton metni
    
    # Vurgular
    palette.setColor(QPalette.ColorRole.BrightText, QColor(220, 53, 69))  # Kırmızı vurgu
    palette.setColor(QPalette.ColorRole.Link, QColor(13, 110, 253))  # Mavi link
    palette.setColor(QPalette.ColorRole.Highlight, QColor(13, 110, 253))  # Mavi seçim
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))  # Beyaz seçili metin
    
    app.setPalette(palette)
    
    # Modern stil sheet eklemeleri
    app.setStyleSheet("""
        QGroupBox {
            font-weight: bold;
            border: 2px solid #dee2e6;
            border-radius: 8px;
            margin-top: 0.5ex;
            margin-bottom: 0.5ex;
            margin-left: 1px;
            margin-right: 1px;
            padding-top: 8px;
            padding-bottom: 4px;
            padding-left: 4px;
            padding-right: 4px;
            background-color: #ffffff;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 8px 0 8px;
            color: #495057;
            background-color: #ffffff;
        }
        
        QPushButton {
            background-color: #0d6efd;
            border: none;
            color: white;
            padding: 8px 16px;
            border-radius: 6px;
            font-weight: 500;
            min-height: 20px;
        }
        
        QPushButton:hover {
            background-color: #0b5ed7;
        }
        
        QPushButton:pressed {
            background-color: #0a58ca;
        }
        
        QComboBox {
            border: 1px solid #ced4da;
            border-radius: 4px;
            padding: 4px 8px;
            background-color: white;
            min-height: 20px;
        }
        
        QComboBox:hover {
            border-color: #86b7fe;
        }
        
        QComboBox:focus {
            border-color: #0d6efd;
            outline: none;
        }
        
        QTableWidget {
            border: 1px solid #dee2e6;
            border-radius: 4px;
            background-color: white;
            gridline-color: #dee2e6;
        }
        
        QTableWidget::item {
            border: 1px solid #dee2e6;
            padding: 4px;
        }
        
        QLabel {
            color: #495057;
        }
        
        QTextEdit {
            border: 1px solid #ced4da;
            border-radius: 4px;
            background-color: #f8f9fa;
            color: #495057;
            font-family: 'Consolas', 'Monaco', monospace;
            font-size: 12px;
        }
    """)
    
    # Ana pencereyi oluştur ve göster
    window = GroundStationGUI()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
