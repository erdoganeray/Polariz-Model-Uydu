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
from datetime import datetime
from typing import Dict, List, Tuple

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QComboBox, QTextEdit, QFrame,
    QSizePolicy, QGroupBox, QTableWidget, QTableWidgetItem
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QFont, QPixmap, QPalette, QColor
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
import pyqtgraph as pg
import numpy as np


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
        self.hata_kodu = "00000"
        self.basinc1 = 101325.0
        self.basinc2 = 101300.0
        self.yukseklik1 = 150.5
        self.yukseklik2 = 149.8
        self.irtifa_farki = 0.7
        self.inis_hizi = -2.3
        self.sicaklik = 25.4
        self.pil_gerilimi = 3.7
        self.pitch = 15.2
        self.roll = 8.7
        self.yaw = 180.5
        self.iot_s1_data = 22.5
        self.iot_s2_data = 23.1
        
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
        self.init_ui()
        self.setup_timers()
        
    def init_ui(self):
        """UI kurulumu"""
        self.setWindowTitle("YIS Ground Station - Model Uydu Kontrol Merkezi")
        self.setGeometry(100, 100, 1400, 900)
        
        # Ana widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Ana layout (yatay)
        main_layout = QHBoxLayout(central_widget)
        
        # Sol panel (grafikler + log)
        left_panel = self.create_left_panel()
        
        # Sağ panel (kamera, harita, veri, 3D model, kontroller)
        right_panel = self.create_right_panel()
        
        # Panelleri ana layout'a ekle
        main_layout.addWidget(left_panel, 3)  # Sol panel için oran
        main_layout.addWidget(right_panel, 2)  # Sağ panel için oran
        
    def create_left_panel(self):
        """Sol panel oluşturma (Grafikler + Log)"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
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
        
        # Üst: Kamera ve Harita
        top_section = self.create_camera_map_section()
        
        # Orta: Data ve 3D Model
        middle_section = self.create_data_3d_section()
        
        # Alt: COM ve Logo
        bottom_section = self.create_com_logo_section()
        
        right_layout.addWidget(top_section, 2)
        right_layout.addWidget(middle_section, 3)
        right_layout.addWidget(bottom_section, 1)
        
        return right_widget
        
    def create_camera_map_section(self):
        """Kamera ve Harita bölümü"""
        section_widget = QWidget()
        section_layout = QHBoxLayout(section_widget)
        
        # Kamera placeholder
        camera_group = QGroupBox("Kamera")
        camera_layout = QVBoxLayout(camera_group)
        camera_placeholder = QLabel("Kamera Görüntüsü\n(Placeholder)")
        camera_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        camera_placeholder.setStyleSheet("""
            QLabel {
                background-color: #e9ecef;
                color: #6c757d;
                border: 2px dashed #ced4da;
                border-radius: 8px;
                font-size: 14px;
                font-weight: 500;
            }
        """)
        camera_placeholder.setMinimumSize(200, 150)
        camera_layout.addWidget(camera_placeholder)
        
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
            if i % 2 == 0:  # Rakam
                combo.addItems([str(x) for x in range(21)])  # 0-20 arası sayılar
            else:  # Harf
                combo.addItems(['M', 'F', 'N', 'R', 'G', 'B', 'P', 'Y', 'C'])
            self.rhrh_combos.append(combo)
            combo_layout.addWidget(combo)
        
        # Telekomut gönder butonu
        self.telecommand_btn = QPushButton("RHRH: Telekomut Gönder")
        self.telecommand_btn.clicked.connect(self.send_telecommand)
        
        telecommand_layout.addLayout(combo_layout)
        telecommand_layout.addWidget(self.telecommand_btn)
        
        # Hata kodu tablosu
        error_group = QGroupBox("Hata Kodu")
        error_layout = QVBoxLayout(error_group)
        
        self.error_table = QTableWidget(2, 6)
        self.error_table.setMaximumHeight(80)
        
        # Hücre boyutlarını minimuma ayarla
        self.error_table.horizontalHeader().setVisible(False)
        self.error_table.verticalHeader().setVisible(False)
        self.error_table.setColumnWidth(0, 30)
        self.error_table.setColumnWidth(1, 30)
        self.error_table.setColumnWidth(2, 30)
        self.error_table.setColumnWidth(3, 30)
        self.error_table.setColumnWidth(4, 30)
        self.error_table.setColumnWidth(5, 30)
        self.error_table.setRowHeight(0, 25)
        self.error_table.setRowHeight(1, 25)
        
        # Üst satır (başlıklar)
        for i in range(6):
            item = QTableWidgetItem(str(i+1))
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.error_table.setItem(0, i, item)
            
        # Alt satır (durum)
        for i in range(6):
            item = QTableWidgetItem("")
            if i < 5:  # İlk 5 hata kodu biti
                item.setBackground(QColor("green"))
            else:
                item.setBackground(QColor("red"))
            self.error_table.setItem(1, i, item)
            
        error_layout.addWidget(self.error_table)
        
        # Durum metinleri
        self.status_label1 = QLabel("Pil Gerilimi: 3.7V (74%)")
        self.status_label1.setStyleSheet("font-weight: 500; color: #495057; font-size: 13px;")
        self.status_label2 = QLabel("Statü: 0 (Uçuşa Hazır)")
        self.status_label2.setStyleSheet("font-weight: 500; color: #198754; font-size: 13px;")
        
        data_layout.addWidget(telecommand_group)
        data_layout.addWidget(error_group)
        data_layout.addWidget(self.status_label1)
        data_layout.addWidget(self.status_label2)
        
        return data_group
        
    def create_com_logo_section(self):
        """COM ve Logo bölümü"""
        section_widget = QWidget()
        section_layout = QHBoxLayout(section_widget)
        
        # COM Kontrol paneli
        com_group = QGroupBox("Sistem Kontrolü")
        com_layout = QVBoxLayout(com_group)
        
        # Manuel ayrılma butonu
        self.manual_separation_btn = QPushButton("Manuel Ayrılma")
        self.manual_separation_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                padding: 10px 20px;
                border-radius: 6px;
                border: none;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
            QPushButton:pressed {
                background-color: #bd2130;
            }
        """)
        self.manual_separation_btn.clicked.connect(self.manual_separation)
        
        # COM port seçimi
        com_label = QLabel("COM Port Seçimi:")
        self.com_combo = QComboBox()
        self.com_combo.addItems(["COM1", "COM3", "COM5", "COM7"])  # Örnek COM portları
        
        com_layout.addWidget(self.manual_separation_btn)
        com_layout.addWidget(com_label)
        com_layout.addWidget(self.com_combo)
        
        # Logo bölümü
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
        
        section_layout.addWidget(com_group)
        section_layout.addWidget(logo_group)
        
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
        """Verileri güncelleme"""
        self.telemetry_data.simulate_update()
        
        # Grafikleri güncelle
        current_time = time.time() - self.start_time  # Başlangıçtan itibaren geçen saniye
        
        # Basınç grafiği
        self.update_graph('pressure', current_time, self.telemetry_data.basinc1)
        
        # Yükseklik grafiği
        self.update_graph('altitude', current_time, self.telemetry_data.yukseklik1)
        
        # İniş hızı grafiği
        self.update_graph('descent_speed', current_time, self.telemetry_data.inis_hizi)
        
        # Sıcaklık grafiği
        self.update_graph('temperature', current_time, self.telemetry_data.sicaklik)
        
        # İrtifa farkı grafiği
        self.update_graph('altitude_diff', current_time, self.telemetry_data.irtifa_farki)
        
        # IoT sıcaklık grafiği (ortalama)
        avg_iot_temp = (self.telemetry_data.iot_s1_data + self.telemetry_data.iot_s2_data) / 2
        self.update_graph('iot_temp', current_time, avg_iot_temp)
        
        # Durum metinlerini güncelle
        battery_percent = int((self.telemetry_data.pil_gerilimi - 3.0) / 1.0 * 100)
        self.status_label1.setText(f"Pil Gerilimi: {self.telemetry_data.pil_gerilimi:.1f}V ({battery_percent}%)")
        self.status_label2.setText(f"Statü: {self.telemetry_data.uydu_statusu} (Uçuşa Hazır)")
        
        # Hata kodunu güncelle
        self.update_error_display()
        
        # Log mesajı ekle
        self.add_log_message(f"Paket #{self.telemetry_data.paket_sayisi} alındı - Alt: {self.telemetry_data.yukseklik1:.1f}m")
        
    def update_graph(self, graph_id, x_value, y_value):
        """Grafik güncelleme"""
        graph = self.graphs[graph_id]
        
        graph['data_x'].append(x_value)
        graph['data_y'].append(y_value)
        
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
        """Hata kodu tablosunu güncelleme"""
        error_code = self.telemetry_data.hata_kodu
        for i in range(min(5, len(error_code))):
            item = self.error_table.item(1, i)
            if error_code[i] == '0':
                item.setBackground(QColor("green"))
            else:
                item.setBackground(QColor("red"))
                
    def send_telecommand(self):
        """Telekomut gönderme"""
        rhrh_code = ''.join([combo.currentText() for combo in self.rhrh_combos])
        self.add_log_message(f"Telekomut gönderildi: RHRH={rhrh_code}")
        
    def manual_separation(self):
        """Manuel ayrılma komutu"""
        self.add_log_message("UYARI: Manuel ayrılma komutu gönderildi!")
        
    def add_log_message(self, message):
        """Log mesajı ekleme"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_text.append(formatted_message)
        
        # Log'u otomatik kaydır
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


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
            margin-top: 1ex;
            padding-top: 10px;
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
