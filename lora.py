import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque


class LoraMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa Monitor - ESP32 Lora Haberleşme Sistemi")
        self.root.geometry("1600x900")  # Genişlik artırıldı istatistik paneli için
        
        # Serial connection variables
        self.serial_connection = None
        self.is_listening = False
        self.listen_thread = None
        
        # Data storage for graphs (30 seconds of data)
        self.max_data_points = 30
        self.time_data = deque(maxlen=self.max_data_points)
        
        # Graph data
        self.basinc1_data = deque(maxlen=self.max_data_points)
        self.basinc2_data = deque(maxlen=self.max_data_points)
        self.yukseklik1_data = deque(maxlen=self.max_data_points)
        self.yukseklik2_data = deque(maxlen=self.max_data_points)
        self.inis_hizi_data = deque(maxlen=self.max_data_points)
        self.sicaklik_data = deque(maxlen=self.max_data_points)
        self.irtifa_farki_data = deque(maxlen=self.max_data_points)
        self.iot_s1_data = deque(maxlen=self.max_data_points)
        self.iot_s2_data = deque(maxlen=self.max_data_points)
        
        # Statistics data for packet tracking
        self.total_received_packets = 0  # LoRa 2 tarafından alınan toplam paket sayısı
        self.last_packet_number = 0     # Son alınan paketteki paket sayısı verisi (beklenen toplam paket sayısı)
        self.packet_loss_count = 0      # Kayıp paket sayısı
        self.success_rate = 0.0         # Başarı oranı (yüzde)
        self.packet_receive_times = []  # Paket alma zamanları
        self.avg_receive_interval = 0.0 # Ortalama paket alma aralığı (saniye)
        
        self.setup_ui()
        self.refresh_com_ports()
        self.setup_graphs()
        
        # Start time for relative timing
        self.start_time = time.time()
        
    def setup_ui(self):
        # Main horizontal frame
        main_frame = ttk.Frame(self.root, padding="5")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Top control panel
        control_panel = ttk.Frame(main_frame)
        control_panel.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # COM Port Selection Frame
        com_frame = ttk.LabelFrame(control_panel, text="COM Port Seçimi", padding="5")
        com_frame.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(com_frame, text="COM Port:").grid(row=0, column=0, padx=(0, 5))
        
        self.com_var = tk.StringVar()
        self.com_combo = ttk.Combobox(com_frame, textvariable=self.com_var, state="readonly", width=15)
        self.com_combo.grid(row=0, column=1, padx=(0, 5))
        
        self.refresh_btn = ttk.Button(com_frame, text="Yenile", command=self.refresh_com_ports)
        self.refresh_btn.grid(row=0, column=2, padx=(0, 10))
        
        ttk.Label(com_frame, text="Baud Rate:").grid(row=0, column=3, padx=(0, 5))
        
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(com_frame, textvariable=self.baud_var, state="readonly", width=10)
        baud_combo['values'] = ('9600', '115200', '230400')
        baud_combo.grid(row=0, column=4)
        
        # Control Buttons Frame
        control_frame = ttk.LabelFrame(control_panel, text="Kontrol", padding="5")
        control_frame.pack(side=tk.LEFT, padx=(0, 10))
        
        self.start_btn = ttk.Button(control_frame, text="Başlat", command=self.start_listening)
        self.start_btn.grid(row=0, column=0, padx=(0, 5))
        
        self.stop_btn = ttk.Button(control_frame, text="Durdur", command=self.stop_listening, state=tk.DISABLED)
        self.stop_btn.grid(row=0, column=1, padx=(0, 5))
        
        self.clear_btn = ttk.Button(control_frame, text="Temizle", command=self.clear_log)
        self.clear_btn.grid(row=0, column=2, padx=(0, 5))
        
        self.send_btn = ttk.Button(control_frame, text="SEND", command=self.send_send_command, state=tk.DISABLED)
        self.send_btn.grid(row=0, column=3, padx=(0, 5))
        
        # Status Frame
        status_frame = ttk.LabelFrame(control_panel, text="Durum", padding="5")
        status_frame.pack(side=tk.LEFT, padx=(0, 10))
        
        self.status_var = tk.StringVar(value="Durum: Bağlantı bekleniyor")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_var)
        self.status_label.pack()
        
        # Statistics Frame
        stats_frame = ttk.LabelFrame(control_panel, text="İstatistikler", padding="5")
        stats_frame.pack(side=tk.LEFT)
        
        # Statistics labels
        self.received_packets_var = tk.StringVar(value="Alınan Paket: 0")
        self.last_packet_var = tk.StringVar(value="Son Paket No: 0")
        self.lost_packets_var = tk.StringVar(value="Kayıp Paket: 0")
        self.success_rate_var = tk.StringVar(value="Başarı: %0.0")
        self.avg_interval_var = tk.StringVar(value="Ort. Süre: 0.0s")
        
        ttk.Label(stats_frame, textvariable=self.received_packets_var).grid(row=0, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.last_packet_var).grid(row=1, column=0, sticky=tk.W)
        ttk.Label(stats_frame, textvariable=self.lost_packets_var).grid(row=0, column=1, sticky=tk.W, padx=(10, 0))
        ttk.Label(stats_frame, textvariable=self.success_rate_var).grid(row=1, column=1, sticky=tk.W, padx=(10, 0))
        ttk.Label(stats_frame, textvariable=self.avg_interval_var).grid(row=0, column=2, sticky=tk.W, padx=(10, 0))
        
        # Left Panel - Graphs (50% width)
        left_panel = ttk.LabelFrame(main_frame, text="Telemetri Grafikleri", padding="5")
        left_panel.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        left_panel.columnconfigure(0, weight=1)
        left_panel.rowconfigure(0, weight=1)
        
        # Graphs frame
        self.graphs_frame = ttk.Frame(left_panel)
        self.graphs_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.graphs_frame.columnconfigure(0, weight=1)
        self.graphs_frame.columnconfigure(1, weight=1)
        self.graphs_frame.rowconfigure(0, weight=1)
        self.graphs_frame.rowconfigure(1, weight=1)
        self.graphs_frame.rowconfigure(2, weight=1)
        
        # Right Panel - Log (50% width)
        right_panel = ttk.LabelFrame(main_frame, text="LoRa Veri Log Ekranı", padding="5")
        right_panel.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        
        # Log Text Area with Scrollbar
        self.log_text = scrolledtext.ScrolledText(
            right_panel, 
            wrap=tk.WORD, 
            font=("Consolas", 9),
            state=tk.DISABLED
        )
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
    def setup_graphs(self):
        """Setup the 6 telemetry graphs"""
        # Create matplotlib figure with 3x2 subplots
        self.fig, self.axes = plt.subplots(3, 2, figsize=(12, 8))
        self.fig.tight_layout(pad=3.0)
        
        # Configure each subplot
        # Graph 1: Basınç (Pascal-Saniye) - basinc1 (yük) ve basinc2 (taşıyıcı)
        self.axes[0, 0].set_title('Basınç (Pascal)', fontsize=10)
        self.axes[0, 0].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[0, 0].set_ylabel('Pascal', fontsize=8)
        self.line_basinc1, = self.axes[0, 0].plot([], [], 'b-', label='Yük', linewidth=2)
        self.line_basinc2, = self.axes[0, 0].plot([], [], 'r-', label='Taşıyıcı', linewidth=2)
        self.axes[0, 0].legend(fontsize=8)
        self.axes[0, 0].grid(True, alpha=0.3)
        
        # Graph 2: Yükseklik (Metre-Saniye) - yukseklik1 (yük) ve yukseklik2 (taşıyıcı)
        self.axes[0, 1].set_title('Yükseklik (Metre)', fontsize=10)
        self.axes[0, 1].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[0, 1].set_ylabel('Metre', fontsize=8)
        self.line_yukseklik1, = self.axes[0, 1].plot([], [], 'b-', label='Yük', linewidth=2)
        self.line_yukseklik2, = self.axes[0, 1].plot([], [], 'r-', label='Taşıyıcı', linewidth=2)
        self.axes[0, 1].legend(fontsize=8)
        self.axes[0, 1].grid(True, alpha=0.3)
        
        # Graph 3: İniş Hızı (m/s-Saniye) - inis_hizi
        self.axes[1, 0].set_title('İniş Hızı (m/s)', fontsize=10)
        self.axes[1, 0].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[1, 0].set_ylabel('m/s', fontsize=8)
        self.line_inis_hizi, = self.axes[1, 0].plot([], [], 'g-', linewidth=2)
        self.axes[1, 0].grid(True, alpha=0.3)
        
        # Graph 4: Sıcaklık (Celsius-Saniye) - sicaklik
        self.axes[1, 1].set_title('Atmosfer Sıcaklığı (°C)', fontsize=10)
        self.axes[1, 1].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[1, 1].set_ylabel('°C', fontsize=8)
        self.line_sicaklik, = self.axes[1, 1].plot([], [], 'orange', linewidth=2)
        self.axes[1, 1].grid(True, alpha=0.3)
        
        # Graph 5: İrtifa Farkı (Metre-Saniye) - irtifa_farki
        self.axes[2, 0].set_title('İrtifa Farkı (Metre)', fontsize=10)
        self.axes[2, 0].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[2, 0].set_ylabel('Metre', fontsize=8)
        self.line_irtifa_farki, = self.axes[2, 0].plot([], [], 'purple', linewidth=2)
        self.axes[2, 0].grid(True, alpha=0.3)
        
        # Graph 6: IoT Sıcaklık (Celsius-Saniye) - iot_s1_data ve iot_s2_data
        self.axes[2, 1].set_title('IoT Sensör Sıcaklığı (°C)', fontsize=10)
        self.axes[2, 1].set_xlabel('Zaman (saniye)', fontsize=8)
        self.axes[2, 1].set_ylabel('°C', fontsize=8)
        self.line_iot_s1, = self.axes[2, 1].plot([], [], 'cyan', label='IoT S1', linewidth=2)
        self.line_iot_s2, = self.axes[2, 1].plot([], [], 'magenta', label='IoT S2', linewidth=2)
        self.axes[2, 1].legend(fontsize=8)
        self.axes[2, 1].grid(True, alpha=0.3)
        
        # Embed the matplotlib figure in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graphs_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
    def parse_telemetry_data(self, data_string):
        """Parse telemetry data and update graphs"""
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
                    basinc1 = float(parts[4])
                    basinc2 = float(parts[5])
                    yukseklik1 = float(parts[6])
                    yukseklik2 = float(parts[7])
                    irtifa_farki = float(parts[8])
                    inis_hizi = float(parts[9])
                    sicaklik = float(parts[10])
                    iot_s1_data = float(parts[19])
                    iot_s2_data = float(parts[20])
                    
                    # Add timestamp (relative to start time)
                    current_time = time.time() - self.start_time
                    
                    # Update data deques
                    self.time_data.append(current_time)
                    self.basinc1_data.append(basinc1)
                    self.basinc2_data.append(basinc2)
                    self.yukseklik1_data.append(yukseklik1)
                    self.yukseklik2_data.append(yukseklik2)
                    self.inis_hizi_data.append(inis_hizi)
                    self.sicaklik_data.append(sicaklik)
                    self.irtifa_farki_data.append(irtifa_farki)
                    self.iot_s1_data.append(iot_s1_data)
                    self.iot_s2_data.append(iot_s2_data)
                    
                    # Update packet statistics
                    self.total_received_packets += 1
                    self.last_packet_number = paket_sayisi
                    self.packet_loss_count = max(0, self.last_packet_number - self.total_received_packets)
                    
                    # Track packet receive times for average interval calculation
                    current_timestamp = time.time()
                    self.packet_receive_times.append(current_timestamp)
                    
                    # Calculate average receive interval (keep only last 10 packets for moving average)
                    if len(self.packet_receive_times) > 10:
                        self.packet_receive_times.pop(0)
                    
                    if len(self.packet_receive_times) > 1:
                        time_diffs = []
                        for i in range(1, len(self.packet_receive_times)):
                            time_diffs.append(self.packet_receive_times[i] - self.packet_receive_times[i-1])
                        self.avg_receive_interval = sum(time_diffs) / len(time_diffs)
                    
                    # Calculate success rate
                    if self.last_packet_number > 0:
                        self.success_rate = (self.total_received_packets / self.last_packet_number) * 100
                    else:
                        self.success_rate = 0.0
                    
                    # Update statistics display in main thread
                    self.root.after(0, self.update_statistics_display)
                    
                    # Update graphs in the main thread
                    self.root.after(0, self.update_graphs)
                    
                    return True
                except ValueError as ve:
                    self.log_message(f"Telemetri parse hatası: {str(ve)}", "ERROR")
                    return False
            else:
                self.log_message(f"Yetersiz telemetri verisi: {len(parts)} bölüm, beklenen: 22", "ERROR")
                return False
        except Exception as e:
            self.log_message(f"Telemetri parsing hatası: {str(e)}", "ERROR")
        return False
        
    def update_graphs(self):
        """Update all graphs with current data"""
        if len(self.time_data) == 0:
            return
            
        time_array = list(self.time_data)
        
        try:
            # Update Graph 1: Basınç
            if len(self.basinc1_data) > 0 and len(self.basinc2_data) > 0:
                self.line_basinc1.set_data(time_array, list(self.basinc1_data))
                self.line_basinc2.set_data(time_array, list(self.basinc2_data))
                self.axes[0, 0].relim()
                self.axes[0, 0].autoscale_view()
                
            # Update Graph 2: Yükseklik
            if len(self.yukseklik1_data) > 0 and len(self.yukseklik2_data) > 0:
                self.line_yukseklik1.set_data(time_array, list(self.yukseklik1_data))
                self.line_yukseklik2.set_data(time_array, list(self.yukseklik2_data))
                self.axes[0, 1].relim()
                self.axes[0, 1].autoscale_view()
                
            # Update Graph 3: İniş Hızı
            if len(self.inis_hizi_data) > 0:
                self.line_inis_hizi.set_data(time_array, list(self.inis_hizi_data))
                self.axes[1, 0].relim()
                self.axes[1, 0].autoscale_view()
                
            # Update Graph 4: Sıcaklık
            if len(self.sicaklik_data) > 0:
                self.line_sicaklik.set_data(time_array, list(self.sicaklik_data))
                self.axes[1, 1].relim()
                self.axes[1, 1].autoscale_view()
                
            # Update Graph 5: İrtifa Farkı
            if len(self.irtifa_farki_data) > 0:
                self.line_irtifa_farki.set_data(time_array, list(self.irtifa_farki_data))
                self.axes[2, 0].relim()
                self.axes[2, 0].autoscale_view()
                
            # Update Graph 6: IoT Sıcaklık
            if len(self.iot_s1_data) > 0 and len(self.iot_s2_data) > 0:
                self.line_iot_s1.set_data(time_array, list(self.iot_s1_data))
                self.line_iot_s2.set_data(time_array, list(self.iot_s2_data))
                self.axes[2, 1].relim()
                self.axes[2, 1].autoscale_view()
                
            # Redraw the canvas
            self.canvas.draw()
            
        except Exception as e:
            self.log_message(f"Grafik güncelleme hatası: {str(e)}", "ERROR")
            
    def update_statistics_display(self):
        """Update statistics display labels"""
        try:
            self.received_packets_var.set(f"Alınan Paket: {self.total_received_packets}")
            self.last_packet_var.set(f"Son Paket No: {self.last_packet_number}")
            self.lost_packets_var.set(f"Kayıp Paket: {self.packet_loss_count}")
            self.success_rate_var.set(f"Başarı: %{self.success_rate:.1f}")
            self.avg_interval_var.set(f"Ort. Süre: {self.avg_receive_interval:.1f}s")
        except Exception as e:
            self.log_message(f"İstatistik güncelleme hatası: {str(e)}", "ERROR")
        
    def refresh_com_ports(self):
        """Available COM ports listesini yenile"""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        
        self.com_combo['values'] = port_list
        if port_list:
            self.com_combo.current(0)
        else:
            self.com_var.set("")
            
    def log_message(self, message, msg_type="INFO"):
        """Log ekranına mesaj ekle"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Thread-safe GUI update
        def update_log():
            self.log_text.config(state=tk.NORMAL)
            
            if msg_type == "ERROR":
                color_tag = "error"
            elif msg_type == "SUCCESS":
                color_tag = "success"
            elif msg_type == "DATA":
                color_tag = "data"
            else:
                color_tag = "info"
                
            # Configure tags if not already done
            self.log_text.tag_configure("error", foreground="red")
            self.log_text.tag_configure("success", foreground="green")
            self.log_text.tag_configure("data", foreground="blue")
            self.log_text.tag_configure("info", foreground="black")
            
            log_entry = f"[{timestamp}] {message}\n"
            self.log_text.insert(tk.END, log_entry, color_tag)
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
            
        # Schedule the update in the main thread
        self.root.after(0, update_log)
        
    def start_listening(self):
        """COM port dinlemeyi başlat"""
        if not self.com_var.get():
            messagebox.showerror("Hata", "Lütfen bir COM port seçin!")
            return
            
        com_port = self.com_var.get().split(" - ")[0]
        baud_rate = int(self.baud_var.get())
        
        try:
            self.serial_connection = serial.Serial(
                port=com_port,
                baudrate=baud_rate,
                timeout=1
            )
            
            self.is_listening = True
            self.listen_thread = threading.Thread(target=self.listen_to_serial, daemon=True)
            self.listen_thread.start()
            
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.com_combo.config(state=tk.DISABLED)
            self.send_btn.config(state=tk.NORMAL)
            
            self.status_var.set(f"Durum: {com_port} dinleniyor...")
            self.log_message(f"COM port bağlantısı başarılı: {com_port} @ {baud_rate} baud", "SUCCESS")
            
        except serial.SerialException as e:
            messagebox.showerror("Bağlantı Hatası", f"COM port açılamadı: {str(e)}")
            self.log_message(f"COM port bağlantı hatası: {str(e)}", "ERROR")
            
    def stop_listening(self):
        """COM port dinlemeyi durdur"""
        self.is_listening = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.com_combo.config(state="readonly")
        self.send_btn.config(state=tk.DISABLED)
        
        self.status_var.set("Durum: Bağlantı durduruldu")
        self.log_message("COM port bağlantısı kapatıldı", "INFO")
        
    def listen_to_serial(self):
        """Serial port'u dinle ve gelen verileri logla"""
        self.log_message("LoRa ESP32 dinleme başladı...", "INFO")
        
        while self.is_listening and self.serial_connection and self.serial_connection.is_open:
            try:
                if self.serial_connection.in_waiting > 0:
                    # Read line from serial
                    data = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    
                    if data:
                        # Parse and format the message
                        if data.startswith("GELEN VERİ:"):
                            # GELEN VERİ formatını özel olarak işle
                            self.log_message(data, "DATA")
                        elif data.startswith("TEL,"):
                            # Telemetry data with TEL prefix - parse and update graphs
                            if self.parse_telemetry_data(data):
                                self.log_message(data, "DATA")
                            else:
                                self.log_message(f"Telemetri Parse Hatası: {data}", "ERROR")
                        elif any(keyword in data for keyword in ["BTN,", "REQ,", "PRS,", "L4DATA,", "L5DATA,"]):
                            self.log_message(f"LoRa Verisi: {data}", "DATA")
                        elif "Received:" in data or "Sending:" in data:
                            self.log_message(f"ESP32 Log: {data}", "INFO")
                        elif "binary" in data.lower() and ("gonderildi" in data.lower() or "alindi" in data.lower()):
                            self.log_message(f"LoRa İletişim: {data}", "SUCCESS")
                        elif "SEND" in data and "yazarak" in data:
                            # Komut yardım mesajlarını gizle
                            pass
                        else:
                            # Check if this looks like telemetry data (starts with a number)
                            if data and data[0].isdigit() and ',' in data:
                                # This might be telemetry data without TEL, prefix
                                if self.parse_telemetry_data(data):
                                    self.log_message(data, "DATA")
                                else:
                                    self.log_message(f"Raw Telemetri Parse Hatası: {data}", "ERROR")
                            else:
                                # Diğer serial mesajları
                                if len(data.strip()) > 0:
                                    self.log_message(f"Serial: {data}", "INFO")
                            
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
            except serial.SerialException as e:
                self.log_message(f"Serial okuma hatası: {str(e)}", "ERROR")
                break
            except Exception as e:
                self.log_message(f"Beklenmeyen hata: {str(e)}", "ERROR")
                break
                
        self.log_message("Serial dinleme durduruldu", "INFO")
        
    def clear_log(self):
        """Log ekranını temizle ve grafik verilerini sıfırla"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        
        # Clear graph data
        self.time_data.clear()
        self.basinc1_data.clear()
        self.basinc2_data.clear()
        self.yukseklik1_data.clear()
        self.yukseklik2_data.clear()
        self.inis_hizi_data.clear()
        self.sicaklik_data.clear()
        self.irtifa_farki_data.clear()
        self.iot_s1_data.clear()
        self.iot_s2_data.clear()
        
        # Clear statistics data
        self.total_received_packets = 0
        self.last_packet_number = 0
        self.packet_loss_count = 0
        self.success_rate = 0.0
        self.packet_receive_times = []
        self.avg_receive_interval = 0.0
        
        # Reset start time
        self.start_time = time.time()
        
        # Clear graphs and update statistics display
        self.update_graphs()
        self.update_statistics_display()
        
        self.log_message("Log, grafik ve istatistik verileri temizlendi", "INFO")
        
    def send_send_command(self):
        """SEND komutunu hızlıca gönder"""
        if not self.is_listening or not self.serial_connection:
            messagebox.showerror("Hata", "Önce COM port bağlantısını başlatın!")
            return
            
        try:
            self.serial_connection.write(b'SEND\n')
            self.log_message("SEND komutu gönderildi", "SUCCESS")
            
        except serial.SerialException as e:
            self.log_message(f"SEND komutu gönderme hatası: {str(e)}", "ERROR")
        
    def on_closing(self):
        """Uygulama kapatılırken çağrılan fonksiyon"""
        if self.is_listening:
            self.stop_listening()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = LoraMonitor(root)
    
    # Handle window close event
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Start the GUI
    root.mainloop()


if __name__ == "__main__":
    main()