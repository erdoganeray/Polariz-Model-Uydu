import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime


class LoraMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa Monitor - ESP32 Lora_2 Dinleyici")
        self.root.geometry("800x600")
        
        # Serial connection variables
        self.serial_connection = None
        self.is_listening = False
        self.listen_thread = None
        
        self.setup_ui()
        self.refresh_com_ports()
        
    def setup_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        
        # COM Port Selection Frame
        com_frame = ttk.LabelFrame(main_frame, text="COM Port Seçimi", padding="5")
        com_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
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
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        
        self.start_btn = ttk.Button(control_frame, text="Başlat", command=self.start_listening)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.stop_btn = ttk.Button(control_frame, text="Durdur", command=self.stop_listening, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        self.clear_btn = ttk.Button(control_frame, text="Temizle", command=self.clear_log)
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 5))
        
        # Status Label
        self.status_var = tk.StringVar(value="Durum: Bağlantı bekleniyor")
        self.status_label = ttk.Label(control_frame, textvariable=self.status_var)
        self.status_label.pack(side=tk.LEFT, padx=(10, 0))
        
        # Command Input Frame
        command_frame = ttk.LabelFrame(main_frame, text="Komut Gönder", padding="5")
        command_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.send_btn = ttk.Button(command_frame, text="SEND", command=self.send_send_command, state=tk.DISABLED)
        self.send_btn.pack(side=tk.LEFT)
        
        ttk.Label(command_frame, text="ESP32'ye SEND komutu gönderir (5x button control)").pack(side=tk.LEFT, padx=(10, 0))
        
        # Log Frame
        log_frame = ttk.LabelFrame(main_frame, text="LoRa Veri Log Ekranı", padding="5")
        log_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Log Text Area with Scrollbar
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            wrap=tk.WORD, 
            font=("Consolas", 10),
            state=tk.DISABLED
        )
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
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
        self.log_message("LoRa_2 ESP32 dinleme başladı...", "INFO")
        
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
                        elif any(keyword in data for keyword in ["TEL,", "BTN,", "REQ,", "PRS,", "L4DATA,", "L5DATA,"]):
                            self.log_message(f"LoRa Verisi: {data}", "DATA")
                        elif "Received:" in data or "Sending:" in data:
                            self.log_message(f"ESP32 Log: {data}", "INFO")
                        elif "binary" in data.lower() and ("gonderildi" in data.lower() or "alindi" in data.lower()):
                            self.log_message(f"LoRa İletişim: {data}", "SUCCESS")
                        elif "SEND" in data and "yazarak" in data:
                            # Komut yardım mesajlarını gizle
                            pass
                        else:
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
        """Log ekranını temizle"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log_message("Log ekranı temizlendi", "INFO")
        
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