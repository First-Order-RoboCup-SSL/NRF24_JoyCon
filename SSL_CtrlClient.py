import tkinter as tk
from tkinter import ttk
import numpy as np
import time
import threading
import serial
import serial.tools.list_ports


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Remote Control Client GUI")

        # Serial communication
        self.serial_port = None
        self.serial_ports = self.get_serial_ports()

        # Control state
        self.active_keys = set()
        self.switches = {
            "kicker_bottom": tk.BooleanVar(value=False),
            "kicker_top": tk.BooleanVar(value=False),
            "dribbler": tk.BooleanVar(value=False),
        }

        # Velocity values (m/s and rad/s)
        self.velocities = {"forward": 0.0, "left": 0.0, "angular": 0.0}

        # Robot ID (0-31)
        self.robot_id = tk.IntVar(value=0)

        self.VELOCITY_SCALE = 0.4  # m/s for linear, rad/s for angular

        self.setup_gui()
        self.setup_key_bindings()

        # Start packet sending thread
        self.running = True
        self.packet_thread = threading.Thread(target=self.send_packets)
        self.packet_thread.daemon = True
        self.packet_thread.start()

        # Start serial receive thread
        self.serial_receive_thread = threading.Thread(target=self.receive_serial_data)
        self.serial_receive_thread.daemon = True
        self.serial_receive_thread.start()

        # Start velocity update loop
        self.update_velocities()

    def get_serial_ports(self):
        """Retrieve available serial ports."""
        return [port.device for port in serial.tools.list_ports.comports()]

    def connect_serial(self, port):
        """Establish serial connection."""
        try:
            # Close existing connection if open
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            # Open new connection
            self.serial_port = serial.Serial(
                port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,  # 8 data bits
                parity=serial.PARITY_EVEN,  # Even parity (makes it 9 bits total)
                stopbits=serial.STOPBITS_TWO,  # 2 stop bits
                timeout=0.1,
            )
            self.serial_status_label.config(
                text=f"Connected to {port}", foreground="green"
            )
            return True
        except Exception as e:
            self.serial_status_label.config(
                text=f"Connection failed: {str(e)}", foreground="red"
            )
            return False

    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Serial Connection Frame
        serial_frame = ttk.LabelFrame(main_frame, text="Serial Connection", padding="5")
        serial_frame.grid(row=0, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))

        # Port Selection Dropdown
        ttk.Label(serial_frame, text="Select Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar()
        port_dropdown = ttk.Combobox(
            serial_frame, textvariable=self.port_var, values=self.serial_ports, width=20
        )
        port_dropdown.grid(row=0, column=1, padx=5)

        # Connect Button
        connect_btn = ttk.Button(
            serial_frame, text="Connect", command=self.connect_to_selected_port
        )
        connect_btn.grid(row=0, column=2, padx=5)

        # Serial Status Label
        self.serial_status_label = ttk.Label(
            serial_frame, text="No port selected", foreground="red"
        )
        self.serial_status_label.grid(row=1, column=0, columnspan=3, padx=5)

        # MCU Response Display
        response_frame = ttk.LabelFrame(main_frame, text="MCU Response", padding="5")
        response_frame.grid(row=1, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))

        self.response_text = tk.Text(
            response_frame, height=5, width=50, state="disabled"
        )
        self.response_text.grid(row=0, column=0, padx=5, pady=5)

        # Velocity display
        vel_frame = ttk.LabelFrame(main_frame, text="Velocities", padding="5")
        vel_frame.grid(row=2, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))

        self.vel_labels = {}
        for i, (name, _) in enumerate(self.velocities.items()):
            ttk.Label(vel_frame, text=f"{name.capitalize()}:").grid(
                row=0, column=i * 2, padx=5
            )
            self.vel_labels[name] = ttk.Label(vel_frame, text="0.00")
            self.vel_labels[name].grid(row=0, column=i * 2 + 1, padx=5)

        # Special Switches Frame
        switches_frame = ttk.LabelFrame(
            main_frame, text="Special Controls", padding="5"
        )
        switches_frame.grid(row=3, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))

        # Kicker and Dribbler Switches
        switch_labels = [
            ("Kicker Bottom", "kicker_bottom"),
            ("Kicker Top", "kicker_top"),
            ("Dribbler", "dribbler"),
        ]
        for i, (label, var_name) in enumerate(switch_labels):
            ttk.Checkbutton(
                switches_frame, text=label, variable=self.switches[var_name]
            ).grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)

        # Robot ID Selector
        ttk.Label(switches_frame, text="Robot ID:").grid(
            row=3, column=0, padx=5, pady=2, sticky=tk.W
        )
        robot_id_spinbox = ttk.Spinbox(
            switches_frame, from_=0, to=31, textvariable=self.robot_id, width=5
        )
        robot_id_spinbox.grid(row=3, column=1, padx=5, pady=2, sticky=tk.W)

        # Instructions
        instructions = """
        Use WASD keys to control robot movement:
        W/S - Forward/Backward
        A/D - Left/Right
        Q/E - Rotate Counter/Clockwise
        """
        ttk.Label(main_frame, text=instructions, justify=tk.LEFT).grid(
            row=4, column=0, columnspan=2, pady=10, sticky=tk.W
        )

    def connect_to_selected_port(self):
        """Connect to the selected serial port."""
        selected_port = self.port_var.get()
        if selected_port:
            self.connect_serial(selected_port)

    def setup_key_bindings(self):
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

    def on_key_press(self, event):
        key = event.keysym.lower()
        if key in ["w", "a", "s", "d", "q", "e"]:
            self.active_keys.add(key)

    def on_key_release(self, event):
        key = event.keysym.lower()
        if key in ["w", "a", "s", "d", "q", "e"]:
            self.active_keys.discard(key)

    def update_velocities(self):
        # Update velocities based on active keys
        self.velocities["forward"] = 0.0
        self.velocities["left"] = 0.0
        self.velocities["angular"] = 0.0

        if "w" in self.active_keys:
            self.velocities["forward"] += self.VELOCITY_SCALE
        if "s" in self.active_keys:
            self.velocities["forward"] -= self.VELOCITY_SCALE
        if "a" in self.active_keys:
            self.velocities["left"] += self.VELOCITY_SCALE
        if "d" in self.active_keys:
            self.velocities["left"] -= self.VELOCITY_SCALE
        if "q" in self.active_keys:
            self.velocities["angular"] += self.VELOCITY_SCALE * 5
        if "e" in self.active_keys:
            self.velocities["angular"] -= self.VELOCITY_SCALE * 5

        # Update display labels
        for name, value in self.velocities.items():
            self.vel_labels[name].config(text=f"{value:.2f}")

        # Schedule next update
        self.root.after(20, self.update_velocities)

    def compute_crc(self, data: bytearray) -> int:
        """
        Calculate CRC-8, use 0x07 polynomial.
        这里的计算对 data 中的每个字节进行处理。
        """
        poly = 0x07
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def create_packet(self):
        """
        Create data packet to send to robot.

        Bytes:
          0:    Frame head (0xAA)
          1:    Robot ID
          2-3:  Local Forward Velocity (float16)
          4-5:  Local Left Velocity (float16)
          6-7:  Angular Velocity (float16)
          8-9:  Dribbler (state 2 bits + 14bits speed)
          10:   Upper kicker[7:4] + Lower Kicker[3:0]
          11:   Frame end (0x55)
        """
        # Convert velocities to float16
        forward_bytes = np.float16(self.velocities["forward"]).view(np.uint16)
        left_bytes = np.float16(self.velocities["left"]).view(np.uint16)
        angular_bytes = np.float16(self.velocities["angular"]).view(np.uint16)

        # # Create packet
        # packet = bytearray(
        #     [
        #         0xAA,  # Frame head
        #         self.robot_id.get() & 0xFF,  # Robot ID
        #         (forward_bytes >> 8) & 0xFF,  # Forward velocity high byte
        #         forward_bytes & 0xFF,  # Forward velocity low byte
        #         (left_bytes >> 8) & 0xFF,  # Left velocity high byte
        #         left_bytes & 0xFF,  # Left velocity low byte
        #         (angular_bytes >> 8) & 0xFF,  # Angular velocity high byte
        #         angular_bytes & 0xFF,  # Angular velocity low byte
        #     ]
        # )

        # TODO: MSB and LSB are being swapped in the joycon NRF transmitter, possibly because of endianness.
        packet = bytearray(
            [
                0xAA,  # Frame head
                self.robot_id.get() & 0xFF,  # Robot ID
                forward_bytes & 0xFF,  # Forward velocity low byte
                (forward_bytes >> 8) & 0xFF,  # Forward velocity high byte
                left_bytes & 0xFF,  # Left velocity low byte
                (left_bytes >> 8) & 0xFF,  # Left velocity high byte
                angular_bytes & 0xFF,  # Angular velocity low byte
                (angular_bytes >> 8) & 0xFF,  # Angular velocity high byte
            ]
        )

        # Create dribbler control bytes
        dribbler_speed = 0
        if self.switches["dribbler"].get():
            dribbler_speed = 0xC000  # Set bits 15:14 to 11
            dribbler_speed |= 4095 & 0x3FFF  # Set speed to max (all 14 bits)
        packet.extend(
            [
                (dribbler_speed >> 8) & 0xFF,  # Dribbler high byte
                dribbler_speed & 0xFF,  # Dribbler low byte
            ]
        )

        # Create kicker byte
        kicker_byte = 0
        if self.switches["kicker_top"].get():
            kicker_byte |= 0xF0  # Upper kicker full power
            self.switches["kicker_top"].set(False)  # Reset after use
        if self.switches["kicker_bottom"].get():
            kicker_byte |= 0x0F  # Lower kicker full power
            self.switches["kicker_bottom"].set(False)  # Reset after use

        packet.extend([kicker_byte, 0x55])  # Kicker controls  # Frame end

        return packet

    def send_packets(self):
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.02)
                continue

            try:
                packet = list(self.create_packet())
                self.serial_port.write(packet)
                binary_representation = [f"{byte:08b}" for byte in packet]
                hex_representation = [f"{byte:02X}" for byte in packet]
                print(f"Sent Packet: {hex_representation}")
                # print(f"Sent Packet: {list(packet)}")
            except Exception as e:
                print(f"Serial send error: {e}")

            time.sleep(0.05)  # 20ms delay

    def receive_serial_data(self):
        while self.running:
            if not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.1)
                continue

            try:
                if self.serial_port.in_waiting > 0:
                    data = str(bin(self.serial_port.read()[0]))
                    print(data)
                    if data:
                        self.root.after(0, self.update_response_display, data)
            except Exception as e:
                print(f"Serial receive error: {e}")
                time.sleep(0.1)

    def update_response_display(self, data):
        """线程安全地更新响应显示窗口。"""
        self.response_text.config(state="normal")
        self.response_text.insert(tk.END, data + "\n")
        self.response_text.see(tk.END)
        self.response_text.config(state="disabled")

    def cleanup(self):
        """退出前关闭线程和串口连接。"""
        self.running = False

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        # 等待线程结束
        self.packet_thread.join(timeout=1)
        self.serial_receive_thread.join(timeout=1)


def main():
    root = tk.Tk()
    app = RobotControlGUI(root)

    def on_closing():
        app.cleanup()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
