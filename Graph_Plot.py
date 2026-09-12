import asyncio
import threading
import tkinter as tk
from tkinter import ttk, messagebox

from bleak import BleakScanner, BleakClient

from collections import deque

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# BLE UUIDs - Nordic UART Service
# ============================================================

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"

TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"


# ============================================================
# GLOBAL VARIABLES
# ============================================================

client = None

devices = []

ble_loop = asyncio.new_event_loop()


# ============================================================
# GRAPH DATA
# ============================================================

MAX_POINTS = 200

sample_numbers = deque(maxlen=MAX_POINTS)

x_data = deque(maxlen=MAX_POINTS)

y_data = deque(maxlen=MAX_POINTS)

sample_number = 0


# ============================================================
# BLE THREAD
# ============================================================

def ble_thread():

    asyncio.set_event_loop(ble_loop)

    ble_loop.run_forever()


threading.Thread(
    target=ble_thread,
    daemon=True
).start()


# ============================================================
# RUN ASYNC FUNCTION
# ============================================================

def run_async(coro):

    return asyncio.run_coroutine_threadsafe(
        coro,
        ble_loop
    )


# ============================================================
# BLE RECEIVE CALLBACK
# ============================================================

def notification_handler(sender, data):

    global sample_number

    # ========================================================
    # EXPECTED FRAME = 10 BYTES
    #
    # [0]     0xBC
    # [1-2]   X
    # [3-4]   Y
    # [5-8]   Sample Count
    # [9]     0xBD
    # ========================================================

    if len(data) != 10:
        return


    # ========================================================
    # CHECK START BYTE
    # ========================================================

    if data[0] != 0xBC:
        return


    # ========================================================
    # CHECK END BYTE
    # ========================================================

    if data[9] != 0xBD:
        return


    # ========================================================
    # DECODE X RAW
    # ========================================================

    x_raw = (
        (data[1] << 8) |
        data[2]
    )


    # Convert uint16 -> int16

    if x_raw & 0x8000:

        x_raw -= 0x10000


    # ========================================================
    # DECODE Y RAW
    # ========================================================

    y_raw = (
        (data[3] << 8) |
        data[4]
    )


    # Convert uint16 -> int16

    if y_raw & 0x8000:

        y_raw -= 0x10000


    # ========================================================
    # DIVIDE RAW DATA BY 100
    # ========================================================

    x_value = x_raw / 100.0

    y_value = y_raw / 100.0


    # ========================================================
    # UPDATE GRAPH DATA
    # ========================================================

    sample_number += 1

    sample_numbers.append(sample_number)

    x_data.append(x_value)

    y_data.append(y_value)


    # ========================================================
    # UPDATE GRAPH IN TKINTER THREAD
    # ========================================================

    root.after(
        0,
        update_graph
    )


# ============================================================
# UPDATE GRAPH
# ============================================================

def update_graph():

    if len(sample_numbers) == 0:
        return


    # Update lines

    line_x.set_data(
        list(sample_numbers),
        list(x_data)
    )

    line_y.set_data(
        list(sample_numbers),
        list(y_data)
    )


    # ========================================================
    # X AXIS
    # ========================================================

    if len(sample_numbers) < MAX_POINTS:

        graph_ax.set_xlim(
            0,
            MAX_POINTS
        )

    else:

        graph_ax.set_xlim(
            sample_numbers[0],
            sample_numbers[-1]
        )


    # ========================================================
    # Y AXIS AUTO SCALE
    # ========================================================

    all_values = (
        list(x_data) +
        list(y_data)
    )


    if all_values:

        minimum = min(all_values)

        maximum = max(all_values)


        if minimum == maximum:

            minimum -= 1

            maximum += 1


        margin = (
            maximum - minimum
        ) * 0.10


        if margin < 0.5:

            margin = 0.5


        graph_ax.set_ylim(
            minimum - margin,
            maximum + margin
        )


    # ========================================================
    # REDRAW
    # ========================================================

    graph_canvas.draw_idle()


# ============================================================
# SCAN BLE
# ============================================================

async def scan_ble():

    global devices

    status_label.config(
        text="Scanning..."
    )


    device_list.delete(
        0,
        tk.END
    )


    devices = await BleakScanner.discover(
        timeout=5
    )


    for device in devices:

        name = device.name

        if name is None:

            name = "Unknown"


        device_list.insert(
            tk.END,
            f"{name} | {device.address}"
        )


    status_label.config(
        text=f"Found {len(devices)} devices"
    )


def scan_button():

    run_async(
        scan_ble()
    )


# ============================================================
# CONNECT BLE
# ============================================================

async def connect_ble():

    global client

    selection = device_list.curselection()


    if not selection:

        root.after(
            0,
            lambda: messagebox.showwarning(
                "BLE",
                "Select a BLE device first."
            )
        )

        return


    index = selection[0]

    device = devices[index]


    status_label.config(
        text=f"Connecting to {device.name}..."
    )


    try:

        client = BleakClient(
            device
        )


        await client.connect()


        if client.is_connected:

            # =================================================
            # ENABLE NOTIFICATIONS
            # =================================================

            await client.start_notify(
                TX_UUID,
                notification_handler
            )


            status_label.config(
                text=f"Connected: {device.name}"
            )


            root.after(
                0,
                lambda: connect_btn.config(
                    state=tk.DISABLED
                )
            )


            root.after(
                0,
                lambda: disconnect_btn.config(
                    state=tk.NORMAL
                )
            )


    except Exception as e:

        status_label.config(
            text="Connection failed"
        )


        root.after(
            0,
            lambda: messagebox.showerror(
                "BLE Connection Error",
                str(e)
            )
        )


def connect_button():

    run_async(
        connect_ble()
    )


# ============================================================
# DISCONNECT BLE
# ============================================================

async def disconnect_ble():

    global client


    try:

        if client:

            if client.is_connected:

                try:

                    await client.stop_notify(
                        TX_UUID
                    )

                except:
                    pass


                await client.disconnect()


    except Exception as e:

        print(
            "Disconnect error:",
            e
        )


    client = None


    status_label.config(
        text="Disconnected"
    )


    root.after(
        0,
        lambda: connect_btn.config(
            state=tk.NORMAL
        )
    )


    root.after(
        0,
        lambda: disconnect_btn.config(
            state=tk.DISABLED
        )
    )


def disconnect_button():

    run_async(
        disconnect_ble()
    )


# ============================================================
# SEND HEX COMMAND
# ============================================================

async def send_command():

    if client is None:

        root.after(
            0,
            lambda: messagebox.showwarning(
                "BLE",
                "Not connected."
            )
        )

        return


    if not client.is_connected:

        root.after(
            0,
            lambda: messagebox.showwarning(
                "BLE",
                "Not connected."
            )
        )

        return


    command_string = command_entry.get().strip()


    if not command_string:

        return


    # ========================================================
    # HEX STRING -> BYTES
    # ========================================================

    try:

        command = bytes.fromhex(
            command_string
        )


    except ValueError:

        root.after(
            0,
            lambda: messagebox.showerror(
                "Command Error",
                "Invalid HEX format.\n\n"
                "Example:\n"
                "AA 01 02 03 FF"
            )
        )

        return


    # ========================================================
    # SEND
    # ========================================================

    try:

        await client.write_gatt_char(
            RX_UUID,
            command,
            response=False
        )


        print(
            f"Command Sent: {command.hex(' ')}"
        )


    except Exception as e:

        root.after(
            0,
            lambda: messagebox.showerror(
                "Send Error",
                str(e)
            )
        )


def send_button():

    run_async(
        send_command()
    )


# ============================================================
# CLEAR COMMAND
# ============================================================

def clear_command():

    command_entry.delete(
        0,
        tk.END
    )


# ============================================================
# CLEAR GRAPH
# ============================================================

def clear_graph():

    global sample_number

    sample_number = 0

    sample_numbers.clear()

    x_data.clear()

    y_data.clear()

    line_x.set_data(
        [],
        []
    )

    line_y.set_data(
        [],
        []
    )

    graph_ax.set_xlim(
        0,
        MAX_POINTS
    )

    graph_ax.set_ylim(
        -10,
        10
    )

    graph_canvas.draw_idle()


# ============================================================
# TKINTER WINDOW
# ============================================================

root = tk.Tk()

root.title(
    "ESP32 BLE Command Tool"
)

root.geometry(
    "1000x850"
)


# ============================================================
# TOP FRAME
# ============================================================

top_frame = ttk.Frame(
    root
)

top_frame.pack(
    fill="x",
    padx=10,
    pady=10
)


# ============================================================
# SCAN
# ============================================================

scan_btn = ttk.Button(
    top_frame,
    text="SCAN",
    command=scan_button
)

scan_btn.pack(
    side="left",
    padx=5
)


# ============================================================
# CONNECT
# ============================================================

connect_btn = ttk.Button(
    top_frame,
    text="CONNECT",
    command=connect_button
)

connect_btn.pack(
    side="left",
    padx=5
)


# ============================================================
# DISCONNECT
# ============================================================

disconnect_btn = ttk.Button(
    top_frame,
    text="DISCONNECT",
    command=disconnect_button,
    state=tk.DISABLED
)

disconnect_btn.pack(
    side="left",
    padx=5
)


# ============================================================
# STATUS
# ============================================================

status_label = ttk.Label(
    top_frame,
    text="Disconnected"
)

status_label.pack(
    side="left",
    padx=20
)


# ============================================================
# DEVICE LIST
# ============================================================

ttk.Label(
    root,
    text="BLE Devices"
).pack(
    anchor="w",
    padx=10
)


device_list = tk.Listbox(
    root,
    height=7
)

device_list.pack(
    fill="x",
    padx=10,
    pady=5
)


# ============================================================
# COMMAND FRAME
# ============================================================

command_frame = ttk.Frame(
    root
)

command_frame.pack(
    fill="x",
    padx=10,
    pady=10
)


ttk.Label(
    command_frame,
    text="HEX Command:"
).pack(
    side="left"
)


command_entry = ttk.Entry(
    command_frame
)

command_entry.pack(
    side="left",
    fill="x",
    expand=True,
    padx=10
)


# Default command

command_entry.insert(
    0,
    "01 02 03 04 AA 55"
)


# ============================================================
# SEND
# ============================================================

send_btn = ttk.Button(
    command_frame,
    text="SEND",
    command=send_button
)

send_btn.pack(
    side="left",
    padx=5
)


# ============================================================
# CLEAR COMMAND
# ============================================================

clear_btn = ttk.Button(
    command_frame,
    text="CLEAR",
    command=clear_command
)

clear_btn.pack(
    side="left",
    padx=5
)


# ============================================================
# GRAPH
# ============================================================

graph_frame = ttk.Frame(
    root
)

graph_frame.pack(
    fill="both",
    expand=True,
    padx=10,
    pady=5
)


# ============================================================
# MATPLOTLIB FIGURE
# ============================================================

figure = Figure(
    figsize=(10, 4),
    dpi=100
)


graph_ax = figure.add_subplot(
    111
)


graph_ax.set_title(
    "Real-Time X / Y Data"
)

graph_ax.set_xlabel(
    "Sample"
)

graph_ax.set_ylabel(
    "Value"
)

graph_ax.grid(
    True
)


# ============================================================
# GRAPH LINES
# ============================================================

line_x, = graph_ax.plot(
    [],
    [],
    label="X"
)


line_y, = graph_ax.plot(
    [],
    [],
    label="Y"
)


graph_ax.legend()


graph_ax.set_xlim(
    0,
    MAX_POINTS
)


graph_ax.set_ylim(
    -10,
    10
)


# ============================================================
# EMBED GRAPH INTO TKINTER
# ============================================================

graph_canvas = FigureCanvasTkAgg(
    figure,
    master=graph_frame
)


graph_canvas.draw()


graph_canvas.get_tk_widget().pack(
    fill="both",
    expand=True
)


# ============================================================
# CLEAR GRAPH BUTTON
# ============================================================

clear_graph_btn = ttk.Button(
    root,
    text="CLEAR GRAPH",
    command=clear_graph
)

clear_graph_btn.pack(
    pady=5
)


# ============================================================
# CLOSE APPLICATION
# ============================================================

def close_application():

    if client:

        run_async(
            disconnect_ble()
        )


    ble_loop.call_soon_threadsafe(
        ble_loop.stop
    )


    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    close_application
)


# ============================================================
# START
# ============================================================

root.mainloop()