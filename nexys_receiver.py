"""
============================================================
MUON LIFETIME MEASUREMENT SYSTEM -- RECEIVER GUI
============================================================
Nexys A7-100T | 921600 baud | Receiver only

FPGA -> PYTHON (6 bytes per edge):
  Byte 0 : {R/F flag, TS[30:24]}  bit7=1 Rising, bit7=0 Falling
  Byte 1 : TS[23:16]
  Byte 2 : TS[15:8]
  Byte 3 : TS[7:0]
  Byte 4 : 0x0D (CR)
  Byte 5 : 0x0A (LF)

Timestamps:
  FPGA  : 31-bit counter at 100MHz, wraparound corrected on Python side
  Wall  : GPS (NEO-6M NMEA) if connected, else laptop system time
  IST   : UTC + 5:30
  Display format : HH:MM:SS.ffffff
  CSV format     : YYYYMMDDHHMMSSffffff
============================================================
"""

import tkinter as tk                            # toolkit to build the window and buttons
from tkinter import ttk, messagebox, filedialog # extra parts like popups and file dialogs
import tkinter.scrolledtext as scrolledtext     # scrollable text box for the event log
import serial                                   # lets Python talk to the FPGA over USB
import serial.tools.list_ports                  # finds all available COM ports on computer
import threading                                # lets multiple tasks run simultaneously
import struct                                   # helps unpack raw bytes into numbers
import time                                     # used for timestamps and small delays
import csv                                      # used to save data as spreadsheet files
import queue                                    # safe inbox for passing data between threads
from datetime import datetime, timezone, timedelta  # tools for working with dates and times
from collections import deque                   # a list that automatically drops old items
import matplotlib                               # the main plotting library
matplotlib.use('TkAgg')                         # tells matplotlib to draw inside tkinter window
from matplotlib.figure import Figure            # creates a blank figure to draw charts on
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # embeds charts into tkinter
import numpy as np                              # math library for averages and arrays

# ============================================================
# CONSTANTS
# ============================================================
BAUD_RATE       = 921600        # speed of data transfer from FPGA to PC
BYTES_PER_FRAME = 6             # each FPGA message is exactly 6 bytes long
NS_PER_CYCLE    = 10            # each FPGA clock tick is 10 nanoseconds
MAX_COUNTER     = 0x7FFFFFFF    # biggest value the FPGA counter can reach
TIMESTAMP_MASK  = 0x7FFFFFFF    # mask to extract just the time bits
RF_FLAG_MASK    = 0x80000000    # mask to check if edge is rising or falling
IST_OFFSET      = timedelta(hours=5, minutes=30)  # India is UTC plus 5.5 hours

# Colours used throughout the GUI
BG_DARK  = '#0d1117'    # very dark background colour
BG_PANEL = '#161b22'    # slightly lighter panel background
BG_CARD  = '#1c2128'    # card background colour
BORDER   = '#30363d'    # border line colour
CYAN     = '#39d0d8'    # cyan used for titles and highlights
GREEN    = '#3fb950'    # green used for connected and valid states
RED      = '#f85149'    # red used for errors and disconnected states
YELLOW   = '#e3b341'    # yellow used for GPS and warnings
PURPLE   = '#bc8cff'    # purple used for time source and limits
TEXT     = '#e6edf3'    # standard white text colour
DIM      = '#7d8590'    # dimmed grey for secondary labels


# ============================================================
# TIME HELPERS
# ============================================================
def to_ist(utc_dt):
    """Convert UTC datetime to IST datetime."""
    return utc_dt + IST_OFFSET              # adds 5 hours 30 minutes to UTC time

def fmt_display(dt):
    """HH:MM:SS.ffffff for display."""
    return dt.strftime("%H:%M:%S.%f")       # formats time as hours minutes seconds microseconds

def fmt_csv(dt):
    """YYYYMMDDHHMMSSffffff for CSV."""
    return dt.strftime("%Y%m%d%H%M%S%f")   # formats as one long number for CSV files

def now_utc():
    return datetime.now(timezone.utc)       # returns the current UTC time right now


# ============================================================
# MAIN APPLICATION
# ============================================================
class ReceiverGUI:                          # blueprint for the entire receiver application
    def __init__(self):                     # runs once when the app starts up
        self.root = tk.Tk()                 # creates the main application window
        self.root.title("Muon Receiver -- Nexys A7-100T")  # sets the window title bar text
        self.root.geometry("1500x900")      # sets starting window size to 1500 by 900 pixels
        self.root.configure(bg=BG_DARK)     # sets very dark background colour
        self.root.resizable(True, True)     # allows window to be resized in any direction

        # Serial (FPGA)
        self.serial_port  = None            # no FPGA USB connection exists yet
        self.is_connected = False           # FPGA is not connected at startup
        self.rx_thread    = None            # no background listening thread yet
        self.stop_rx      = False           # flag to tell RX thread to stop

        # GPS serial
        self.gps_port      = None           # no GPS USB connection exists yet
        self.gps_connected = False          # GPS is not connected at startup
        self.gps_thread    = None           # no GPS listening thread yet
        self.stop_gps      = False          # flag to tell GPS thread to stop
        self.gps_utc_time  = None           # stores the latest time received from GPS
        self.gps_pc_time   = None           # stores the PC clock when GPS message arrived
        self.gps_lock      = threading.Lock()  # prevents two threads reading GPS at once

        # FPGA timestamp wraparound
        self.last_raw      = None           # stores the previous FPGA counter value
        self.wraparound    = 0              # counts how many times counter has reset to zero

        # Experiment anchor -- set on first edge received
        self.anchor_utc    = None           # real clock time at the very first pulse
        self.anchor_fpga_s = None           # FPGA time in seconds at the very first pulse

        # Edge state machine
        self.t_rise1 = None                 # stores timestamp of first pulse rising edge
        self.t_fall1 = None                 # stores timestamp of first pulse falling edge
        self.t_rise2 = None                 # stores timestamp of second pulse rising edge

        # Data
        self.pulse_pairs  = []              # list storing all completed pulse pairs
        self.delta_t_list = []              # list of all measured time gaps between pulses
        self.waveform_edges = deque(maxlen=40)  # stores last 40 edges for waveform display

        # Stats
        self.total_edges = 0                # counts every edge received so far
        self.valid_pairs = 0                # counts only complete valid pulse pairs

        self._build_ui()                    # draws all panels buttons and labels on screen
        self._refresh_fpga_ports()          # fills FPGA port dropdown with available ports
        self._refresh_gps_ports()           # fills GPS port dropdown with available ports
        self._schedule_plot_update()        # starts automatically updating the histogram

    # ========================================================
    # FPGA SECONDS FROM CYCLES (wraparound corrected)
    # ========================================================
    def _fpga_seconds(self, raw_ts):
        """Return FPGA time in seconds with wraparound correction.
        Returns None if raw_ts looks like garbage (too far from last_raw).
        This protects the wraparound counter from corruption.
        """
        MAX_FWD_CYCLES = 50_000_000         # 500ms worth of cycles maximum jump allowed

        if self.last_raw is None:           # if this is the very first timestamp received
            self.last_raw = raw_ts          # saves it as the starting reference point
            true_cycles = self.wraparound * (MAX_COUNTER + 1) + raw_ts  # total cycles ever
            return true_cycles * NS_PER_CYCLE / 1e9  # converts cycles to seconds

        if raw_ts >= self.last_raw:         # if counter moved forward normally
            diff_fwd = raw_ts - self.last_raw   # calculates how many cycles moved forward
            if diff_fwd <= MAX_FWD_CYCLES:      # if jump is within allowed range
                # Normal forward -- accept
                self.last_raw = raw_ts          # updates reference to new value
                true_cycles = self.wraparound * (MAX_COUNTER + 1) + raw_ts  # total cycles
                return true_cycles * NS_PER_CYCLE / 1e9  # returns time in seconds
            else:
                # Too far forward -- garbage
                return None                     # rejects suspicious timestamp as garbage
        else:
            diff_back = self.last_raw - raw_ts  # counter went backwards, check why
            if diff_back > (MAX_COUNTER // 2):  # if backwards by more than half the range
                # Legitimate wraparound
                self.wraparound += 1            # increments the overflow counter by one
                self.last_raw = raw_ts          # saves new counter value as reference
                true_cycles = self.wraparound * (MAX_COUNTER + 1) + raw_ts  # total cycles
                return true_cycles * NS_PER_CYCLE / 1e9  # returns corrected time in seconds
            else:
                # Backwards without wraparound -- garbage
                return None                     # rejects as a corrupted packet


    def _wall_utc(self, fpga_s):
        """
        Returns UTC datetime for a given FPGA timestamp.
        If GPS available: anchor to GPS time.
        Else: anchor to laptop system time.
        Both anchors are set on the first edge received.
        """
        if self.anchor_utc is None:         # if no anchor has been set yet
            return None                     # cannot calculate wall time yet
        elapsed = fpga_s - self.anchor_fpga_s  # seconds elapsed since first pulse
        return self.anchor_utc + timedelta(seconds=elapsed)  # adds elapsed to anchor time

    def _set_anchor(self, fpga_s, wall_capture):
        """Set experiment anchor on first edge.
        wall_capture is the PC time.time() captured in the RX thread
        at the moment the byte arrived -- not GUI thread delay."""
        if self.anchor_utc is not None:     # if anchor already set, do nothing
            return
        self.anchor_fpga_s = fpga_s         # saves FPGA time at first pulse
        with self.gps_lock:                 # locks GPS data so it is not changed mid-read
            if self.gps_connected and self.gps_utc_time is not None:  # if GPS is working
                # GPS available: interpolate from last GPS fix
                elapsed_since_gps = wall_capture - self.gps_pc_time   # time since last GPS message
                self.anchor_utc = self.gps_utc_time + timedelta(
                    seconds=elapsed_since_gps)  # calculates accurate UTC from GPS
                self._log("Anchor set from GPS time")   # logs that GPS anchor was used
            else:
                # Use PC time captured in RX thread (accurate)
                self.anchor_utc = datetime.fromtimestamp(
                    wall_capture, tz=timezone.utc)  # uses laptop clock as anchor instead
                self._log("Anchor set from laptop system time")  # logs that laptop time used

    # ========================================================
    # UI BUILD
    # ========================================================
    def _build_ui(self):                    # constructs the entire visible interface
        title_bar = tk.Frame(self.root, bg='#0d1f2d', height=54)  # dark blue title strip
        title_bar.pack(fill=tk.X)           # stretches title bar across full window width
        title_bar.pack_propagate(False)     # prevents title bar from shrinking to fit contents
        tk.Label(title_bar,
                 text="  MUON LIFETIME MEASUREMENT -- RECEIVER",
                 bg='#0d1f2d', fg=CYAN,
                 font=('Courier New', 17, 'bold')).pack(
                     side=tk.LEFT, pady=12)  # app title text on left side
        tk.Label(title_bar,
                 text="Nexys A7-100T  |  100 MHz  |  921600 baud  ",
                 bg='#0d1f2d', fg=DIM,
                 font=('Courier New', 10)).pack(
                     side=tk.RIGHT, pady=16)  # board info text on right side

        body = tk.Frame(self.root, bg=BG_DARK)  # main content area below title
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)  # fills all remaining space

        left = tk.Frame(body, bg=BG_DARK, width=320)  # narrow left panel for controls
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))  # sticks to left side
        left.pack_propagate(False)          # keeps left panel at fixed 320px width

        right = tk.Frame(body, bg=BG_DARK)  # wide right panel for charts and data
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)  # fills all remaining space

        self._build_left(left)              # builds all left panel controls
        self._build_right(right)            # builds all right panel charts and tables

    # -- LEFT PANEL ------------------------------------------
    def _build_left(self, parent):          # builds every widget in the left control panel

        def card(label, color=CYAN):        # helper to create a labelled box panel
            f = tk.LabelFrame(parent, text=f"  {label}  ",
                              bg=BG_PANEL, fg=color,
                              font=('Courier New', 9, 'bold'),
                              relief=tk.FLAT, bd=1,
                              highlightbackground=BORDER,
                              highlightthickness=1)  # creates a framed box with a label
            f.pack(fill=tk.X, pady=(0, 6))  # stretches box across full left panel width
            return f                         # returns the box so items can be added to it

        # -- FPGA CONNECTION ----------------------------------
        cf = card("FPGA CONNECTION")        # creates the FPGA connection box
        row = tk.Frame(cf, bg=BG_PANEL)     # horizontal row inside the box
        row.pack(fill=tk.X, padx=8, pady=6) # stretches row across the box width
        tk.Label(row, text="Port:", bg=BG_PANEL, fg=DIM,
                 font=('Courier New', 9)).pack(side=tk.LEFT)  # Port label on left
        self.fpga_port_combo = ttk.Combobox(row, width=8,
                                            font=('Courier New', 9),
                                            state='readonly')  # dropdown for FPGA COM port
        self.fpga_port_combo.pack(side=tk.LEFT, padx=4)  # places dropdown next to label
        tk.Button(row, text="Refresh",
                  command=self._refresh_fpga_ports,
                  bg=BG_CARD, fg=CYAN,
                  font=('Courier New', 8, 'bold'),
                  relief=tk.FLAT, padx=4,
                  cursor='hand2').pack(side=tk.LEFT, padx=2)  # refresh ports button
        self.fpga_conn_btn = tk.Button(row, text="Connect",
                                       command=self._toggle_fpga,
                                       bg=GREEN, fg=BG_DARK,
                                       font=('Courier New', 9, 'bold'),
                                       relief=tk.FLAT, padx=8,
                                       cursor='hand2')  # green connect button
        self.fpga_conn_btn.pack(side=tk.LEFT, padx=(6, 0))  # places button on right of row
        self.fpga_status = tk.Label(cf, text="Disconnected",
                                    bg=BG_PANEL, fg=RED,
                                    font=('Courier New', 10, 'bold'))  # red disconnected label
        self.fpga_status.pack(pady=(0, 6))  # adds spacing below status label

        # -- GPS CONNECTION -----------------------------------
        gf = card("GPS  (NEO-6M)", YELLOW)  # creates the GPS connection box in yellow
        grow = tk.Frame(gf, bg=BG_PANEL)    # horizontal row for GPS controls
        grow.pack(fill=tk.X, padx=8, pady=6)  # stretches row across box width
        tk.Label(grow, text="Port:", bg=BG_PANEL, fg=DIM,
                 font=('Courier New', 9)).pack(side=tk.LEFT)  # Port label
        self.gps_port_combo = ttk.Combobox(grow, width=8,
                                           font=('Courier New', 9),
                                           state='readonly')  # dropdown for GPS COM port
        self.gps_port_combo.pack(side=tk.LEFT, padx=4)  # places dropdown next to label
        tk.Label(grow, text="Baud:", bg=BG_PANEL, fg=DIM,
                 font=('Courier New', 9)).pack(side=tk.LEFT)  # Baud label
        self.gps_baud_combo = ttk.Combobox(grow, width=7,
                                           font=('Courier New', 9),
                                           values=['9600','115200'],
                                           state='readonly')  # dropdown for GPS baud rate
        self.gps_baud_combo.set('9600')     # defaults to 9600 baud for NEO-6M
        self.gps_baud_combo.pack(side=tk.LEFT, padx=4)  # places baud dropdown in row
        self.gps_conn_btn = tk.Button(grow, text="Connect",
                                      command=self._toggle_gps,
                                      bg=YELLOW, fg=BG_DARK,
                                      font=('Courier New', 9, 'bold'),
                                      relief=tk.FLAT, padx=6,
                                      cursor='hand2')  # yellow GPS connect button
        self.gps_conn_btn.pack(side=tk.LEFT, padx=(4, 0))  # places button on right of row
        self.gps_status = tk.Label(gf,
                                   text="Not connected -- using laptop time",
                                   bg=BG_PANEL, fg=DIM,
                                   font=('Courier New', 9))  # grey status text below
        self.gps_status.pack(pady=(0, 6))   # adds spacing below GPS status label

        # -- TIME SOURCE INDICATOR ----------------------------
        tf = card("TIME SOURCE", PURPLE)    # creates the time source indicator box
        self.time_source_label = tk.Label(tf,
                                          text="Laptop System Time",
                                          bg=BG_PANEL, fg=PURPLE,
                                          font=('Courier New', 10, 'bold'))  # shows which clock is active
        self.time_source_label.pack(pady=6) # adds spacing around the label

        # -- LIVE STATISTICS ----------------------------------
        sf = card("LIVE STATISTICS", GREEN) # creates the live stats box in green
        self.stat_labels = {}               # empty dictionary to store stat label widgets

        def stat_row(key, label, color=TEXT):  # helper to add one row of statistics
            r = tk.Frame(sf, bg=BG_PANEL)   # horizontal row for one statistic
            r.pack(fill=tk.X, padx=10, pady=2)  # adds row with padding
            tk.Label(r, text=label, bg=BG_PANEL, fg=DIM,
                     font=('Courier New', 9), width=14,
                     anchor='w').pack(side=tk.LEFT)  # label name on left side
            lbl = tk.Label(r, text="--", bg=BG_PANEL, fg=color,
                           font=('Courier New', 11, 'bold'))  # value shows dashes until data arrives
            lbl.pack(side=tk.LEFT)          # places value next to label
            self.stat_labels[key] = lbl     # saves label widget so it can be updated later

        stat_row('edges',   'Total Edges',  TEXT)    # row showing total edge count
        stat_row('pairs',   'Valid Pairs',  GREEN)   # row showing valid pair count
        stat_row('last_dt', 'Last Dt R-R',  CYAN)    # row showing most recent time gap
        stat_row('mean_dt', 'Mean Dt R-R',  CYAN)    # row showing average time gap
        stat_row('min_dt',  'Min Dt',       PURPLE)  # row showing smallest gap seen
        stat_row('max_dt',  'Max Dt',       PURPLE)  # row showing largest gap seen
        tk.Frame(sf, bg=BG_PANEL, height=4).pack()   # small spacer at bottom of stats box

        # -- ACTIONS ------------------------------------------
        af = card("ACTIONS", RED)           # creates the actions box in red
        tk.Button(af, text="Export CSV",
                  command=self._export_csv,
                  bg=BG_CARD, fg=GREEN,
                  font=('Courier New', 10, 'bold'),
                  relief=tk.FLAT, padx=8, pady=5,
                  cursor='hand2').pack(fill=tk.X,
                                      padx=10, pady=3)  # button to save data as CSV file
        tk.Button(af, text="Reset All",
                  command=self._reset_all,
                  bg=BG_CARD, fg=RED,
                  font=('Courier New', 10, 'bold'),
                  relief=tk.FLAT, padx=8, pady=5,
                  cursor='hand2').pack(fill=tk.X,
                                      padx=10, pady=3)  # button to clear all recorded data
        tk.Frame(af, bg=BG_PANEL, height=4).pack()  # small spacer at bottom

        # -- EVENT LOG ----------------------------------------
        lf = card("EVENT LOG", DIM)         # creates the event log box in grey
        self.log_box = scrolledtext.ScrolledText(
            lf, height=12, bg='#090c10', fg=CYAN,
            font=('Courier New', 8), relief=tk.FLAT,
            state=tk.DISABLED, wrap=tk.WORD)  # scrollable text area showing system messages
        self.log_box.pack(fill=tk.BOTH, expand=True,
                          padx=6, pady=6)   # fills available space in the log box

    # -- RIGHT PANEL -----------------------------------------
    def _build_right(self, parent):         # builds the tabs on the right side
        nb = ttk.Notebook(parent)           # creates a tabbed panel widget
        nb.pack(fill=tk.BOTH, expand=True)  # fills all available space
        style = ttk.Style()                 # creates style manager for the tabs
        style.configure('TNotebook',
                        background=BG_DARK, borderwidth=0)  # dark background for tab bar
        style.configure('TNotebook.Tab',
                        background=BG_PANEL, foreground=DIM,
                        font=('Courier New', 9, 'bold'),
                        padding=[12, 6])    # styles each tab button appearance
        style.map('TNotebook.Tab',
                  background=[('selected', BG_CARD)],
                  foreground=[('selected', CYAN)])  # active tab is lighter with cyan text

        wf_tab   = tk.Frame(nb, bg=BG_DARK) # frame for the live waveform tab
        hist_tab = tk.Frame(nb, bg=BG_DARK) # frame for the histogram tab
        tbl_tab  = tk.Frame(nb, bg=BG_DARK) # frame for the data table tab
        nb.add(wf_tab,   text="  Live Waveform  ")   # adds waveform tab to notebook
        nb.add(hist_tab, text="  Dt Histogram  ")    # adds histogram tab to notebook
        nb.add(tbl_tab,  text="  Pulse Pair Data  ") # adds data table tab to notebook
        self._build_waveform_tab(wf_tab)    # fills the waveform tab with its chart
        self._build_histogram_tab(hist_tab) # fills the histogram tab with its chart
        self._build_table_tab(tbl_tab)      # fills the data table tab with its grid

    def _build_waveform_tab(self, parent):  # creates the live pulse waveform chart
        self.fig_wf = Figure(figsize=(10, 4.5), dpi=100,
                             facecolor=BG_DARK)  # blank figure with dark background
        self.ax_wf  = self.fig_wf.add_subplot(
            111, facecolor=BG_PANEL)        # one chart inside the figure
        self._style_ax(self.ax_wf)          # applies dark theme styling to chart
        self.ax_wf.set_ylim([-0.3, 1.6])    # sets vertical range for the waveform
        self.ax_wf.text(0.5, 0.5, "Waiting for pulses...",
                        ha='center', va='center',
                        color=DIM, fontsize=13,
                        transform=self.ax_wf.transAxes)  # placeholder text before data arrives
        self.canvas_wf = FigureCanvasTkAgg(
            self.fig_wf, master=parent)     # embeds the chart inside the tkinter tab
        self.canvas_wf.get_tk_widget().pack(
            fill=tk.BOTH, expand=True, padx=6, pady=6)  # makes chart fill the tab area

    def _build_histogram_tab(self, parent): # creates the delta-t histogram chart
        self.fig_hist = Figure(figsize=(10, 4.5), dpi=100,
                               facecolor=BG_DARK)  # blank figure with dark background
        self.ax_hist  = self.fig_hist.add_subplot(
            111, facecolor=BG_PANEL)        # one chart inside the figure
        self._style_ax(self.ax_hist)        # applies dark theme styling to chart
        self.ax_hist.text(0.5, 0.5, "Waiting for data...",
                          ha='center', va='center',
                          color=DIM, fontsize=13,
                          transform=self.ax_hist.transAxes)  # placeholder text before data arrives
        self.canvas_hist = FigureCanvasTkAgg(
            self.fig_hist, master=parent)   # embeds the chart inside the tkinter tab
        self.canvas_hist.get_tk_widget().pack(
            fill=tk.BOTH, expand=True, padx=6, pady=6)  # makes chart fill the tab area

    def _build_table_tab(self, parent):     # creates the scrollable data table
        # 17 columns
        cols = (
            'Pair#',
            'T_rise1 (IST)',    'T_rise1 (FPGA s)',
            'T_fall1 (IST)',    'T_fall1 (FPGA s)',
            'T_rise2 (IST)',    'T_rise2 (FPGA s)',
            'T_fall2 (IST)',    'T_fall2 (FPGA s)',
            'P1 Width (ns)',    'P2 Width (ns)',
            'Dt R-R (us)'
        )                                   # defines all column names for the table
        self.tree = ttk.Treeview(parent, columns=cols,
                                 show='headings', height=28)  # creates the table widget
        widths = {
            'Pair#': 55,
            'T_rise1 (IST)': 145, 'T_rise1 (FPGA s)': 130,
            'T_fall1 (IST)': 145, 'T_fall1 (FPGA s)': 130,
            'T_rise2 (IST)': 145, 'T_rise2 (FPGA s)': 130,
            'T_fall2 (IST)': 145, 'T_fall2 (FPGA s)': 130,
            'P1 Width (ns)': 100, 'P2 Width (ns)': 100,
            'Dt R-R (us)': 100
        }                                   # pixel widths for each column
        for col in cols:
            self.tree.heading(col, text=col)    # sets the column header text
            self.tree.column(col, width=widths[col],
                             anchor='center')   # sets column width and centres the text
        sb = ttk.Scrollbar(parent, orient=tk.VERTICAL,
                           command=self.tree.yview)  # vertical scrollbar for the table
        self.tree.configure(yscrollcommand=sb.set)   # links scrollbar to table
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH,
                       expand=True, padx=(6, 0), pady=6)  # table fills most of the tab
        sb.pack(side=tk.RIGHT, fill=tk.Y, pady=6)    # scrollbar on far right side
        self.tree.tag_configure('pair', foreground=GREEN)  # table rows shown in green

    # ========================================================
    # HELPERS
    # ========================================================
    def _style_ax(self, ax):               # applies dark theme styling to a chart
        ax.tick_params(colors=DIM, labelsize=9)  # tick marks and numbers in grey
        for spine in ['bottom', 'left']:
            ax.spines[spine].set_color(BORDER)   # left and bottom border lines in grey
        ax.spines['top'].set_visible(False)       # hides the top border line
        ax.spines['right'].set_visible(False)     # hides the right border line
        ax.grid(True, color=BORDER, linestyle='--',
                linewidth=0.5, alpha=0.6)          # adds faint dashed grid lines

    def _log(self, msg):                    # adds a timestamped message to event log
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # current time to milliseconds
        self.log_box.config(state=tk.NORMAL)    # temporarily unlocks log for writing
        self.log_box.insert(tk.END, f"[{ts}] {msg}\n")  # appends message with timestamp
        self.log_box.see(tk.END)                # scrolls log down to show latest message
        self.log_box.config(state=tk.DISABLED)  # locks log again to prevent typing

    def _refresh_fpga_ports(self):          # finds all COM ports for FPGA dropdown
        ports = [p.device for p in
                 serial.tools.list_ports.comports()]  # gets list of all COM port names
        self.fpga_port_combo['values'] = ports  # fills dropdown with found ports
        if ports:
            self.fpga_port_combo.set(ports[0])  # selects first available port

    def _refresh_gps_ports(self):           # finds all COM ports for GPS dropdown
        ports = [p.device for p in
                 serial.tools.list_ports.comports()]  # gets list of all COM port names
        self.gps_port_combo['values'] = ports   # fills GPS dropdown with found ports
        if len(ports) > 1:
            self.gps_port_combo.set(ports[1])   # selects second port if two available
        elif ports:
            self.gps_port_combo.set(ports[0])   # otherwise selects first port

    # ========================================================
    # FPGA CONNECTION
    # ========================================================
    def _toggle_fpga(self):                 # switches between connect and disconnect
        if not self.is_connected:           # if currently disconnected
            port = self.fpga_port_combo.get()  # reads selected port from dropdown
            if not port:
                messagebox.showerror("Error",
                                     "Select FPGA COM port")  # shows error if no port chosen
                return
            try:
                self.serial_port = serial.Serial(
                    port, BAUD_RATE, timeout=1)  # opens USB connection to FPGA
                time.sleep(0.2)             # waits 200ms for connection to stabilise
                self.is_connected = True    # marks as connected
                self.fpga_status.config(
                    text="Connected", fg=GREEN)  # status label turns green saying Connected
                self.fpga_conn_btn.config(
                    text="Disconnect", bg=RED)   # button turns red saying Disconnect
                self.stop_rx   = False          # clears the stop flag
                self.rx_thread = threading.Thread(
                    target=self._rx_loop, daemon=True)  # creates background listening thread
                self.rx_thread.start()          # starts the background thread
                self._log(f"FPGA connected: {port} @ {BAUD_RATE}")  # logs connection success
            except Exception as e:
                messagebox.showerror("Error", str(e))  # shows error popup if connection fails
        else:
            self._disconnect_fpga()         # if already connected, disconnect instead

    def _disconnect_fpga(self):             # closes the FPGA USB connection cleanly
        self.is_connected = False           # marks as disconnected immediately
        self.stop_rx      = True            # tells background thread to stop
        if self.serial_port:
            try:
                self.serial_port.close()    # closes the USB port
            except Exception:
                pass                        # ignores any errors while closing
            self.serial_port = None         # clears the port object
        self.fpga_status.config(text="Disconnected", fg=RED)  # status turns red
        self.fpga_conn_btn.config(text="Connect", bg=GREEN)   # button turns green
        self._log("FPGA disconnected")      # logs the disconnection

    # ========================================================
    # GPS CONNECTION
    # ========================================================
    def _toggle_gps(self):                  # switches GPS between connect and disconnect
        if not self.gps_connected:          # if GPS currently disconnected
            port = self.gps_port_combo.get()  # reads selected GPS port
            baud = int(self.gps_baud_combo.get())  # reads selected GPS baud rate
            if not port:
                messagebox.showerror("Error",
                                     "Select GPS COM port")  # shows error if no port chosen
                return
            try:
                self.gps_port = serial.Serial(
                    port, baud, timeout=2)  # opens USB connection to GPS module
                self.gps_connected = True   # marks GPS as connected
                self.stop_gps = False       # clears the GPS stop flag
                self.gps_conn_btn.config(
                    text="Disconnect", bg=RED)  # button turns red saying Disconnect
                self.gps_status.config(
                    text="Waiting for fix...", fg=YELLOW)  # shows waiting for satellite fix
                self.time_source_label.config(
                    text="GPS (NEO-6M) -- waiting for fix",
                    fg=YELLOW)              # updates time source indicator to yellow
                self.gps_thread = threading.Thread(
                    target=self._gps_loop, daemon=True)  # creates GPS background thread
                self.gps_thread.start()     # starts listening to GPS messages
                self._log(f"GPS connected: {port} @ {baud}")  # logs GPS connection
            except Exception as e:
                messagebox.showerror("GPS Error", str(e))  # shows error if connection fails
        else:
            self._disconnect_gps()          # if already connected, disconnect instead

    def _disconnect_gps(self):              # closes the GPS USB connection cleanly
        self.gps_connected = False          # marks GPS as disconnected
        self.stop_gps = True                # tells GPS thread to stop
        if self.gps_port:
            try:
                self.gps_port.close()       # closes the GPS USB port
            except Exception:
                pass                        # ignores errors while closing
            self.gps_port = None            # clears the GPS port object
        self.gps_conn_btn.config(text="Connect", bg=YELLOW)  # button turns yellow
        self.gps_status.config(
            text="Not connected -- using laptop time", fg=DIM)  # status goes grey
        self.time_source_label.config(
            text="Laptop System Time", fg=PURPLE)  # time source reverts to laptop
        self._log("GPS disconnected -- using laptop time")  # logs the switch

    # ========================================================
    # GPS NMEA LOOP
    # ========================================================
    def _gps_loop(self):                    # background thread that reads GPS sentences
        buf = ""                            # empty text buffer to collect GPS data
        while not self.stop_gps and self.gps_connected:  # keeps running while GPS connected
            try:
                if self.gps_port.in_waiting:    # if new GPS bytes have arrived
                    raw = self.gps_port.read(
                        self.gps_port.in_waiting)   # reads all available bytes
                    buf += raw.decode('ascii', errors='ignore')  # adds to text buffer
                    while '\n' in buf:          # while there is a complete GPS sentence
                        line, buf = buf.split('\n', 1)  # splits off one complete line
                        line = line.strip()     # removes whitespace from ends
                        utc = self._parse_nmea(line)  # tries to extract time from sentence
                        if utc is not None:     # if a valid time was found
                            with self.gps_lock:
                                self.gps_utc_time = utc         # saves the GPS UTC time
                                self.gps_pc_time  = time.time() # saves when we got it
                            self.root.after(
                                0, self._on_gps_fix, utc)  # updates GPS status display
                time.sleep(0.01)            # waits 10ms before checking again
            except Exception as e:
                if self.gps_connected:
                    self.root.after(
                        0, self._log, f"GPS error: {e}")  # logs GPS error message
                break                       # stops the GPS thread on error

    def _on_gps_fix(self, utc):             # called when a valid GPS time is received
        ist = to_ist(utc)                   # converts UTC to Indian Standard Time
        self.gps_status.config(
            text=f"Fix OK  {fmt_display(ist)} IST",
            fg=GREEN)                       # shows current IST time in green
        self.time_source_label.config(
            text="GPS (NEO-6M) -- synced",
            fg=GREEN)                       # updates time source label to green synced

    def _parse_nmea(self, sentence):
        """Parse $GPRMC or $GNRMC, return UTC datetime or None."""
        try:
            if not sentence.startswith('$'):
                return None                 # ignores lines that are not NMEA sentences
            # Validate checksum
            if '*' in sentence:             # if sentence has a checksum at the end
                body, cs = sentence[1:].split('*')  # splits sentence from checksum
                calc = 0
                for c in body:
                    calc ^= ord(c)          # calculates expected checksum by XOR
                if calc != int(cs.strip(), 16):
                    return None             # discards sentence if checksum is wrong
            else:
                body = sentence[1:]         # uses sentence without checksum validation

            fields = body.split(',')        # splits sentence into comma-separated fields
            tag = fields[0]                 # first field is the sentence type name

            if 'RMC' in tag:               # if this is an RMC position sentence
                time_s  = fields[1]         # time field from the sentence
                status  = fields[2]         # A means valid fix, V means no fix
                date_s  = fields[9]         # date field from the sentence
                if status != 'A' or not time_s or not date_s:
                    return None             # discards if no valid GPS fix
                h  = int(time_s[0:2])       # extracts hours from time string
                m  = int(time_s[2:4])       # extracts minutes from time string
                s  = float(time_s[4:])      # extracts seconds from time string
                d  = int(date_s[0:2])       # extracts day from date string
                mo = int(date_s[2:4])       # extracts month from date string
                y  = 2000 + int(date_s[4:6])  # extracts year adding 2000
                return datetime(y, mo, d, h, m,
                                int(s),
                                int((s % 1) * 1e6),
                                tzinfo=timezone.utc)  # returns a proper UTC datetime object
            elif 'GGA' in tag:             # if this is a GGA position sentence
                time_s  = fields[1]         # time field from the sentence
                fix_q   = fields[6]         # 0 means no fix
                if fix_q == '0' or not time_s:
                    return None             # discards if GPS has no satellite fix
                h = int(time_s[0:2])        # extracts hours
                m = int(time_s[2:4])        # extracts minutes
                s = float(time_s[4:])       # extracts seconds
                n = now_utc()               # gets current UTC date for year month day
                return datetime(n.year, n.month, n.day,
                                h, m, int(s),
                                int((s % 1) * 1e6),
                                tzinfo=timezone.utc)  # returns UTC datetime using today's date
        except Exception:
            pass                            # silently ignores any parsing errors
        return None                         # returns nothing if parsing failed

    # ========================================================
    # FPGA RX LOOP
    # ========================================================
    def _rx_loop(self):                     # background thread that receives FPGA data
        buf = bytearray()                   # empty byte buffer to collect incoming data
        self._log("Listening for pulse pairs...")  # logs that receiving has started
        while not self.stop_rx and self.is_connected:  # keeps running while connected
            try:
                if self.serial_port.in_waiting:  # if new bytes have arrived from FPGA
                    buf += self.serial_port.read(
                        self.serial_port.in_waiting)  # reads all available bytes into buffer

                # Frame sync: scan for 0xAA start byte then validate 6-byte frame
                while len(buf) >= BYTES_PER_FRAME:  # while enough bytes for a full frame
                    # Scan for 0xAA start byte
                    aa_pos = -1                     # position of start byte not found yet
                    for i in range(len(buf)):
                        if buf[i] == 0xAA:
                            aa_pos = i              # found the start byte position
                            break

                    if aa_pos == -1:                # if no start byte found at all
                        if len(buf) > 5:
                            buf = buf[-5:]          # keeps only last 5 bytes
                        break

                    if aa_pos > 0:                  # if start byte is not at position zero
                        buf = buf[aa_pos:]          # discards all bytes before start byte
                        continue

                    # buf[0] == 0xAA
                    if len(buf) < BYTES_PER_FRAME:  # if not enough bytes for full frame yet
                        break

                    frame = bytes(buf[:6])          # grabs the 6-byte frame
                    # Checksum: byte5 == byte1^byte2^byte3^byte4
                    expected_cs = (frame[1]^frame[2]^frame[3]^frame[4]) & 0xFF  # calculates checksum
                    if frame[5] == expected_cs:     # if checksum matches
                        wall_capture = time.time()  # records precise arrival time
                        buf = buf[6:]               # removes processed frame from buffer
                        self.root.after(
                            0, self._process_frame,
                            frame, wall_capture)    # sends frame to main thread for processing
                    else:
                        # Bad checksum -- skip 1 byte and resync
                        buf = buf[1:]               # discards one byte and tries again

                time.sleep(0.0001)          # waits 0.1ms to avoid maxing out the CPU
            except Exception as e:
                if self.is_connected:
                    self.root.after(
                        0, self._log, f"RX Error: {e}")  # logs receive error
                break                       # stops the thread on error

    def _process_frame(self, frame, wall_capture=None):  # processes one 6-byte FPGA frame
        # New format: [0xAA][B1][B2][B3][B4][checksum]
        b1, b2, b3, b4 = frame[1], frame[2], frame[3], frame[4]  # extracts 4 data bytes
        raw32     = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4  # combines bytes into 32-bit number
        is_rising = bool(raw32 & RF_FLAG_MASK)  # checks top bit to see if rising or falling
        raw_ts    = raw32 & TIMESTAMP_MASK  # extracts just the 31-bit counter value

        if wall_capture is None:            # if no arrival time was provided
            wall_capture = time.time()      # uses current PC time instead

        fpga_s = self._fpga_seconds(raw_ts) # converts counter value to seconds
        if fpga_s is None:                  # if timestamp was detected as garbage
            return                          # discards this frame entirely

        self._set_anchor(fpga_s, wall_capture)  # sets the time anchor on very first pulse

        wall_utc = self._wall_utc(fpga_s)   # converts FPGA time to real wall clock UTC

        self.total_edges += 1               # increments the total edge counter
        self._run_state_machine(is_rising, fpga_s, wall_utc)  # processes edge in sequence
        self._update_stats()                # refreshes the statistics display

    # ========================================================
    # EDGE STATE MACHINE
    # Collect R1 F1 R2 F2 in order
    # All validation done in Verilog
    # ========================================================
    def _run_state_machine(self, is_rising, fpga_s, wall_utc):  # collects four edges in order

        def make_edge(fpga_s, wall_utc):    # packages one edge into a dictionary
            return {'fpga_s': fpga_s, 'wall_utc': wall_utc}  # stores time in two formats

        MAX_WITHIN_PAIR_S = 0.001           # 1ms maximum time allowed within one pair

        if self.t_rise1 is None:            # waiting for first rising edge
            if is_rising:
                self.t_rise1 = make_edge(fpga_s, wall_utc)  # saves first pulse rising edge

        elif self.t_fall1 is None:          # waiting for first falling edge
            if not is_rising:               # must be a falling edge
                dt = fpga_s - self.t_rise1['fpga_s']  # time since first rising edge
                if 0 < dt < MAX_WITHIN_PAIR_S:
                    self.t_fall1 = make_edge(fpga_s, wall_utc)  # saves first pulse falling edge
                else:
                    # Bad timing -- reset, treat this as new R1 if rising
                    self.t_rise1 = None     # resets state machine back to start
                    self.t_fall1 = None

        elif self.t_rise2 is None:          # waiting for second rising edge
            if is_rising:                   # must be a rising edge
                dt = fpga_s - self.t_rise1['fpga_s']  # time since first rising edge
                if 0 < dt < MAX_WITHIN_PAIR_S:
                    self.t_rise2 = make_edge(fpga_s, wall_utc)  # saves second pulse rising edge
                else:
                    # Bad timing -- reset
                    self.t_rise1 = None     # resets state machine back to start
                    self.t_fall1 = None

        else:                               # waiting for second falling edge
            if not is_rising:               # must be a falling edge
                t_fall2 = make_edge(fpga_s, wall_utc)  # saves second pulse falling edge
                dt = fpga_s - self.t_rise2['fpga_s']   # time since second rising edge
                if not (0 < dt < MAX_WITHIN_PAIR_S):
                    # Bad timing -- reset
                    self.t_rise1 = None     # resets all four stored edges
                    self.t_fall1 = None
                    self.t_rise2 = None
                    return

                # Dt = rising to rising
                delta_t_us = (self.t_rise2['fpga_s'] -
                              self.t_rise1['fpga_s']) * 1e6  # time gap in microseconds

                p1_width_ns = ((self.t_fall1['fpga_s'] -
                                self.t_rise1['fpga_s']) * 1e9)  # first pulse width in nanoseconds
                p2_width_ns = ((t_fall2['fpga_s'] -
                                self.t_rise2['fpga_s']) * 1e9)  # second pulse width in nanoseconds

                # Sanity check -- discard physically impossible values
                if (p1_width_ns < 0 or p1_width_ns > 100000 or
                    p2_width_ns < 0 or p2_width_ns > 100000 or
                    delta_t_us  < 0 or delta_t_us  > 100000):  # checks values are physically realistic
                    self._log(
                        f"DISCARDED garbage pair: "
                        f"p1={p1_width_ns:.0f}ns "
                        f"p2={p2_width_ns:.0f}ns "
                        f"dt={delta_t_us:.3f}us")  # logs the discarded pair details
                    self.t_rise1 = None     # resets state machine after bad pair
                    self.t_fall1 = None
                    self.t_rise2 = None
                    return

                self.valid_pairs += 1       # increments the valid pair counter
                self.delta_t_list.append(delta_t_us)  # adds time gap to the list

                pair = {
                    'pair_num':    self.valid_pairs,    # sequential pair number
                    't_rise1':     self.t_rise1,        # first pulse rising edge data
                    't_fall1':     self.t_fall1,        # first pulse falling edge data
                    't_rise2':     self.t_rise2,        # second pulse rising edge data
                    't_fall2':     t_fall2,             # second pulse falling edge data
                    'p1_width_ns': p1_width_ns,         # first pulse width in nanoseconds
                    'p2_width_ns': p2_width_ns,         # second pulse width in nanoseconds
                    'delta_t_us':  delta_t_us,          # time gap between pulses in microseconds
                }
                self.pulse_pairs.append(pair)       # adds completed pair to the master list
                self._add_table_row(pair)           # adds new row to the data table
                self._update_waveform(pair)         # redraws the waveform chart

                self.t_rise1 = None         # resets all four edges ready for next pair
                self.t_fall1 = None
                self.t_rise2 = None

    # ========================================================
    # UI UPDATES
    # ========================================================
    def _update_stats(self):                # refreshes all six statistic labels
        self.stat_labels['edges'].config(
            text=str(self.total_edges))     # updates total edges count display
        self.stat_labels['pairs'].config(
            text=str(self.valid_pairs))     # updates valid pairs count display
        if self.delta_t_list:               # only calculates stats if there is data
            last = self.delta_t_list[-1]    # most recent time gap value
            mean = np.mean(self.delta_t_list)  # calculates average of all gaps
            mn   = np.min(self.delta_t_list)   # finds smallest gap recorded
            mx   = np.max(self.delta_t_list)   # finds largest gap recorded
            self.stat_labels['last_dt'].config(
                text=f"{last:.3f} us")      # shows last gap to 3 decimal places
            self.stat_labels['mean_dt'].config(
                text=f"{mean:.3f} us")      # shows average gap to 3 decimal places
            self.stat_labels['min_dt'].config(
                text=f"{mn:.3f} us")        # shows minimum gap to 3 decimal places
            self.stat_labels['max_dt'].config(
                text=f"{mx:.3f} us")        # shows maximum gap to 3 decimal places

    def _fmt_ist_display(self, wall_utc):   # formats UTC time as IST for display
        if wall_utc is None:
            return "--"                     # shows dashes if no time available
        return fmt_display(to_ist(wall_utc))  # converts to IST and formats nicely

    def _fmt_fpga(self, fpga_s):            # formats FPGA seconds to 9 decimal places
        return f"{fpga_s:.9f}"              # shows nanosecond precision in the table

    def _add_table_row(self, p):            # adds one completed pair as a table row
        r1u = self._fmt_ist_display(p['t_rise1']['wall_utc'])  # formats rise1 IST time
        f1u = self._fmt_ist_display(p['t_fall1']['wall_utc'])  # formats fall1 IST time
        r2u = self._fmt_ist_display(p['t_rise2']['wall_utc'])  # formats rise2 IST time
        f2u = self._fmt_ist_display(p['t_fall2']['wall_utc'])  # formats fall2 IST time

        self.tree.insert('', 'end', tags=('pair',),
                         values=(
                             p['pair_num'],
                             r1u,
                             self._fmt_fpga(p['t_rise1']['fpga_s']),
                             f1u,
                             self._fmt_fpga(p['t_fall1']['fpga_s']),
                             r2u,
                             self._fmt_fpga(p['t_rise2']['fpga_s']),
                             f2u,
                             self._fmt_fpga(p['t_fall2']['fpga_s']),
                             f"{p['p1_width_ns']:.0f}",
                             f"{p['p2_width_ns']:.0f}",
                             f"{p['delta_t_us']:.3f}",
                         ))                # inserts all values as one new table row
        kids = self.tree.get_children()    # gets list of all rows in table
        if kids:
            self.tree.see(kids[-1])        # scrolls table down to show newest row

    def _update_waveform(self, p):          # redraws the live waveform chart for latest pair
        self.ax_wf.clear()                  # clears the previous waveform drawing
        self._style_ax(self.ax_wf)          # reapplies dark theme styling

        ref = p['t_rise1']['fpga_s']        # uses first rising edge as the zero reference

        def us(e):                          # converts absolute FPGA seconds to relative microseconds
            return (e['fpga_s'] - ref) * 1e6  # subtracts reference and converts to microseconds

        r1 = us(p['t_rise1'])   # time of first rising edge in microseconds from reference
        f1 = us(p['t_fall1'])   # time of first falling edge in microseconds
        r2 = us(p['t_rise2'])   # time of second rising edge in microseconds
        f2 = us(p['t_fall2'])   # time of second falling edge in microseconds

        t_pts = [r1 - 0.5, r1, r1, f1, f1,
                 r2, r2, f2, f2, f2 + 0.5]  # time values for drawing both pulse shapes
        s_pts = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0]  # signal level 0=low 1=high for each point

        self.ax_wf.plot(t_pts, s_pts,
                        color=CYAN, linewidth=2.5)  # draws the pulse waveform line in cyan
        self.ax_wf.fill_between(t_pts, s_pts,
                                alpha=0.15, color=CYAN)  # adds light cyan shading under pulses

        for label, e, color in [
            ('T_rise1', p['t_rise1'], GREEN),
            ('T_fall1', p['t_fall1'], RED),
            ('T_rise2', p['t_rise2'], GREEN),
            ('T_fall2', p['t_fall2'], RED),
        ]:
            x = us(e)                       # converts each edge time to microseconds
            self.ax_wf.axvline(x, color=color,
                               linewidth=1,
                               linestyle='--', alpha=0.7)  # draws vertical dashed line at edge
            self.ax_wf.text(x, 1.35, label,
                            color=color, fontsize=7,
                            ha='center', rotation=90)  # adds rotated label above each line

        # Dt arrow: rise1 to rise2
        self.ax_wf.annotate(
            '', xy=(r2, 0.5), xytext=(r1, 0.5),
            arrowprops=dict(arrowstyle='<->',
                            color=YELLOW, lw=2))  # draws double-headed arrow between pulses
        self.ax_wf.text(
            (r1 + r2) / 2, 0.6,
            f"Dt(R-R) = {p['delta_t_us']:.3f} us",
            color=YELLOW, fontsize=9,
            ha='center', fontweight='bold')  # labels the arrow with the measured time gap

        self.ax_wf.set_ylim([-0.3, 1.7])    # sets vertical display range
        self.ax_wf.set_xlabel("Time (us)",
                               color=DIM, fontsize=10)   # labels horizontal axis
        self.ax_wf.set_ylabel("Signal",
                               color=DIM, fontsize=10)   # labels vertical axis
        self.ax_wf.set_title(
            f"Pair #{p['pair_num']}  |  "
            f"P1: {p['p1_width_ns']:.0f} ns  |  "
            f"P2: {p['p2_width_ns']:.0f} ns  |  "
            f"Dt(R-R): {p['delta_t_us']:.3f} us",
            color=CYAN, fontsize=10, pad=8)  # sets chart title showing all key measurements
        self.canvas_wf.draw()               # refreshes the chart display

    def _schedule_plot_update(self):        # schedules histogram to refresh every 500ms
        self._update_histogram()            # updates the histogram right now
        self.root.after(500, self._schedule_plot_update)  # schedules itself to run again

    def _update_histogram(self):            # redraws the delta-t histogram chart
        if len(self.delta_t_list) < 2:      # needs at least 2 values to draw
            return
        self.ax_hist.clear()                # clears the previous histogram
        self._style_ax(self.ax_hist)        # reapplies dark theme styling
        data = np.array(self.delta_t_list)  # converts list to numpy array for calculations
        bins = min(50, max(10, len(data) // 10))  # calculates sensible number of histogram bars
        n, edges, patches = self.ax_hist.hist(
            data, bins=bins, color=CYAN,
            alpha=0.75, edgecolor=BG_PANEL) # draws the histogram bars in cyan
        max_n = max(n) if max(n) > 0 else 1  # finds tallest bar for colour scaling
        for patch, count in zip(patches, n):
            patch.set_facecolor(
                matplotlib.colors.to_hex(
                    matplotlib.cm.plasma(count / max_n)))  # colours each bar by relative height
        self.ax_hist.set_xlabel("Dt R-R (us)",
                                 color=DIM, fontsize=10)   # labels horizontal axis
        self.ax_hist.set_ylabel("Count",
                                 color=DIM, fontsize=10)   # labels vertical axis
        self.ax_hist.set_title(
            f"Rise-to-Rise Dt Distribution  "
            f"|  N={len(data)}  |  "
            f"Mean={np.mean(data):.3f} us",
            color=CYAN, fontsize=10, pad=8)  # title showing sample count and mean
        self.canvas_hist.draw()             # refreshes the histogram display

    # ========================================================
    # EXPORT
    # ========================================================
    def _export_csv(self):                  # saves all pulse pair data to a CSV file
        if not self.pulse_pairs:
            messagebox.showinfo("No Data",
                                "No pulse pairs to export")  # shows message if nothing to save
            return
        fname = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=(
                f"muon_data_"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                f".csv"))                   # opens save dialog with auto-generated filename
        if not fname:
            return                          # cancels if user closed the dialog
        try:
            src = ("GPS (NEO-6M)"
                   if self.gps_connected
                   else "Laptop System Time")  # records which time source was used
            with open(fname, 'w', newline='',
                      encoding='utf-8') as f:  # opens the CSV file for writing
                w = csv.writer(f)           # creates a CSV writer object
                w.writerow(["MUON RECEIVER EXPORT"])        # writes file title row
                w.writerow(["Time Source:", src])           # writes which clock was used
                w.writerow(["Exported:",
                             datetime.now().isoformat()])   # writes when file was saved
                w.writerow(["Total Pairs:",
                             len(self.pulse_pairs)])        # writes total number of pairs
                w.writerow([])                              # writes blank separator row
                w.writerow([
                    'Pair#',
                    'T_rise1_IST(YYYYMMDDHHMMSSffffff)',
                    'T_rise1_FPGA(s)',
                    'T_fall1_IST(YYYYMMDDHHMMSSffffff)',
                    'T_fall1_FPGA(s)',
                    'T_rise2_IST(YYYYMMDDHHMMSSffffff)',
                    'T_rise2_FPGA(s)',
                    'T_fall2_IST(YYYYMMDDHHMMSSffffff)',
                    'T_fall2_FPGA(s)',
                    'P1_Width(ns)',
                    'P2_Width(ns)',
                    'Dt_RiseToRise(us)',
                ])                          # writes the column header row

                def csv_ist(wall_utc):      # formats wall time as IST for CSV
                    if wall_utc is None:
                        return "--"         # returns dashes if no time available
                    return fmt_csv(to_ist(wall_utc))  # converts and formats as long number

                for p in self.pulse_pairs:  # loops through every recorded pair
                    w.writerow([
                        p['pair_num'],
                        csv_ist(p['t_rise1']['wall_utc']),
                        f"{p['t_rise1']['fpga_s']:.9f}",
                        csv_ist(p['t_fall1']['wall_utc']),
                        f"{p['t_fall1']['fpga_s']:.9f}",
                        csv_ist(p['t_rise2']['wall_utc']),
                        f"{p['t_rise2']['fpga_s']:.9f}",
                        csv_ist(p['t_fall2']['wall_utc']),
                        f"{p['t_fall2']['fpga_s']:.9f}",
                        f"{p['p1_width_ns']:.0f}",
                        f"{p['p2_width_ns']:.0f}",
                        f"{p['delta_t_us']:.4f}",
                    ])                      # writes one data row for each pulse pair
            self._log(
                f"Exported {len(self.pulse_pairs)} pairs "
                f"to {fname}")              # logs successful export with filename
            messagebox.showinfo(
                "Export Complete",
                f"Saved {len(self.pulse_pairs)} pairs")  # shows success popup
        except Exception as e:
            self._log(f"Export failed: {e}")            # logs the export error
            messagebox.showerror("Export Error", str(e))  # shows error popup

    # ========================================================
    # RESET
    # ========================================================
    def _reset_all(self):                   # clears all recorded data and resets everything
        if not messagebox.askyesno(
                "Reset", "Clear all data?"):
            return                          # cancels if user clicks No in the dialog
        self.last_raw      = None           # resets the last FPGA counter reference
        self.wraparound    = 0              # resets the overflow counter to zero
        self.anchor_utc    = None           # clears the time anchor
        self.anchor_fpga_s = None           # clears the FPGA anchor time
        self.t_rise1       = None           # clears saved first rising edge
        self.t_fall1       = None           # clears saved first falling edge
        self.t_rise2       = None           # clears saved second rising edge
        self.pulse_pairs.clear()            # empties the list of all pulse pairs
        self.delta_t_list.clear()           # empties the list of all time gaps
        self.waveform_edges.clear()         # empties the waveform edge buffer
        self.total_edges   = 0              # resets total edge counter to zero
        self.valid_pairs   = 0              # resets valid pair counter to zero
        for key in self.stat_labels:
            self.stat_labels[key].config(text="--")  # resets all stat displays to dashes
        for item in self.tree.get_children():
            self.tree.delete(item)          # deletes every row from the data table
        self.ax_wf.clear()                  # clears the waveform chart
        self._style_ax(self.ax_wf)          # reapplies dark theme to waveform chart
        self.ax_wf.text(0.5, 0.5, "Waiting for pulses...",
                        ha='center', va='center',
                        color=DIM, fontsize=13,
                        transform=self.ax_wf.transAxes)  # shows placeholder text again
        self.canvas_wf.draw()               # refreshes the waveform display
        self.ax_hist.clear()                # clears the histogram chart
        self._style_ax(self.ax_hist)        # reapplies dark theme to histogram
        self.ax_hist.text(0.5, 0.5, "Waiting for data...",
                          ha='center', va='center',
                          color=DIM, fontsize=13,
                          transform=self.ax_hist.transAxes)  # shows placeholder text again
        self.canvas_hist.draw()             # refreshes the histogram display
        self._log("Reset complete")         # logs that reset finished successfully

    # ========================================================
    # RUN
    # ========================================================
    def _on_close(self):                    # runs when user clicks the X to close window
        if self.is_connected:
            self._disconnect_fpga()         # disconnects FPGA if still connected
        if self.gps_connected:
            self._disconnect_gps()          # disconnects GPS if still connected
        self.root.destroy()                 # closes and destroys the window

    def run(self):                          # starts the application running
        self.root.protocol("WM_DELETE_WINDOW",
                           self._on_close)  # links window close button to cleanup function
        self.root.mainloop()                # keeps the window open and responsive


# ============================================================
if __name__ == "__main__":                  # runs only when this file opened directly
    print("=" * 56)
    print("  MUON LIFETIME MEASUREMENT -- RECEIVER GUI")
    print("  921600 baud | 6-byte packets | GPS + laptop time")
    print("=" * 56)                         # prints startup banner in the terminal
    app = ReceiverGUI()                     # creates the receiver application object
    app.run()                               # starts the GUI and enters the main loop