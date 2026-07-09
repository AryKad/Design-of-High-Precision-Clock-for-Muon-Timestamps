"""
pulse_gen_gui.py  -  Tkinter GUI for Basys 3 Pulse Pair Generator
COM5 | 115200 8N1

Pulse pair timing diagram:
  |<-- width -->|<-- inter-pulse gap (variable) -->|<-- width -->|<-- pair gap -->|
  _______________                                   _______________
 |   PULSE A    |___________________________________|   PULSE B    |_______________|

Controls:
  Pulse Width      : 200 ns  - 500 ns   (10 ns  steps)
  Inter-pulse Gap  : 10 us   - 100 us   (100 ns steps)
  Pair Gap         : 333.3 us - 1000 us (100 ns steps)

Keyboard shortcuts:
  F11   - Toggle fullscreen
  Esc   - Exit fullscreen
"""

import tkinter as tk                        # toolkit to build the window and buttons
from tkinter import ttk, scrolledtext, messagebox  # extra window parts like dropdowns and popups
import serial                               # lets Python talk to the FPGA board
import serial.tools.list_ports             # finds all USB/COM ports on your computer
import threading                            # lets two tasks run at the same time
import time                                 # used for timestamps and small delays
import queue                                # a safe list for passing messages between threads

# ── Constants ─────────────────────────────────────────────────────────────────
COM_PORT   = "COM5"     # default USB port the FPGA board is plugged into
BAUD_RATE  = 115200     # speed of data transfer to the FPGA (bits per second)

WIDTH_MIN  = 200;   WIDTH_MAX  = 500;   WIDTH_STEP  = 10    # pulse width limits in nanoseconds
INTER_MIN  = 10;    INTER_MAX  = 100;   INTER_STEP  = 0.1   # gap between two pulses in microseconds
GAP_MIN    = 333;   GAP_MAX    = 1000;  GAP_STEP    = 0.1   # gap between pulse pairs in microseconds

# Internal units (100ns) for inter and gap
INTER_100NS_MIN = 100;  INTER_100NS_MAX = 1000    # same gap but counted in 100ns chunks
GAP_100NS_MIN   = 3334; GAP_100NS_MAX   = 10000   # same pair gap in 100ns chunks


class PulseGenGUI:                          # blueprint for the entire application window
    def __init__(self, root):               # runs once when the app starts
        self.root = root                    # saves the main window for later use
        self.root.title("Pulse Pair Generator  —  Basys 3")  # sets the window title bar text
        self.root.resizable(True, True)     # allows the window to be resized freely
        self.root.configure(bg="#1e1e2e")   # sets dark background colour for the window

        self._is_fullscreen = False         # tracks whether fullscreen is on or off

        self.serial_port = None             # no USB connection exists yet
        self.connected   = False            # board is not connected at startup
        self.running     = False            # pulses are not being generated yet
        self.rx_queue    = queue.Queue()    # creates an empty inbox for board messages

        self._build_ui()                    # draws all buttons, sliders, and labels
        self._poll_rx()                     # starts checking inbox for board replies

        # ── Keyboard shortcuts ────────────────────────────────────────────────
        self.root.bind("<F11>",  lambda e: self._toggle_fullscreen())   # F11 key toggles fullscreen
        self.root.bind("<Escape>", lambda e: self._exit_fullscreen())   # Escape key exits fullscreen

    # ── Fullscreen helpers ────────────────────────────────────────────────────
    def _toggle_fullscreen(self):                           # switches fullscreen on or off
        self._is_fullscreen = not self._is_fullscreen       # flips the true/false value
        self.root.attributes("-fullscreen", self._is_fullscreen)  # applies fullscreen to window
        self._update_fs_button()                            # updates the button label text

    def _exit_fullscreen(self):                             # turns off fullscreen only
        if self._is_fullscreen:                             # only acts if currently fullscreen
            self._is_fullscreen = False                     # marks fullscreen as off
            self.root.attributes("-fullscreen", False)      # shrinks window back to normal
            self._update_fs_button()                        # updates the button label text

    def _update_fs_button(self):                            # changes the fullscreen button text
        if self._is_fullscreen:                             # if currently fullscreen
            self.fs_btn.configure(text="⊠  Exit Fullscreen")   # show exit option
        else:                                               # if in normal window mode
            self.fs_btn.configure(text="⛶  Fullscreen  F11")   # show enter option

    # ── UI Construction ────────────────────────────────────────────────────────
    def _build_ui(self):                    # builds every visual element on screen
        PAD    = dict(padx=10, pady=6)      # standard spacing around each panel
        BG     = "#1e1e2e"                  # dark blue-black background colour
        FG     = "#cdd6f4"                  # soft white text colour
        ACC    = "#89b4fa"                  # blue accent colour for highlights
        GRN    = "#a6e3a1"                  # green colour for connect and start buttons
        RED    = "#f38ba8"                  # red colour for stop and disconnect buttons
        GREY   = "#313244"                  # dark grey for input box backgrounds
        FONT   = ("Consolas", 10)           # standard monospace font used everywhere
        FONT_LG= ("Consolas", 12, "bold")   # larger bold font for value displays

        style = ttk.Style()                 # creates a style manager for ttk widgets
        style.theme_use("clam")             # uses the clam theme as a base style
        style.configure("TScale",  background=BG, troughcolor=GREY,
                        sliderlength=18, sliderrelief="flat")   # styles the sliders
        style.configure("TFrame",  background=BG)               # styles frames to dark background
        style.configure("TLabel",  background=BG, foreground=FG, font=FONT)  # styles all labels
        style.configure("TButton", background=GREY, foreground=FG,
                        font=FONT, relief="flat", padding=4)    # styles all ttk buttons
        style.map("TButton",
                  background=[("active", ACC), ("disabled", "#45475a")],
                  foreground=[("active", "#1e1e2e"), ("disabled", "#6c7086")])  # button hover colours

        # ── Title ─────────────────────────────────────────────────────────────
        title_f = tk.Frame(self.root, bg="#181825", pady=8)  # dark strip at top of window
        title_f.pack(fill="x")              # stretches the title bar across full width

        tk.Label(title_f, text="⚡  Pulse Pair Generator",
                 bg="#181825", fg=ACC,
                 font=("Consolas", 14, "bold")).pack(side="left", padx=14)  # app name on left
        tk.Label(title_f, text="Basys 3  |  100 MHz  |  115200 baud",
                 bg="#181825", fg="#6c7086", font=FONT).pack(side="left", padx=4)  # board info text

        self.fs_btn = tk.Button(            # creates the fullscreen toggle button
            title_f, text="⛶  Fullscreen  F11",
            bg="#313244", fg=ACC,
            font=("Consolas", 9, "bold"),
            relief="flat", bd=0,
            activebackground=ACC, activeforeground="#1e1e2e",
            cursor="hand2",                 # shows hand cursor when hovering
            command=self._toggle_fullscreen # calls fullscreen function when clicked
        )
        self.fs_btn.pack(side="right", padx=14, pady=2, ipadx=6, ipady=3)  # places button on right

        # ── Scrollable main canvas ────────────────────────────────────────────
        outer = tk.Frame(self.root, bg=BG)  # container frame for everything below title
        outer.pack(fill="both", expand=True)  # fills all remaining window space

        self._scroll_canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)  # invisible scrollable area
        vsb = ttk.Scrollbar(outer, orient="vertical",
                            command=self._scroll_canvas.yview)  # vertical scrollbar on right
        self._scroll_canvas.configure(yscrollcommand=vsb.set)   # links scrollbar to canvas
        vsb.pack(side="right", fill="y")    # places scrollbar on far right
        self._scroll_canvas.pack(side="left", fill="both", expand=True)  # canvas fills rest of space

        main = tk.Frame(self._scroll_canvas, bg=BG)  # main content frame inside the canvas
        self._main_window = self._scroll_canvas.create_window(
            (0, 0), window=main, anchor="nw")  # places content frame at top-left of canvas

        def _on_frame_configure(e):         # runs whenever content size changes
            self._scroll_canvas.configure(
                scrollregion=self._scroll_canvas.bbox("all"))  # updates scrollable region size

        def _on_canvas_configure(e):        # runs whenever canvas is resized
            self._scroll_canvas.itemconfig(
                self._main_window, width=e.width)  # keeps content same width as canvas

        main.bind("<Configure>", _on_frame_configure)           # watches content for size changes
        self._scroll_canvas.bind("<Configure>", _on_canvas_configure)  # watches canvas for resize

        def _on_mousewheel(e):              # runs when mouse wheel is scrolled
            self._scroll_canvas.yview_scroll(int(-1*(e.delta/120)), "units")  # scrolls the canvas
        self._scroll_canvas.bind_all("<MouseWheel>", _on_mousewheel)  # applies scroll to whole window

        # ── Connection Panel ──────────────────────────────────────────────────
        conn_f = tk.LabelFrame(main, text=" Connection ", bg=BG, fg=ACC,
                               font=FONT, bd=1, relief="groove")  # box labelled Connection
        conn_f.grid(row=0, column=0, columnspan=3, sticky="ew", **PAD)  # spans full width at top

        tk.Label(conn_f, text="Port:", bg=BG, fg=FG, font=FONT).grid(
            row=0, column=0, padx=6, pady=4)  # label saying Port
        self.port_var = tk.StringVar(value=COM_PORT)  # variable storing selected port name
        self.port_cb  = ttk.Combobox(conn_f, textvariable=self.port_var,
                                      width=10, font=FONT, state="readonly")  # dropdown for COM ports
        self.port_cb.grid(row=0, column=1, padx=4)  # places dropdown next to label
        self._refresh_ports()               # fills dropdown with available ports now
        ttk.Button(conn_f, text="↺", width=3,
                   command=self._refresh_ports).grid(row=0, column=2, padx=2)  # refresh ports button
        self.conn_btn = tk.Button(conn_f, text="Connect", width=10,
                                   bg=GRN, fg="#1e1e2e",
                                   font=("Consolas", 10, "bold"),
                                   relief="flat", command=self._toggle_connect)  # connect/disconnect button
        self.conn_btn.grid(row=0, column=3, padx=10, pady=4)  # places connect button in row
        self.conn_led = tk.Canvas(conn_f, width=14, height=14, bg=BG,
                                   highlightthickness=0)  # small circle to show connection status
        self.conn_led.grid(row=0, column=4, padx=4)    # places status dot next to button
        self._draw_led(self.conn_led, "red")            # starts with red dot meaning disconnected

        # ── Pulse Width Panel ─────────────────────────────────────────────────
        self.width_var = tk.IntVar(value=200)           # stores current pulse width value (ns)

        def on_width_slide(v):              # runs when width slider is moved
            val = round(float(v) / 10) * 10  # rounds to nearest 10ns step
            self.width_var.set(val)         # saves the rounded value
            self.width_lbl.configure(text=f"{val} ns")  # updates the displayed number
            self._draw_diagram()            # redraws the timing diagram below

        def on_width_spin():                # runs when spinbox arrows are clicked
            self.width_lbl.configure(text=f"{self.width_var.get()} ns")  # updates displayed number
            self._draw_diagram()            # redraws the timing diagram

        pw_f = tk.LabelFrame(main, text=" Pulse Width ", bg=BG, fg=ACC,
                              font=FONT, bd=1, relief="groove")  # box labelled Pulse Width
        pw_f.grid(row=1, column=0, sticky="nsew", **PAD)  # placed in first column second row
        self.width_lbl = tk.Label(pw_f, text="200 ns", bg=BG, fg=FG,
                                   font=FONT_LG, width=12)  # big text showing current width
        self.width_lbl.pack(pady=(6,2))     # adds spacing above and below label
        ttk.Scale(pw_f, from_=200, to=500, orient="horizontal", length=190,
                  variable=self.width_var, command=on_width_slide).pack(
                  padx=10, pady=4)          # horizontal slider from 200 to 500ns
        rf = tk.Frame(pw_f, bg=BG); rf.pack(fill="x", padx=10)  # row for min/max labels
        tk.Label(rf, text="200 ns", bg=BG, fg="#6c7086", font=FONT).pack(side="left")   # min label
        tk.Label(rf, text="500 ns", bg=BG, fg="#6c7086", font=FONT).pack(side="right")  # max label
        sf = tk.Frame(pw_f, bg=BG); sf.pack(pady=(4,4))  # row for fine control spinbox
        tk.Label(sf, text="Fine:", bg=BG, fg=FG, font=FONT).pack(side="left", padx=4)  # Fine label
        tk.Spinbox(sf, from_=200, to=500, increment=10,
                   textvariable=self.width_var, width=6, font=FONT,
                   bg=GREY, fg=FG, insertbackground=FG, relief="flat",
                   command=on_width_spin).pack(side="left")  # number box with up/down arrows
        tk.Label(sf, text="ns", bg=BG, fg=FG, font=FONT).pack(side="left", padx=2)  # units label
        ttk.Button(pw_f, text="Send Width",
                   command=self._send_width).pack(pady=(2,8))  # button to send value to FPGA

        # ── Inter-pulse Gap Panel ─────────────────────────────────────────────
        self.inter_100ns = tk.IntVar(value=400)         # stores gap between pulses in 100ns units

        def on_inter_slide(v):              # runs when inter-pulse slider is moved
            val = round(float(v))           # rounds to nearest whole number
            self.inter_100ns.set(val)       # saves the value
            self.inter_lbl.configure(text=f"{val*0.1:.1f} µs")  # converts to microseconds and shows
            self._draw_diagram()            # redraws the timing diagram

        def on_inter_spin():                # runs when inter spinbox arrows clicked
            val = self.inter_100ns.get()    # reads current spinbox value
            self.inter_lbl.configure(text=f"{val*0.1:.1f} µs")  # updates displayed microseconds
            self._draw_diagram()            # redraws the timing diagram

        ip_f = tk.LabelFrame(main, text=" Inter-pulse Gap ", bg=BG, fg=ACC,
                              font=FONT, bd=1, relief="groove")  # box labelled Inter-pulse Gap
        ip_f.grid(row=1, column=1, sticky="nsew", **PAD)  # placed in middle column
        self.inter_lbl = tk.Label(ip_f, text="40.0 µs", bg=BG, fg=FG,
                                   font=FONT_LG, width=12)  # big text showing current gap
        self.inter_lbl.pack(pady=(6,2))     # adds spacing around label
        ttk.Scale(ip_f, from_=100, to=1000, orient="horizontal", length=190,
                  variable=self.inter_100ns, command=on_inter_slide).pack(
                  padx=10, pady=4)          # slider from 10us to 100us
        rf2 = tk.Frame(ip_f, bg=BG); rf2.pack(fill="x", padx=10)  # row for min/max labels
        tk.Label(rf2, text="10 µs",  bg=BG, fg="#6c7086", font=FONT).pack(side="left")   # min label
        tk.Label(rf2, text="100 µs", bg=BG, fg="#6c7086", font=FONT).pack(side="right")  # max label
        sf2 = tk.Frame(ip_f, bg=BG); sf2.pack(pady=(4,4))  # row for fine control spinbox
        tk.Label(sf2, text="Fine:", bg=BG, fg=FG, font=FONT).pack(side="left", padx=4)  # Fine label
        tk.Spinbox(sf2, from_=100, to=1000, increment=1,
                   textvariable=self.inter_100ns, width=6, font=FONT,
                   bg=GREY, fg=FG, insertbackground=FG, relief="flat",
                   command=on_inter_spin).pack(side="left")  # number box in 100ns steps
        tk.Label(sf2, text="×100ns", bg=BG, fg=FG, font=FONT).pack(side="left", padx=2)  # units label
        ttk.Button(ip_f, text="Send Inter",
                   command=self._send_inter).pack(pady=(2,8))  # button to send gap to FPGA

        # ── Pair Gap Panel ────────────────────────────────────────────────────
        self.gap_100ns = tk.IntVar(value=3334)          # stores time between pulse pairs in 100ns units

        def on_gap_slide(v):                # runs when pair gap slider is moved
            val = round(float(v))           # rounds to whole number
            self.gap_100ns.set(val)         # saves the value
            self.gap_lbl.configure(text=f"{val*0.1:.1f} µs")  # converts and shows in microseconds
            self._draw_diagram()            # redraws the timing diagram

        def on_gap_spin():                  # runs when pair gap spinbox clicked
            val = self.gap_100ns.get()      # reads current spinbox value
            self.gap_lbl.configure(text=f"{val*0.1:.1f} µs")  # updates displayed microseconds
            self._draw_diagram()            # redraws the timing diagram

        gap_f = tk.LabelFrame(main, text=" Pair Gap ", bg=BG, fg=ACC,
                               font=FONT, bd=1, relief="groove")  # box labelled Pair Gap
        gap_f.grid(row=1, column=2, sticky="nsew", **PAD)  # placed in third column
        self.gap_lbl = tk.Label(gap_f, text="333.4 µs", bg=BG, fg=FG,
                                 font=FONT_LG, width=12)  # big text showing current pair gap
        self.gap_lbl.pack(pady=(6,2))       # adds spacing around label
        ttk.Scale(gap_f, from_=3334, to=10000, orient="horizontal", length=190,
                  variable=self.gap_100ns, command=on_gap_slide).pack(
                  padx=10, pady=4)          # slider from 333us to 1000us
        rf3 = tk.Frame(gap_f, bg=BG); rf3.pack(fill="x", padx=10)  # row for min/max labels
        tk.Label(rf3, text="333 µs",  bg=BG, fg="#6c7086", font=FONT).pack(side="left")   # min label
        tk.Label(rf3, text="1000 µs", bg=BG, fg="#6c7086", font=FONT).pack(side="right")  # max label
        sf3 = tk.Frame(gap_f, bg=BG); sf3.pack(pady=(4,4))  # row for fine control spinbox
        tk.Label(sf3, text="Fine:", bg=BG, fg=FG, font=FONT).pack(side="left", padx=4)  # Fine label
        tk.Spinbox(sf3, from_=3334, to=10000, increment=1,
                   textvariable=self.gap_100ns, width=8, font=FONT,
                   bg=GREY, fg=FG, insertbackground=FG, relief="flat",
                   command=on_gap_spin).pack(side="left")  # number box for fine gap control
        tk.Label(sf3, text="×100ns", bg=BG, fg=FG, font=FONT).pack(side="left", padx=2)  # units label
        ttk.Button(gap_f, text="Send Gap",
                   command=self._send_gap).pack(pady=(2,8))  # button to send gap to FPGA

        # ── Timing Diagram ────────────────────────────────────────────────────
        diag_f = tk.LabelFrame(main, text=" Timing Diagram ", bg=BG, fg=ACC,
                                font=FONT, bd=1, relief="groove")  # box showing pulse drawing
        diag_f.grid(row=2, column=0, columnspan=3, sticky="ew", **PAD)  # spans full width
        self.canvas = tk.Canvas(diag_f, width=620, height=90,
                                 bg="#11111b", highlightthickness=0)  # dark drawing area for pulses
        self.canvas.pack(padx=8, pady=6)    # adds padding around the drawing area
        self._draw_diagram()                # draws the initial pulse diagram on startup

        # ── Start / Stop Buttons ──────────────────────────────────────────────
        ctrl_f = tk.Frame(main, bg=BG)      # row of control buttons
        ctrl_f.grid(row=3, column=0, columnspan=3, pady=8)  # placed below the diagram

        self.start_btn = tk.Button(ctrl_f, text="▶  START", width=14, height=2,
                                    bg=GRN, fg="#1e1e2e",
                                    font=("Consolas", 11, "bold"),
                                    relief="flat", state="disabled",
                                    command=self._start)  # green button to begin generating pulses
        self.start_btn.grid(row=0, column=0, padx=12)  # placed on left of controls row

        self.stop_btn = tk.Button(ctrl_f, text="■  STOP", width=14, height=2,
                                   bg=RED, fg="#1e1e2e",
                                   font=("Consolas", 11, "bold"),
                                   relief="flat", state="disabled",
                                   command=self._stop)  # red button to stop generating pulses
        self.stop_btn.grid(row=0, column=1, padx=12)   # placed next to start button

        ttk.Button(ctrl_f, text="⟳ Query Status",
                   command=self._query).grid(row=0, column=2, padx=12)  # asks FPGA for its status

        ttk.Button(ctrl_f, text="↑ Send All",
                   command=self._send_all).grid(row=0, column=3, padx=12)  # sends all three values at once

        # ── Log Panel ─────────────────────────────────────────────────────────
        log_f = tk.LabelFrame(main, text=" Log ", bg=BG, fg=ACC,
                               font=FONT, bd=1, relief="groove")  # box showing message history
        log_f.grid(row=4, column=0, columnspan=3, sticky="ew", **PAD)  # spans full width at bottom
        self.log = scrolledtext.ScrolledText(
            log_f, width=72, height=7, bg="#11111b", fg=FG,
            font=("Consolas", 9), relief="flat", state="disabled",
            insertbackground=FG)            # scrollable text box for sent and received messages
        self.log.pack(padx=6, pady=6)       # adds padding around the log box
        self.log.tag_config("tx",  foreground="#89dceb")   # sent messages shown in cyan
        self.log.tag_config("rx",  foreground="#a6e3a1")   # received messages shown in green
        self.log.tag_config("err", foreground="#f38ba8")   # error messages shown in red
        self.log.tag_config("inf", foreground="#f9e2af")   # info messages shown in yellow

        self._log("GUI started. Click Connect to open COM5.", "inf")   # startup message in log
        self._log("Tip: Press F11 or click ⛶ Fullscreen to toggle fullscreen.", "inf")  # tip message

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _draw_led(self, canvas, color):     # draws a small coloured circle dot
        canvas.delete("all")                # clears the canvas first
        canvas.create_oval(2, 2, 12, 12, fill=color, outline="")  # draws filled circle

    def _draw_diagram(self):                # redraws the pulse timing picture
        c = self.canvas                     # shortcut to the drawing canvas
        c.delete("all")                     # clears old drawing first
        W, H   = 620, 90                    # width and height of drawing area
        base   = 65                         # vertical position of the flat baseline
        HIGH   = 18                         # vertical position of the pulse tops
        x0     = 20                         # starting x position on left side

        width_ns  = self.width_var.get()            # reads current pulse width in ns
        inter_us  = self.inter_100ns.get() * 0.1    # converts inter-pulse gap to microseconds
        gap_us    = self.gap_100ns.get() * 0.1      # converts pair gap to microseconds

        total_us  = width_ns/1000*2 + inter_us + gap_us  # total time span in microseconds
        scale     = 560 / total_us          # pixels per microsecond for the drawing

        pw_px     = max(3, width_ns / 1000 * scale)  # pulse width in pixels (minimum 3)
        ip_px     = inter_us * scale        # inter-pulse gap in pixels
        gap_px    = gap_us * scale          # pair gap in pixels

        c.create_line(x0, base, x0+580, base, fill="#45475a", width=1)  # draws horizontal baseline

        xa1, xa2 = x0, x0 + pw_px          # start and end x of first pulse
        c.create_line(xa1, base, xa1, HIGH, xa2, HIGH, xa2, base,
                      fill="#89b4fa", width=2)  # draws first pulse shape in blue
        c.create_text((xa1+xa2)/2, HIGH-7, text="A",
                      fill="#cdd6f4", font=("Consolas", 8))  # labels first pulse A

        xb1 = xa2 + ip_px                   # start x of second pulse
        mid_ip = (xa2 + xb1) / 2            # midpoint of the inter-pulse gap
        c.create_line(xa2, base+12, xb1, base+12,
                      fill="#cba6f7", width=1, arrow="both")  # draws arrow showing gap size
        c.create_text(mid_ip, base+24,
                      text=f"{inter_us:.1f} µs (inter-pulse)",
                      fill="#cba6f7", font=("Consolas", 8))   # labels the gap measurement

        xb2 = xb1 + pw_px                   # end x of second pulse
        c.create_line(xb1, base, xb1, HIGH, xb2, HIGH, xb2, base,
                      fill="#89b4fa", width=2)  # draws second pulse shape in blue
        c.create_text((xb1+xb2)/2, HIGH-7, text="B",
                      fill="#cdd6f4", font=("Consolas", 8))   # labels second pulse B

        xc1 = min(xb2 + gap_px, x0 + 578)  # end x of pair gap (capped at canvas edge)
        mid_g = (xb2 + xc1) / 2            # midpoint of the pair gap
        c.create_line(xb2, base+12, xc1, base+12,
                      fill="#a6e3a1", width=1, arrow="both")  # draws arrow showing pair gap
        c.create_text(mid_g, base+24,
                      text=f"{gap_us:.1f} µs (pair gap)",
                      fill="#a6e3a1", font=("Consolas", 8))   # labels the pair gap measurement

        c.create_text(x0+290, 7,
                      text=f"Width: {width_ns} ns   Inter: {inter_us:.1f} µs   Gap: {gap_us:.1f} µs",
                      fill="#cba6f7", font=("Consolas", 9))   # shows all three values at top

    def _log(self, msg, tag="inf"):         # adds a timestamped message to the log box
        self.log.configure(state="normal")  # temporarily unlocks the log for writing
        ts = time.strftime("%H:%M:%S")      # gets current time as HH:MM:SS string
        self.log.insert("end", f"[{ts}] {msg}\n", tag)  # adds message with timestamp
        self.log.see("end")                 # scrolls log down to show newest message
        self.log.configure(state="disabled")  # locks the log again to prevent typing

    def _refresh_ports(self):               # finds all available COM ports on the computer
        ports = [p.device for p in serial.tools.list_ports.comports()]  # gets list of port names
        self.port_cb["values"] = ports      # puts the port names in the dropdown
        if COM_PORT in ports:               # if the default COM5 is available
            self.port_var.set(COM_PORT)     # selects COM5 automatically
        elif ports:                         # if COM5 not found but others exist
            self.port_var.set(ports[0])     # selects the first available port

    # ── Connect ───────────────────────────────────────────────────────────────
    def _toggle_connect(self):              # switches between connect and disconnect
        if self.connected: self._disconnect()   # if connected, disconnect
        else: self._connect()               # if disconnected, connect

    def _connect(self):                     # opens the serial connection to the FPGA
        port = self.port_var.get()          # reads which port is selected in dropdown
        try:
            self.serial_port = serial.Serial(port, BAUD_RATE,
                                              timeout=0.1, write_timeout=1.0)  # opens USB connection
            self.connected = True           # marks that we are now connected
            self.conn_btn.configure(text="Disconnect", bg="#f38ba8")  # button turns red saying Disconnect
            self._draw_led(self.conn_led, "#a6e3a1")  # status dot turns green
            self.start_btn.configure(state="normal")  # enables the start button
            self.stop_btn.configure(state="normal")   # enables the stop button
            self._log(f"Connected to {port} @ {BAUD_RATE} baud", "inf")  # logs success message
            threading.Thread(target=self._rx_thread, daemon=True).start()  # starts listening for replies
        except serial.SerialException as e:     # if connection fails
            messagebox.showerror("Connection Error", str(e))  # shows error popup
            self._log(f"Connection failed: {e}", "err")       # logs the error

    def _disconnect(self):                  # closes the serial connection cleanly
        self.connected = False              # marks as disconnected immediately
        if self.serial_port and self.serial_port.is_open:  # if port is still open
            try: self.serial_port.close()   # closes the USB connection
            except Exception: pass          # ignores any errors while closing
        self.conn_btn.configure(text="Connect", bg="#a6e3a1")  # button turns green saying Connect
        self._draw_led(self.conn_led, "red")    # status dot turns red
        self.start_btn.configure(state="disabled")  # disables start button
        self.stop_btn.configure(state="disabled")   # disables stop button
        self.running = False                # marks that pulses are no longer running
        self._log("Disconnected.", "inf")   # logs disconnection message

    def _rx_thread(self):                   # background thread that listens for FPGA replies
        buf = b""                           # empty buffer to collect incoming bytes
        while self.connected:               # keeps running while board is connected
            try:
                if self.serial_port.in_waiting:      # if new bytes have arrived
                    buf += self.serial_port.read(self.serial_port.in_waiting)  # reads all waiting bytes
                    while b"\n" in buf:              # while there is a complete line
                        line, buf = buf.split(b"\n", 1)  # splits off one complete line
                        self.rx_queue.put(line.decode("ascii","replace").strip())  # adds line to inbox
                else:
                    time.sleep(0.01)        # waits 10ms if nothing arrived yet
            except Exception as e:
                self.rx_queue.put(f"[RX ERROR] {e}"); break  # logs error and stops thread

    def _poll_rx(self):                     # checks inbox for new messages every 100ms
        while not self.rx_queue.empty():    # while there are unread messages
            self._log(f"← {self.rx_queue.get()}", "rx")  # shows each message in log
        self.root.after(100, self._poll_rx)  # schedules itself to run again in 100ms

    # ── Send ──────────────────────────────────────────────────────────────────
    def _send(self, cmd):                   # sends a text command to the FPGA board
        if not self.connected:
            self._log("Not connected.", "err"); return  # stops if not connected
        try:
            self.serial_port.write((cmd + "\n").encode("ascii"))  # sends command as bytes
            self._log(f"→ {cmd}", "tx")     # logs the sent command in cyan
        except serial.SerialException as e:
            self._log(f"TX error: {e}", "err")  # logs send failure in red

    def _send_width(self):                  # sends pulse width command to FPGA
        w = self.width_var.get()            # reads current width slider value
        w = max(200, min(500, round(w / 10) * 10))  # clamps and rounds to valid range
        self._send(f"W{w // 10:03d}")       # sends W command e.g. W020 for 200ns
        self._draw_diagram()                # refreshes the timing diagram

    def _send_inter(self):                  # sends inter-pulse gap command to FPGA
        i = max(100, min(1000, self.inter_100ns.get()))  # clamps value to valid range
        self._send(f"I{i:04d}")             # sends I command e.g. I0400 for 40us
        self._draw_diagram()                # refreshes the timing diagram

    def _send_gap(self):                    # sends pair gap command to FPGA
        g = max(3334, min(10000, self.gap_100ns.get()))  # clamps value to valid range
        self._send(f"G{g:05d}")             # sends G command e.g. G03334 for 333us
        self._draw_diagram()                # refreshes the timing diagram

    def _send_all(self):                    # sends width, inter, and gap one after another
        self._send_width()                  # sends width command immediately
        self.root.after(150, self._send_inter)  # sends inter command after 150ms
        self.root.after(300, self._send_gap)    # sends gap command after 300ms

    def _start(self):                       # sends start command to begin pulse generation
        self._send("S"); self.running = True  # sends S command and marks as running

    def _stop(self):                        # sends stop command to halt pulse generation
        self._send("T"); self.running = False  # sends T command and marks as stopped

    def _query(self):                       # asks the FPGA to report its current settings
        self._send("?")                     # sends question mark command to FPGA


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":                  # runs only when this file is opened directly
    root = tk.Tk()                          # creates the main application window
    PulseGenGUI(root)                       # builds the full GUI inside the window
    root.mainloop()                         # keeps the window open until user closes it