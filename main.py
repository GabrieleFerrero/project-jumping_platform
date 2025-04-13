import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import serial
from serial.tools import list_ports
import json
from abc import ABC, abstractmethod
import threading
import time
import queue
import numpy as np
from collections import deque
from functools import partial
from itertools import product
import math



class TimeoutException(Exception):
    """Custom exception for timeout"""
    pass

class ArduinoCommunicationException(Exception):
    """Custom exception for Arduino communication"""
    pass


class Code():
    
    code_list = ["ERROR", "OK", "INFO", "NULL"]

    def __init__(self, code):

        if code in Code.code_list:
            self.code=code
        else:
            raise Exception("Non-existent code!")
        
    def __eq__(self, other):
        if isinstance(other, Code):
            return self.code==other.code
        else: raise TypeError("The object passed is not of the expected type.")

    def __str__(self):
        return str(self.code)


    

class IODevice(ABC):
    def __init__(self, info_device={}, device_timeout=5):
        
        self.info_device = info_device
        self.device_timeout=device_timeout

        self.device_connection = None

        self.lock_IO = threading.RLock()

    def function_execution_with_timeout(self, timeout, func, *args, **kwargs):
        """
        Wrapper that executes a function with a time limit.
        :param timeout: Maximum execution time in seconds
        :param func: The function to execute
        :param args: Positional arguments of the function
        :param kwargs: Keyword arguments of the function
        :return: The return value of the function if executed in time
        :raises TimeoutException: If the function takes too long
        """

        #print(f"Function: {func}")

        # Function to execute the target function
        def target(q, *args, **kwargs):
            try:
                result = func(*args, **kwargs)
                q.put(result)  # Put the result in the queue
            except Exception as e:
                q.put(e)

        # Let's create a queue to get the result from the function
        q = queue.Queue()
        
        # We run the function in a separate thread
        thread = threading.Thread(target=target, args=(q, *args), kwargs=kwargs)
        thread.start()

        # We wait for the function to finish or for the timeout to expire
        thread.join(timeout)

        if thread.is_alive():
            thread.join()
            raise TimeoutException(f"Function execution exceeded {timeout} seconds.")

        if not q.empty():
            result = q.get()
            if isinstance(result, Exception):
                raise result
            return result
        return None  # In case the function returns nothing

    

    @abstractmethod
    def _check_connection(self):
        pass
    
    @abstractmethod
    def _close_connection(self):
        pass
    
    @abstractmethod
    def _open_connection(self):
        pass
    
    @abstractmethod
    def _read(self):
        pass
    
    @abstractmethod
    def _write(self, data):
        pass

    @abstractmethod
    def _reset_buffer(self):
        pass

    def set_info_device(self, info_device):
        self.info_device=info_device    

    def reset_buffer(self):
        with self.lock_IO:
            self.function_execution_with_timeout(self.device_timeout, self._reset_buffer)

    def check_connection(self):
        with self.lock_IO:
            try: return self.function_execution_with_timeout(self.device_timeout, self._check_connection)
            except: return False
    
    def close_connection(self):
        with self.lock_IO:
            try: self.function_execution_with_timeout(self.device_timeout, self._close_connection)
            except: pass
            self.device_connection = None
    
    def open_connection(self):
        with self.lock_IO:
            self.close_connection()
            self.device_connection = self.function_execution_with_timeout(self.device_timeout, self._open_connection)
            self.reset_buffer()
    
    def read(self):
        with self.lock_IO:
            return self.function_execution_with_timeout(self.device_timeout, self._read)
    
    def write(self, data):
        with self.lock_IO:
            self.function_execution_with_timeout(self.device_timeout, self._write, data)
    
    def write_and_read(self, data):
        with self.lock_IO:
            self.write(data)
            return self.read()


class USBIODevice(IODevice):
    pass
    

class Arduino(USBIODevice):
    def __init__(self, info_device={}, device_timeout=5):
        super().__init__(info_device, device_timeout)
    
    def _reset_buffer(self):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        self.device_connection.reset_input_buffer()

    def _check_connection(self):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        if self.device_connection.is_open:
            self._write({"command":"is_alive"})
            message_arduino_read = self._read()
            if message_arduino_read is not None:
                if message_arduino_read["code"]=="OK": return True
        return False
    
    def _close_connection(self):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        self.device_connection.close()
    
    def _open_connection(self):
        if not (isinstance(self.info_device, dict)
            and "port" in self.info_device.keys()
            and self.info_device["port"]
            and "baudrate" in self.info_device.keys()
            and isinstance(self.info_device["baudrate"], int)
            and self.info_device["baudrate"] > 0
            ):
            raise ValueError("Incorrect parameters")

        device_connection=serial.Serial(port=self.info_device["port"], baudrate=self.info_device["baudrate"], timeout=self.device_timeout+1)
        time.sleep(1)
        return device_connection

    
    def _read(self):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        response=""
        exit_while=False
        while exit_while==False:
            char=self.device_connection.read(size=1).decode(encoding='utf-8')
            if char:
                if char=='?': response=""
                elif char=='!': exit_while=True
                elif char=='\r': pass
                elif char=='\n': pass
                else: response += char
            else:
                raise ArduinoCommunicationException("Timeout expired while waiting for data")

        if response: 
            result = json.loads(response)
            if not (result["code"] == "OK"): raise ArduinoCommunicationException("Bad json")
            return result["response"]
        else: return json.loads("{}")
    
    def _write(self, data):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        if not (isinstance(data, dict)):
            raise ValueError("Incorrect parameters")
        
        self.device_connection.write(f"?{json.dumps(data)}!".encode())


class RawData(dict):
    def __getitem__(self, key):
        if key == 'time':
            time = self['unsliced_time']
            size = self['size']
            return time[:size]
        elif key == 'lc':
            lc = self['unsliced_lc']
            size = self['size']
            return lc[:, :size]

        return super().__getitem__(key)



class DataRappresentor():
    def __init__(self):

        self.after_id=None
        self.raw_data = None
        self.jump_data={
            "mass": 0.0,
            "acceleration_of_gravity": 9.81
        }
        self.stop_representation=False # force stop representation
        self.request_stop_representation=False 
    
        self.time_sleep=2 # ms (milliseconds)
        
        self.initializationRawData(0)
        self.stopRepresentation()


    def run(self, func1, args1, func2, args2):
        if not self.stop_representation:
            try:
                data = data_dq.popleft()
                if data is not None:
                    self.addData(data)
                    if not self.request_stop_representation:
                        smart_update_label(label_status_acquisition, "Acquisition in progress...", "green")
                        func1(*args1)
                    elif self.request_stop_representation:
                        smart_update_label(label_status_acquisition, "Saving the last samples...", "orange")
                        while not self.stop_representation:
                            data = data_dq.popleft()
                            self.addData(data)

                else: self.analysisError()
            except IndexError as e: # deque is empty
                #print(f"IndexError: {e}")
                if not self.request_stop_representation:
                    smart_update_label(label_status_acquisition, "Waiting for data...", "blue")
                elif self.request_stop_representation:
                    func1(*args1)
                    self.stopRepresentation()
                    smart_update_label(label_status_acquisition, "Analysis in progress...", "purple")
                    func2(*args2)
                    smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
            except tk.TclError as e:
                #print(f"TclError: {e}")
                pass
            except Exception as e:
                #print(f"Exception: {e}")
                self.analysisError()

            func = partial(self.run, func1, args1, func2, args2)
            self.after_id = root.after(self.time_sleep, func)


    def analysisError(self):
        smart_update_label(label_status_acquisition, "Error in analysis!", "red")
        self.stopRepresentation()
        if not check_device_connection():
            close_device_connection()
            func_Focus_combobox_menu_device(None)


    def initializationRawData(self, max_num_samples):
        raw_data = RawData()
        raw_data["initial_time"] = 0.0
        raw_data["unsliced_time"] = np.zeros(max_num_samples, dtype=float)
        raw_data["unsliced_lc"] = np.zeros((len(info_load_cells), max_num_samples), dtype=float)
        raw_data["size"] = 0
        raw_data["max_size"] = max_num_samples
        self.raw_data = raw_data
    

    def launchAnalysis(self, x, y, labels, default_min, default_max, analyze_callback):

        def find_intervals_for_vectors(matrix, range_mask):
            if matrix.size==0: return []

            intervals = []
            
            for i in range(matrix.shape[0]):
                vector = matrix[i]
                mask = (vector >= range_mask[0]) & (vector <= range_mask[1])
                vector_intervals = set()
                start = None

                for j, val in enumerate(mask):
                    if val and start is None:
                        start = j
                    elif not val and start is not None:
                        vector_intervals.add((start, j-1))
                        start = None
                if start is not None: vector_intervals.add((start, len(mask)-1))
                
                intervals.append((vector_intervals))
            
            return intervals



        def range_intersection(intervals):
            start = max(i[0] for i in intervals)
            end = min(i[1] for i in intervals)
            if start <= end: return (start, end)
            return None

        def range_union(intervals):
            start = min(i[0] for i in intervals)
            end = max(i[1] for i in intervals)
            if start <= end: return (start, end)
            return None

        def overlaps(a, b):
            return not (a[1] < b[0] or a[0] > b[1])

        def find_valid_non_overlapping_intervals(sets, min_len=1, max_len=float("inf")):
            preliminary = []

            # FASE 1: Trova tutte le intersezioni valide con lunghezza ammessa
            for combo in product(*sets):
                intersection_range = range_intersection(combo)
                if intersection_range:
                    length = intersection_range[1] - intersection_range[0] + 1
                    if min_len <= length <= max_len:
                        union_range = range_union(combo)
                        if union_range:
                            preliminary.append((intersection_range, combo, union_range))


            # FASE 2: Costruisci grafo dei conflitti tra le union_range
            n = len(preliminary)
            conflict_graph = {i: set() for i in range(n)}

            for i in range(n):
                for j in range(i + 1, n):
                    if overlaps(preliminary[i][2], preliminary[j][2]):
                        conflict_graph[i].add(j)
                        conflict_graph[j].add(i)


            # FASE 3: Trova insiemi conflittuali (componenti connesse > 1 → vanno eliminate)
            visited = set()
            to_remove = set()

            def dfs(node, component):
                if node in visited:
                    return
                visited.add(node)
                component.append(node)
                for neighbor in conflict_graph[node]:
                    dfs(neighbor, component)

            for i in range(n):
                if i not in visited:
                    component = []
                    dfs(i, component)
                    if len(component) > 1:
                        to_remove.update(component)


            # FASE 4: Ritorna solo quelli non coinvolti nei conflitti
            final = [
                (inter, combo)
                for idx, (inter, combo, _) in enumerate(preliminary)
                if idx not in to_remove
            ]

            return final

        range_mask_min = tk.DoubleVar(value=default_min)
        range_mask_max = tk.DoubleVar(value=default_max)

        win = tk.Toplevel(root)
        win.title("Data Analyzed")


        fig, ax = plt.subplots(figsize=(10, 4))
        lines = [ax.plot(x, y_row, label=label)[0] for y_row, label in zip(y, labels)]
        visible_window = 5000 if len(x) >= 5000 else len(x) # 5000 is the number of visible samples
        ax.set_xlim(0, x[visible_window-1])
        offset_ylim = 2
        ax.set_ylim(np.min(y)-offset_ylim, np.max(y)+offset_ylim)
        ax.legend(loc="upper right")

        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.grid(row=0, column=1, sticky="nsew")

        # Linee orizzontali fucsia e verde
        lines_range = {
            "min": ax.axhline(range_mask_min.get(), color='fuchsia', linestyle='--'),
            "max": ax.axhline(range_mask_max.get(), color='green', linestyle='--')
        }

        highlighted_patches = []
        selected_interval_idx = 0
        current_intervals = []

        def update_lines():
            lines_range["min"].set_ydata([range_mask_min.get()])
            lines_range["max"].set_ydata([range_mask_max.get()])

        def refresh_intervals(updating_from):
            nonlocal current_intervals
            nonlocal selected_interval_idx

            min_val = range_mask_min.get()
            max_val = range_mask_max.get()

            # Blocca min <= max
            if min_val > max_val:
                if updating_from:
                    if updating_from == "min":
                        min_val = max_val
                        range_mask_min.set(min_val)
                    elif updating_from == "max":
                        max_val = min_val
                        range_mask_max.set(max_val)
                else:
                    raise Exception("Problem with Scale min/max limits")

            # Aggiorna linee
            lines_range["min"].set_ydata([min_val])
            lines_range["max"].set_ydata([max_val])

            intervals = find_intervals_for_vectors(y, (min_val, max_val))
            valid = find_valid_non_overlapping_intervals(intervals)
            current_intervals = sorted(valid, key=lambda x: x[0][0])

            #print(current_intervals)
            
            selected_interval_idx = 0

            for patch in highlighted_patches:
                patch.remove()
            highlighted_patches.clear()


            for i, ((start, end), _) in enumerate(current_intervals):
                color = 'lightcoral' if i == selected_interval_idx else 'lightblue'
                patch = ax.axvspan(start, end, color=color, alpha=0.3, zorder=0)
                highlighted_patches.append(patch)

            fig.canvas.draw_idle()

        def on_scroll_min(_):
            update_lines()
            refresh_intervals(updating_from="min")

        def on_scroll_max(_):
            update_lines()
            refresh_intervals(updating_from="max")

        # --- Layout dinamico ---
        win.columnconfigure(1, weight=1)
        win.rowconfigure(0, weight=1)

        # Scroll verticali a sinistra e destra
        offset_scroll = 1
        min_scroll = ttk.Scale(win, from_=np.max(y)+offset_scroll, to=np.min(y)-offset_scroll, variable=range_mask_min,
                        orient='vertical', command=on_scroll_min)
        max_scroll = ttk.Scale(win, from_=np.max(y)+offset_scroll, to=np.min(y)-offset_scroll, variable=range_mask_max,
                            orient='vertical', command=on_scroll_max)

        min_scroll.grid(row=0, column=0, sticky="ns", padx=(5, 0))
        max_scroll.grid(row=0, column=2, sticky="ns", padx=(0, 5))

        # Scroll orizzontale per muovere il grafico
        scroll_var = tk.DoubleVar(value=0)

        def update_xlim(val):
            start = int(scroll_var.get())
            ax.set_xlim(x[start], x[start + visible_window - 1])
            fig.canvas.draw_idle()

        scrollbar = ttk.Scale(win, from_=0, to=len(x)-visible_window,
                            variable=scroll_var, orient='horizontal', command=update_xlim)
        scrollbar.grid(row=1, column=0, columnspan=3, sticky="ew", pady=5)

        def on_click(event):
            nonlocal selected_interval_idx
            if not current_intervals or event.inaxes != ax:
                return
            clicked_x = int(event.xdata)
            for i, ((start, end), _) in enumerate(current_intervals):
                if start <= clicked_x <= end:
                    prev = selected_interval_idx
                    selected_interval_idx = i
                    highlighted_patches[prev].set_color('lightblue')
                    highlighted_patches[i].set_color('lightcoral')
                    fig.canvas.draw_idle()
                    break

        fig.canvas.mpl_connect('button_press_event', on_click)

        def confirm_selection():
            win.destroy()
            if current_intervals:
                analyze_callback(current_intervals[selected_interval_idx][0],
                                current_intervals[selected_interval_idx][1])

        analyze_button = ttk.Button(win, text="Analyze", command=confirm_selection)
        analyze_button.grid(row=2, column=0, columnspan=3, pady=10)

        # Inizializzazione
        update_lines()
        refresh_intervals(updating_from=None)
    
    
    def testNormalJump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], float) and self.jump_data["mass"]>0): raise ValueError("Invalid mass data")

        # --------------------------------------------
        label_frame_indicator_general_data = tk.LabelFrame(frame_indicator, text=f"General data", padx=10, pady=10)
        label_frame_indicator_general_data.pack(fill="both")

        label_frame_indicator_jump_power = tk.LabelFrame(frame_indicator, text=f"Jump power", padx=10, pady=10)
        label_frame_indicator_jump_power.pack(fill="both")

        label_indicator_mass = tk.Label(label_frame_indicator_general_data, text=f"Mass: {self.jump_data['mass']} Kg", font=("Arial", 12))
        label_indicator_mass.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_height = tk.Label(label_frame_indicator_general_data, text=f"Jump height: {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_time_AVG = tk.Label(label_frame_indicator_general_data, text=f"Jump time: {0.0} s", font=("Arial", 12))
        label_indicator_jump_time_AVG.pack(side=tk.LEFT, padx=20)

        label_indicator_first_touch = tk.Label(label_frame_indicator_general_data, text=f"First touch: {0.0} s", font=("Arial", 12))
        label_indicator_first_touch.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_power_AVG = tk.Label(label_frame_indicator_jump_power, text=f"Jump power AVG: {0.0} N", font=("Arial", 12))
        label_indicator_jump_power_AVG.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_power = {k: tk.Label(label_frame_indicator_jump_power, text=f"Jump power {k}: {0.0} N", font=("Arial", 12)) for k in info_load_cells}
        for k in info_load_cells:
            label_indicator_jump_power[k].pack(side=tk.LEFT, padx=20)
    
        # --------------------------------------------
        n = len(info_load_cells)
        cols = 1 if n == 1 else 2
        rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 2.5 * rows))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        axes = np.atleast_1d(axes).flatten()
        axes = {k: ax for ax, k in zip(axes, info_load_cells)}

        # Creating empty lines
        lines = {k: axes[k].plot([], [])[0] for k in info_load_cells}

        # Axis configuration
        for k in info_load_cells:
            axes[k].set_title(f"Load Cell {k}")
            axes[k].set_xlabel("Time (s)")
            axes[k].set_ylabel("Newton (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # --------------------------------------------
        
        
        def analyze_callback(interval, associated_interval):
            start, end = interval

            data = {}
            
            data["first_touch"]=self.raw_data["time"][end]
            data["jump_time"]=data["first_touch"]-self.raw_data["time"][start]

            data["jump_height"]=(1/2)*self.jump_data["acceleration_of_gravity"]*((data["jump_time"]/2)**2)

            data["jump_power"] = {}
            for i, k in enumerate(info_load_cells): 
                jump_time = self.raw_data["time"][associated_interval[i][1]]-self.raw_data["time"][associated_interval[i][0]]
                data["jump_power"][k] = (self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]*data["jump_height"])/(jump_time/2)
            
            data["jump_power_AVG"]=sum(data["jump_power"].values())/len(data["jump_power"])


            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
            smart_update_label(label_indicator_jump_height, f"Jump height: {round(data["jump_height"]*100,2)} cm", "black")

            smart_update_label(label_indicator_jump_time_AVG, f"Jump time: {round(data["jump_time"], 5)} s", "black")

            smart_update_label(label_indicator_first_touch, f"First touch: {round(data["first_touch"], 5)} s", "black")

            for k in info_load_cells: smart_update_label(label_indicator_jump_power[k], f"Jump power {k}: {round(data["jump_power"][k], 3)} N", "black")
            smart_update_label(label_indicator_jump_power_AVG, f"Jump power AVG: {round(data["jump_power_AVG"], 3)} N", "black")



        def func1():
            for i, k in enumerate(info_load_cells):
                lines[k].set_data(self.raw_data["time"], self.raw_data["lc"][i])
                axes[k].relim()
                axes[k].autoscale_view()
            canvas.draw()


        def func2():
            if not (self.raw_data and self.raw_data["size"]>0): return 
            clear_frame(frame_control_test)
            analyze_btn = ttk.Button(frame_control_test, text="Analyze data", command=lambda: self.launchAnalysis(self.raw_data["time"], self.raw_data["lc"], info_load_cells, 0, 13, analyze_callback))
            analyze_btn.pack(padx=20, pady=20)
            automatic_scaling()           

        return 50000, func1, (), func2, ()
    

    def testCalculateMass(self):
        self.jump_data["mass"] = 0.0

        # --------------------------------------------
        label_indicator_number_samples = tk.Label(frame_indicator, text=f"Number of samples collected: {0}", font=("Arial", 12))
        label_indicator_number_samples.grid(row=0, column=0)
        label_indicator_mass = tk.Label(frame_indicator, text=f"Mass: {self.jump_data['mass']} Kg", font=("Arial", 12))
        label_indicator_mass.grid(row=1, column=0, pady=10)
        # --------------------------------------------
        fig, axis = plt.subplots(1, 1, figsize=(5, 2))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        # Creating empty lines
        line = axis.plot([], [])[0]

        # Axis configuration
        axis.set_title(f"Mass chart")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("Newton (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # --------------------------------------------

        def func1():
            smart_update_label(label_indicator_number_samples, f"Number of samples collected: {self.raw_data["size"]}", "black")


        def func2():
            line.set_data(self.raw_data["time"], np.sum(self.raw_data["lc"], axis=0))
            axis.relim()
            axis.autoscale_view()
            canvas.draw()

            if self.raw_data["size"]==0: return 

            self.jump_data["mass"] = np.mean(np.sum(self.raw_data["lc"], axis=0))/self.jump_data["acceleration_of_gravity"]
            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")

     
        return 50000, func1, (), func2, ()


    def startRepresentation(self, test):
        smart_update_label(label_status_acquisition, "Initialization...", "black")
        self.stopRepresentation()
        data_acquirer.stopAcquisition()
        data_dq.clear()

        clear_frame(frame_control_test)
        clear_frame(frame_indicator)
        clear_frame(frame_chart)
        try:
            max_num_samples, func1, args1, func2, args2 = test()
            automatic_scaling()

            self.initializationRawData(max_num_samples)
            data_acquirer.startAcquisition()
            self.stop_representation=False
            self.request_stop_representation=False
            func = partial(self.run, func1, args1, func2, args2)
            self.after_id = root.after(self.time_sleep, func)
        except ValueError as e: 
            self.analysisError()
            show_warning("Attention!", e)

        

    def requestStopRepresentation(self):
        data_acquirer.stopAcquisition()
        self.request_stop_representation=True

    def stopRepresentation(self):
        try: root.after_cancel(self.after_id)
        except: pass
        self.stop_representation=True
        self.requestStopRepresentation()

    
    def addData(self, data):
        if self.raw_data and data:
            data_lc = data["response"]["lc"]
            if None not in list(data_lc["values"].values()):
                if self.raw_data["size"]<self.raw_data["max_size"]:
                    if self.raw_data["size"]==0:  self.raw_data["initial_time"] = data_lc["time"]
                    self.raw_data["unsliced_time"][self.raw_data["size"]] = data_lc["time"]-self.raw_data["initial_time"]
                    for i, k in enumerate(info_load_cells): self.raw_data["unsliced_lc"][i, self.raw_data["size"]] = data_lc["values"][k]
                    self.raw_data["size"] += 1
                else:
                    show_warning("Attention!", "You have reached the maximum number of saveable champions.")
                    self.stopRepresentation()






class DataAcquirer(threading.Thread):
    def __init__(self):
        super().__init__()

        self.acquisition_activity=True
        self.acquisition=False
        self.acquiring=threading.Lock()

        self.time_sleep_acquiring=0.0
        self.time_sleep_not_acquiring=0.3

        self._stopAcquisition()
    
    def run(self):
        while self.acquisition_activity:
            with self.acquiring:
                if self.acquisition:
                    try: data_dq.append(arduino.write_and_read({"command":"get_data"}))
                    except Exception as e: 
                        #print(e)
                        data_dq.append(None)
                        self._stopAcquisition()

            if self.acquisition: time.sleep(self.time_sleep_acquiring)
            else: time.sleep(self.time_sleep_not_acquiring)


    def _startAcquisition(self):
        self.acquisition=True

    def startAcquisition(self):
        with self.acquiring:
            self._startAcquisition()

    def _stopAcquisition(self):
        self.acquisition=False

    def stopAcquisition(self):
        with self.acquiring: 
            self._stopAcquisition()

    def stopAcquisitionActivity(self):
        self.acquisition_activity=False
        self._stopAcquisition()
    









def automatic_scaling():
    # Calculate and set the minimum size automatically
    root.update_idletasks()
    root.minsize(root.winfo_width(), root.winfo_height())
    

def get_connected_USB_devices():
    global list_info_device

    list_device = []
    list_valid_device = []
    for device in list_ports.comports(): 
        if device.vid is not None and device.pid is not None and device.device: list_valid_device.append(device)

    if list_valid_device:
        for i, device in enumerate(list_valid_device):
            info_device = {"port": device.device}
            list_device.append(info_device)
    else:
        #print("No valid USB device found.")
        pass

    list_info_device = [{"port": "prova"}]
    #list_info_device = list_device

def open_device_connection(info_device, timeout):
    global arduino
    close_device_connection()
    smart_update_label(label_status_connection, "Connecting...", "orange")
    try: 
        arduino = ArduinoEmulate(info_device, timeout) 
        arduino.open_connection()
        if check_device_connection():
            get_info_load_cells()
            command_button_scale_tare()
        else:
            close_device_connection()
            func_Focus_combobox_menu_device(None)
    except:
        close_device_connection()
        func_Focus_combobox_menu_device(None)

def clear_frame(frame):
    #print(f"CLEAR: {frame}")
    for widget in frame.winfo_children(): widget.destroy()
    automatic_scaling()
    

def close_device_connection():
    smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
    smart_update_label(label_status_connection, "Disconnecting...", "black")
    smart_update_label(label_scale_tare, "Not calibrated!", "red")
    data_rappresentor.stopRepresentation()
    if arduino is not None: arduino.close_connection()
    check_device_connection()

def func_Focus_combobox_menu_device(event):
    get_connected_USB_devices()
    if list_info_device: combobox_menu_device['values'] = [info_device["port"] for info_device in list_info_device]
    else:
        combobox_menu_device['values'] = []
        combobox_menu_device.set('')  # Removes selected text
    combobox_menu_device.update_idletasks() 
    
def func_ComboboxSelected_combobox_menu_device(event):
    selected = stringvar_menu_device.get()
    for info_device in list_info_device:
        if info_device["port"]==selected:
            info_device["baudrate"] = 460800
            open_device_connection(info_device, 2)
            break
        
def check_device_connection():
    if arduino is not None and arduino.check_connection(): 
        smart_update_label(label_status_connection, "Connected", "green")
        return True

    smart_update_label(label_status_connection, "Not Connected", "red")
    return False

def command_button_test(test):
    if check_device_connection() and info_load_cells: data_rappresentor.startRepresentation(test)
    else: 
        close_device_connection()
        func_Focus_combobox_menu_device(None)

def command_button_stop_test():
    data_rappresentor.requestStopRepresentation()

def func_WM_DELETE_WINDOW():
    data_rappresentor.stopRepresentation()
    data_acquirer.stopAcquisitionActivity()
    data_acquirer.join()
    close_device_connection()
    root.quit()
    root.destroy()

def command_button_scale_tare():
    smart_update_label(label_scale_tare, "Calibration...", "black")

    if check_device_connection():
        try:
            result = arduino.write_and_read({"command":"scale_tare"})["response"]["lc"]
            result_string = ""
            for i, k in enumerate(info_load_cells):
                result_string += f"{k}: {result[k]}"
                if i < len(info_load_cells)-1: result_string += " | "
            smart_update_label(label_scale_tare, result_string, "black")
        except Exception as e:
            #print(e)
            smart_update_label(label_scale_tare, "Not calibrated!", "red")
    else: 
        close_device_connection()
        func_Focus_combobox_menu_device(None)

                
def smart_update_label(label, text=None, fg=None):
    current_text = label.cget("text")
    current_fg = label.cget("fg")

    if (text is not None and text != current_text) or (fg is not None and fg != current_fg):
        label.config(text=text if text is not None else current_text, fg=fg if fg is not None else current_fg)
        label.update_idletasks()


def show_warning(title, message):
    root = tk.Tk()
    root.withdraw()
    messagebox.showwarning(title, message)


def get_info_load_cells():
    global info_load_cells
    info_load_cells=[]

    if check_device_connection():
        try:
            result = arduino.write_and_read({"command":"get_info"})
            info_load_cells = result["lc"]
        except:
            close_device_connection()
            func_Focus_combobox_menu_device(None)
    else:
        close_device_connection()
        func_Focus_combobox_menu_device(None)





# ----------------------

stringa_scritta = ""
class ArduinoEmulate(USBIODevice):
    def __init__(self, info_device={}, device_timeout=5):
        super().__init__(info_device, device_timeout)
    
    def _reset_buffer(self):
        pass

    def _check_connection(self):
        return True
    
    def _close_connection(self):
        pass
    
    def _open_connection(self):
        return None

    
    def _read(self):
        #time.sleep(1)
        #print("read: "+stringa_scritta)
        import random
        if "get_data" in stringa_scritta:
            return {"code":"OK", "response":{"lc":{"values":{"LX":random.randint(1, 10), "RX":random.randint(1, 10), "UP":random.randint(1, 10), "DW":random.randint(1, 10)}, "time": time.time()}}}
        elif "scale_tare" in stringa_scritta:
            return {"code":"OK", "response":{"lc":{"LX":"OK", "RX":"OK", "UP":"OK", "DW":"OK"}}}
        elif "is_alive" in stringa_scritta:
            return {"code":"OK"}
        elif "get_info" in stringa_scritta:
            return {"code":"OK", "lc": ["LX", "RX", "UP", "DW"]}
    
    def _write(self, data):
        global stringa_scritta
        stringa_scritta = f"?{json.dumps(data)}!"
        #print("write: "+stringa_scritta)






# ------------------------------------

info_load_cells = []
list_info_device = []

root = tk.Tk()
arduino = None
data_dq = deque()
data_acquirer = DataAcquirer()
data_rappresentor = DataRappresentor()


# Creation of main window
root.title("Jumping platform")
#root.geometry("1000x1000")

root.protocol("WM_DELETE_WINDOW", func_WM_DELETE_WINDOW)

# Device Selection and Status
frame_menu_device = tk.Frame(root)
frame_menu_device.pack(pady=10)

tk.Label(frame_menu_device, text="Select Device:").pack(side=tk.LEFT)
stringvar_menu_device = tk.StringVar()
combobox_menu_device = ttk.Combobox(frame_menu_device, textvariable=stringvar_menu_device, state="readonly")
combobox_menu_device.pack(side=tk.LEFT, padx=10)
combobox_menu_device.bind("<<ComboboxSelected>>", func_ComboboxSelected_combobox_menu_device)
#combobox_menu_device.bind("<FocusIn>", func_Focus_combobox_menu_device)
combobox_menu_device.bind("<FocusOut>", func_Focus_combobox_menu_device)
func_Focus_combobox_menu_device(None)

label_status_connection = tk.Label(frame_menu_device, text="Not Connected", fg="red")
label_status_connection.pack(side=tk.LEFT, padx=10)

# Scale tare
frame_scale_tare = tk.Frame(root)
frame_scale_tare.pack(pady=10)
button_scale_tare = tk.Button(frame_scale_tare, text="Tare", command=command_button_scale_tare)
button_scale_tare.pack(side=tk.LEFT, padx=10)
label_scale_tare = tk.Label(frame_scale_tare, text=f"Not calibrated!", font=("Arial", 12), fg="red")
label_scale_tare.pack(side=tk.LEFT, padx=20)

# Acquisition section
frame_acquisition = tk.Frame(root)
frame_acquisition.pack(pady=10)

button_test_normal_jump = tk.Button(frame_acquisition, text="Normal jump", command=lambda: command_button_test(data_rappresentor.testNormalJump))
button_test_normal_jump.pack(side=tk.LEFT, padx=20)
button_test_calculate_mass = tk.Button(frame_acquisition, text="Calculate mass", command=lambda: command_button_test(data_rappresentor.testCalculateMass))
button_test_calculate_mass.pack(side=tk.LEFT, padx=20)
button_stop_test = tk.Button(frame_acquisition, text="Stop test", command=command_button_stop_test)
button_stop_test.pack(side=tk.LEFT, padx=20)
button_force_stop = tk.Button(frame_acquisition, text="Force stop", command=data_rappresentor.stopRepresentation)
button_force_stop.pack(side=tk.LEFT, padx=20)

frame_acquisition_status = tk.Frame(root)
frame_acquisition_status.pack(pady=10)
label_status_acquisition = tk.Label(frame_acquisition_status, text=f"Acquisition status: Stop", font=("Arial", 15))
label_status_acquisition.pack(side=tk.LEFT, padx=20)

# Control test section
frame_control_test = tk.Frame(root)
frame_control_test.pack(pady=10)

# Indicators section
frame_indicator = tk.Frame(root)
frame_indicator.pack(pady=10)

# Chart section
frame_chart = tk.Frame(root)
frame_chart.pack(fill=tk.BOTH, expand=True)

# Starting data acquirer
data_acquirer.start()


automatic_scaling()
# Starting root mainloop
root.mainloop()