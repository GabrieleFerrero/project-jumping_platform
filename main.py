from enum import Enum, auto
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
from functools import partial
from scipy.signal import savgol_filter
from matplotlib.patches import Rectangle
import math
from scipy.integrate import cumtrapz



class TimeoutException(Exception):
    """Custom exception for timeout"""
    pass

class ArduinoCommunicationException(Exception):
    """Custom exception for Arduino communication"""
    pass

class ResourceBusyException(Exception):
    pass

class StateCommunicationDevice(Enum):
    AVAILABLE_CONNECTED = auto()
    AVAILABLE_NOT_CONNECTED = auto()
    NOT_AVAILABLE = auto()

class StateMultipleReadings(Enum):
    START = auto()
    STOP = auto()
    CLOSE = auto()
    NEXT = auto()

class StateDataAcquirer(Enum):
    START = auto()
    STOP = auto()
    CLOSE = auto()
    GET = auto()
    GET_LAST = auto()
    FINISH = auto()
    START_REQUEST = auto()
    END_REQUEST = auto()


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

    @abstractmethod
    def _write_and_read(self, data):
        pass

    def set_info_device(self, info_device):
        self.info_device=info_device    

    def reset_buffer(self, timeout=None, silent_time=1.0):
        timeout = self.device_timeout if timeout is None else timeout

        if silent_time>=timeout: raise Exception("Problem with timing")

        self.function_execution_with_timeout(timeout, self._reset_buffer, silent_time)

    def check_connection(self, timeout=None):
        if self.lock_IO.acquire(blocking=False):
            try:
                try: return self.function_execution_with_timeout(self.device_timeout if timeout is None else timeout, self._check_connection)
                except: return False
            finally: self.lock_IO.release()
        else: raise ResourceBusyException("IO resource busy")
    
    def close_connection(self, timeout=None):
        try: self.function_execution_with_timeout(self.device_timeout if timeout is None else timeout, self._close_connection)
        except: pass
        self.device_connection = None

    def open_connection(self, timeout=None):
        self.close_connection(timeout)
        self.device_connection = self.function_execution_with_timeout(self.device_timeout if timeout is None else timeout, self._open_connection)
        self.reset_buffer(timeout)

    def read(self, timeout=None):
        if self.lock_IO.acquire(blocking=False):
            try:
                return self.function_execution_with_timeout(self.device_timeout if timeout is None else timeout, self._read)
            finally: self.lock_IO.release()
        else: raise ResourceBusyException("IO resource busy")

    def write(self, data, timeout=None):
        if self.lock_IO.acquire(blocking=False):
            try:
                self.function_execution_with_timeout(self.device_timeout if timeout is None else timeout, self._write, data)
            finally: self.lock_IO.release()
        else: raise ResourceBusyException("IO resource busy")

    def write_and_read(self, data, timeout=None):
        if self.lock_IO.acquire(blocking=False):
            try:
                return self.function_execution_with_timeout(self.device_timeout*2 if timeout is None else timeout, self._write_and_read, data)
            finally: self.lock_IO.release()
        else: raise ResourceBusyException("IO resource busy")

    def multiple_readings(self, timeout=None, silent_time_reset_buffer=None):
        if self.lock_IO.acquire(blocking=False):
            try:
                while True:
                    state = yield
                    if state == StateMultipleReadings.START:
                        msg = yield  
                        self.write(msg, timeout)
                    elif state == StateMultipleReadings.STOP:
                        msg = yield  
                        self.write(msg, timeout)
                    elif state == StateMultipleReadings.CLOSE:
                        try: self.reset_buffer(silent_time=silent_time_reset_buffer)
                        except TimeoutException: break
                        return
                    elif state == StateMultipleReadings.NEXT:
                        result = self.read(timeout)
                        yield result
            finally: self.lock_IO.release()
        else: raise ResourceBusyException("IO resource busy")


class USBIODevice(IODevice):
    pass
    

class Arduino(USBIODevice):
    def __init__(self, info_device={}, device_timeout=5):
        super().__init__(info_device, device_timeout)

        self.pause_reset_buffer = 0.0001

    def _write_and_read(self, data):
        self._reset_buffer()
        self._write(data)
        result = self._read()
        if not("command" in data and "command" in result and data["command"]==result["command"]): raise ArduinoCommunicationException("Response received different from request sent")
        return result
    
    def _reset_buffer(self, silent_time=0):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        to = self.device_connection.timeout

        self.device_connection.timeout = 0
        last_data_time = time.time()
        while True:
            data = self.device_connection.read(self.device_connection.in_waiting or 1)
            if data: last_data_time = time.time()
            else:
                if (time.time() - last_data_time) > silent_time: break 
            time.sleep(self.pause_reset_buffer)
        self.device_connection.reset_input_buffer()
        self.device_connection.reset_output_buffer()

        self.device_connection.timeout = to

    def _check_connection(self):
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        
        if self.device_connection.is_open:
            result = self._write_and_read({"command":"is_alive"})
            if result: return True
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
            #print(f"Read: {response}")
            result = json.loads(response)
            if "code" in result and result["code"] == "ERROR": raise ArduinoCommunicationException(f"Bad json: {result}")
            return result
        else: return json.loads("{}")
    
    def _write(self, data):
        #print(f"Write: {data}")
        if not (isinstance(self.device_connection, serial.Serial)):
            raise ValueError("Invalid connection")
        if not (isinstance(data, dict)):
            raise ValueError("Incorrect parameters")
        
        self.device_connection.write(f"?{json.dumps(data)}!".encode())
        #self.device_connection.flush() # Aspetto che siano mandati tutti sulla seriale


class RawData(dict):
    def __init__(self, max_num_samples, num_lc):
        super().__init__()
        self["initial_time"] = 0.0
        self["unsliced_time"] = np.zeros(max_num_samples, dtype=float)
        self["unsliced_lc"] = np.zeros((num_lc, max_num_samples), dtype=float)
        self["size"] = 0
        self["max_size"] = max_num_samples

    def __getitem__(self, key):
        if key == 'time':
            return self["unsliced_time"][:self["size"]]
        elif key == 'lc':
            return self["unsliced_lc"][:, :self["size"]]
        return super().__getitem__(key)

    def append(self, time_value, lc_values):
        i = self["size"]

        if i >= self["max_size"]: raise IndexError("RawData full: max size reached")

        if i==0: self["initial_time"] = time_value

        self["unsliced_time"][i] = time_value-self["initial_time"]
        self["unsliced_lc"][:, i] = lc_values
        self["size"] += 1



class DataRappresentor():
    def __init__(self):

        self.after_id=None
        self.raw_data = None
        self.jump_data={
            "mass": 0.0,
            "acceleration_of_gravity": 9.81
        }
        self.stop_representation=True # force stop representation
        self.request_stop_representation=True
    
        self.function_recall_time=1 # ms (milliseconds)
        self.get_timeout = 3


    def run(self, func1, args1, func2, args2):
        if not self.stop_representation:
            try:
                data = data_queue.get(timeout=self.get_timeout)
                if data is not None:
                    if isinstance(data, StateDataAcquirer) and data==StateDataAcquirer.FINISH:
                        func1(*args1)
                        self.stopRepresentation()
                        smart_update_label(label_status_acquisition, "Analysis in progress...", "purple")
                        func2(*args2)
                        smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
                    else:
                        self.addData(data)
                        if not self.request_stop_representation:
                            func1(*args1)
                            smart_update_label(label_status_acquisition, "Acquisition in progress...", "green")
                        elif self.request_stop_representation:
                            smart_update_label(label_status_acquisition, "Saving the last samples...", "orange")

                else: self.analysisError()
            except queue.Empty as e: 
                show_warning("Error!", "Waiting too long to receive data")
                self.analysisError()
            except IndexError as e:
                show_warning("Attention!", "You have reached the maximum number of saveable champions.")
                func1(*args1)
                self.stopRepresentation()
                smart_update_label(label_status_acquisition, "Analysis in progress...", "purple")
                func2(*args2)
                smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
            except Exception as e:
                self.analysisError()


        if not self.stop_representation:
            func = partial(self.run, func1, args1, func2, args2)
            self.after_id = root.after(self.function_recall_time, func)


    def analysisError(self):
        smart_update_label(label_status_acquisition, "Error in analysis!", "red")
        self.stopRepresentation()
        if verify_the_possibility_of_communicating()==StateCommunicationDevice.AVAILABLE_NOT_CONNECTED:
            close_device_connection()
            func_Focus_combobox_menu_device(None)


    def initializationRawData(self, max_num_samples):
        self.raw_data = RawData(max_num_samples, len(info_load_cells))

        
    def analysisJumpPhase(self, analysis_result_calculation, analysis_callback, additional_params):

        def data_analysis():
            nonlocal results_analysis

            if selection_area["start"] is None or selection_area["end"] is None: return

            for artist in plotted_elements["data_analysis"]: artist.remove()
            plotted_elements["data_analysis"].clear()

            results_analysis = None

            try:  
                selected_time = self.raw_data["time"][selection_area["start"]:selection_area["end"]+1]
                selected_lc = self.raw_data["lc"][:, selection_area["start"]:selection_area["end"]+1]
                selected_force = np.sum(selected_lc, axis=0)
                
                if entries_params["use_filter"]["info"]["value"]:
                    selected_force_filtered = savgol_filter(selected_force, window_length=entries_params["savgol_filter_window_length"]["info"]["value"], polyorder=entries_params["savgol_filter_polyorder"]["info"]["value"])
                    plotted_elements["data_analysis"].append(ax.plot(selected_time, selected_force_filtered, label='Filtered force', color='gray')[0])
                    selected_force = selected_force_filtered

                  
                results_analysis, plot_el = analysis_result_calculation(selected_force, selected_time, selection_area["start"], selection_area["end"], ax, entries_params)
                plotted_elements["data_analysis"] += plot_el

            except Exception as e: show_warning("Error", f"Problem in the analyses: {e}")

            update_plot()


        def clear_plotted_analysis_elements():
            for artist in plotted_elements["on_motion"]: artist.remove()
            plotted_elements["on_motion"].clear()

            for artist in plotted_elements["on_release"]: artist.remove()
            plotted_elements["on_release"].clear()

            for artist in plotted_elements["data_analysis"]: artist.remove()
            plotted_elements["data_analysis"].clear()

            update_plot()


        def update_plot():
            update_legend()
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
            canvas.draw()


        def update_view_x(start_visible_time):
            total_duration = self.raw_data["time"][-1] - self.raw_data["time"][0]
            visible_duration = total_duration if total_duration < entries_params["window_duration"]["info"]["value"] else entries_params["window_duration"]["info"]["value"]
            scroll_x.config(to=self.raw_data["time"][-1]-entries_params["window_duration"]["info"]["value"])
            var_scroll_x.set(start_visible_time)
            ax.set_xlim(start_visible_time, start_visible_time+visible_duration)
            update_plot()


        def on_click(event):
            nonlocal mouse_start

            if event.inaxes != ax or toolbar.mode != '': return

            legend = ax.get_legend()
            # Ottieni il bounding box della legenda (in formato [x0, y0, width, height])
            if legend:
                bbox = legend.get_window_extent()
                # Controlla se il click è dentro il bounding box della legenda
                if bbox.contains(event.x, event.y): return  # Ignora l'evento di click


            selection_area["start"], selection_area["end"] = None, None

            clear_plotted_analysis_elements()
            mouse_start = event.xdata


        def on_motion(event):
            nonlocal mouse_start

            if mouse_start is not None: 
                for artist in plotted_elements["on_motion"]: artist.remove()
                plotted_elements["on_motion"].clear()
                canvas.draw_idle()

            if event.inaxes != ax and mouse_start is not None: mouse_start=None
            if event.inaxes != ax or mouse_start is None or toolbar.mode != '': return

            x0 = mouse_start
            x1 = event.xdata
            xmin, xmax = sorted([x0, x1])

            y_min, y_max = ax.get_ylim()
            rect = Rectangle((xmin, y_min),
                            xmax - xmin,
                            y_max - y_min,
                            color='gray', alpha=0.3)
            plotted_elements["on_motion"].append(ax.add_patch(rect))
            ax.set_ylim(y_min, y_max)

            canvas.draw_idle()


        def on_release(event):
            nonlocal mouse_start

            if mouse_start is not None: clear_plotted_analysis_elements()
            if event.inaxes != ax and mouse_start is not None: mouse_start=None
            if event.inaxes != ax or mouse_start is None or toolbar.mode != '': return

            x0 = mouse_start
            x1 = event.xdata
            xmin, xmax = sorted([x0, x1])

            mouse_start = None

            selection_area["start"], selection_area["end"] = find_window_indices(self.raw_data["time"], xmin, xmax)

            plotted_elements["on_release"].append(ax.axvline(xmin, color='black', linestyle='--'))
            plotted_elements["on_release"].append(ax.axvline(xmax, color='black', linestyle='--'))
            update_plot()
            apply_params(recalls_actions=False)
            data_analysis()


        def apply_params(recalls_actions=True):
            changed_params = {}
            for key, value in entries_params.items():
                var = value["var"]
                info_param = value["info"]

                if info_param["ptype"] not in changed_params: changed_params[info_param["ptype"]] = False
                try:
                    value = info_param["type"](var.get())
                    if info_param["cond"](value):
                        if value != info_param["value"]: changed_params[info_param["ptype"]] = True
                        info_param["value"] = value
                    else: show_warning("Error", f"Condition not met for {key}")
                except ValueError: show_warning("Error", f"Invalid input for {key}")

            if info_param["type"] == int or info_param["type"] == float: var.set(str(info_param["value"]))
            elif info_param["type"] == bool: var.set(info_param["value"])
            else: show_warning("Error", f"Parameter type not recognized: {info_param["type"]}")

            if recalls_actions:
                if "interface" in changed_params and changed_params["interface"]: update_view_x(0)
                if "processing" in changed_params and changed_params["processing"]: data_analysis()


        def add_param(info_param, n_row):

            ttk.Label(param_frame, text=info_param["label"]).grid(row=n_row, column=0, sticky='w', pady=5)

            if info_param["type"] == int or info_param["type"] == float:
                var = tk.StringVar(value=info_param["value"])
                ttk.Entry(param_frame, textvariable=var, width=10).grid(row=n_row, column=1, padx=5)

            elif info_param["type"] == bool:
                var = tk.BooleanVar(value=info_param["value"])
                tk.Checkbutton(param_frame, variable=var).grid(row=n_row, column=1, sticky='w', pady=5)
            else: 
                show_warning("Error", f"Parameter type not recognized: {info_param["type"]}")

            entries_params[info_param["key"]] = {"var":var, "info":info_param}


        def confirm_analysis():
            if selection_area["start"] is None or selection_area["end"] is None or results_analysis is None: return

            try:
                selected_time = self.raw_data["time"][selection_area["start"]:selection_area["end"]+1]
                selected_lc = self.raw_data["lc"][:, selection_area["start"]:selection_area["end"]+1]
                selected_force = np.sum(selected_lc, axis=0)

                if entries_params["use_filter"]["info"]["value"]:
                    selected_force_filtered = savgol_filter(selected_force, window_length=entries_params["savgol_filter_window_length"]["info"]["value"], polyorder=entries_params["savgol_filter_polyorder"]["info"]["value"])
                    selected_force = selected_force_filtered
                
                analysis_callback(selected_force, selected_time, selection_area["start"], selection_area["end"], results_analysis, entries_params)
            except Exception as e: show_warning("Error", f"Problem in the analyses: {e}")

            #win.destroy()

        
        def find_window_indices(array, start, end):
            """
            Finds the indices (start_index, end_index) for the window 
            [start, end] within a sorted NumPy array, properly handling bounds.
            """
            start_index = np.searchsorted(array, start, side="left")
            end_index = np.searchsorted(array, end, side="right") - 1

            # Protection from extreme cases
            start_index = min(start_index, len(array) - 1)
            end_index = max(end_index, 0)

            return start_index, end_index
        

        def update_legend():
            handles, labels = ax.get_legend_handles_labels()
            legend = ax.legend(handles, labels)

            for handle, label in zip(legend.legendHandles, labels):
                for artist in ax.get_children():
                    if hasattr(artist, "get_label") and artist.get_label() == label:
                        if not hasattr(artist, "_original_alpha"):
                            artist._original_alpha = artist.get_alpha() or 1.0
                        is_visible = artist.get_alpha() != 0.0
                        handle.set_alpha(1.0 if is_visible else 0.2)
                        handle._linked_artist = artist
                        handle.set_picker(True)
                        break

            canvas.draw_idle()


        def on_pick(event):
            legend_handle = event.artist
            if hasattr(legend_handle, "_linked_artist") and hasattr(legend_handle._linked_artist, "_original_alpha"):
                artist = legend_handle._linked_artist
                is_visible = artist.get_alpha() != 0.0
                artist.set_alpha(0.0 if is_visible else artist._original_alpha)
                legend_handle.set_alpha(0.2 if is_visible else 1.0)
                canvas.draw_idle()

        
        def clear_analysis():
            nonlocal results_analysis
            clear_plotted_analysis_elements()
            selection_area["start"], selection_area["end"] = None, None
            results_analysis = None



        # -------------------------------------------

        selection_area = {"start":None, "end":None}
        params = [
            {"label":"Savgol Window Length","type":int,"value":21,"key":"savgol_filter_window_length","ptype":"processing","cond":lambda x: 1 <= x <= np.iinfo(np.uint32).max and x % 2 == 1},
            {"label":"Savgol Polyorder","type":int,"value":3,"key":"savgol_filter_polyorder","ptype":"processing","cond":lambda x: 0 <= x <= np.iinfo(np.uint32).max},
            {"label":"Enable filter","type":bool,"value":True,"key":"use_filter","ptype":"processing","cond":lambda x: True},
            {"label":"Window duration (s)","type":float,"value":2,"key":"window_duration","ptype":"interface","cond":lambda x: 0 < x}
        ]
        entries_params = {}
        results_analysis = None
        mouse_start = None
        plotted_elements = {
            "on_motion": [],
            "on_release": [],
            "data_analysis": [],
            "data_input": []
        }

        win = tk.Toplevel()
        open_windows["win_analysis"].append(win)

        win.title("Plot Viewer")
        win.geometry("1000x750")
        win.grid_columnconfigure(1, weight=1)
        win.grid_rowconfigure(0, weight=1)

        plot_frame = ttk.Frame(win)
        plot_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")
        fig, ax = plt.subplots(figsize=(10, 4))
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        toolbar = NavigationToolbar2Tk(canvas, plot_frame)
        toolbar.update()
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True)
        ax.set_autoscaley_on(True)
        ax.set_autoscalex_on(False)
        plt.close(fig)

        for i, vec in enumerate(self.raw_data["lc"]): plotted_elements["data_input"].append(ax.plot(self.raw_data["time"], vec, label=f"Force {info_load_cells[i]}", alpha=0.3)[0])
        plotted_elements["data_input"].append(ax.plot(self.raw_data["time"], np.sum(self.raw_data["lc"], axis=0), label="Total force", linewidth=2, color='black', alpha=0.6)[0])
        ax.grid(True)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("GRF (N)")

        fig.canvas.mpl_connect('button_press_event', on_click)
        fig.canvas.mpl_connect('motion_notify_event', on_motion)
        fig.canvas.mpl_connect('button_release_event', on_release)
        fig.canvas.mpl_connect('pick_event', on_pick)

        # ---------------- Scrollbar ----------------
        var_scroll_x = tk.DoubleVar()
        scroll_x = ttk.Scale(plot_frame, from_=0, orient='horizontal', length=800, variable=var_scroll_x, command=lambda val: update_view_x(float(val)))
        scroll_x.pack(fill=tk.X, expand=False)

        # ---------------- Parametri ----------------
        param_frame = ttk.Frame(win, padding=10)
        param_frame.grid(row=0, column=0, sticky="ns")

        ttk.Label(param_frame, text="Params management:").grid(row=0, column=0, columnspan=2, sticky='ew', pady=10)

        num_row = 1
        for info_param in params:
            add_param(info_param, num_row)
            num_row+=1
        for info_param in additional_params:
            add_param(info_param, num_row)
            num_row+=1
            
        ttk.Button(param_frame, text="Apply params", command=apply_params).grid(row=num_row, column=0, columnspan=2, pady=5, sticky="ew")
        ttk.Button(param_frame, text="Clear analysis", command=clear_analysis).grid(row=num_row+1, column=0, columnspan=2, pady=5, sticky="ew")

        # ---------------- Bottone finale ----------------    
        ttk.Button(win, text="Calculating values", command=confirm_analysis).grid(row=2, column=1, pady=20)

        # ------------ Default initialization ----------
        update_view_x(0)


    def testDepthJump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], float) and self.jump_data["mass"]>0): raise ValueError("Invalid mass data")

        # --------------------------------------------


        def analysis_result_calculation(force, time, start_idx, end_idx, ax, entries_params):
            
            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]
            force_net = force - weight
            acceleration = force_net / self.jump_data["mass"]
            
            start_amortization = np.where(force >= weight*(entries_params["start_threshold"]["info"]["value"]/100))[0][0]

            v_impact = - math.sqrt(2*self.jump_data["acceleration_of_gravity"]*entries_params["jump_height"]["info"]["value"])
            velocity = cumtrapz(acceleration[start_amortization:], time[start_amortization:], initial=0) + v_impact
            #ax.plot(time[start_amortization:], velocity*100, color="red")

            start_push_off =  start_amortization + np.where(np.abs(velocity) <= entries_params["zero_velocity_threshold"]["info"]["value"])[0][0]
            takeoff = start_push_off + np.where(force[start_push_off:] <= weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))[0][0]
            landing = takeoff + np.where(force[takeoff:] > weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))[0][0]
            peak_amortization_force = start_amortization + np.argmax(force[start_amortization:start_push_off])
            peak_landing_force = landing + np.argmax(force[landing:])
            end_push = start_push_off + np.where(force[start_push_off:] <= weight)[0][0]

            # Save result

            result = {
                "start_amortization": start_amortization,
                "start_push_off": start_push_off,
                "peak_amortization_force": peak_amortization_force,
                "takeoff": takeoff,
                "landing": landing,
                "peak_landing_force": peak_landing_force,
                "end_push": end_push
            }

            # ==== 3. Plot ====

            plotted_elements = []
 
            events = {
                "Start amortization": start_amortization,
                "Start push-off": start_push_off,
                "Peak amortization force": peak_amortization_force,
                "Takeoff": takeoff,
                "Landing": landing,
                "Peak Landing Force": peak_landing_force
            }
 
            event_color = ['red', 'orange', 'blue', 'green', 'brown', 'purple']
 
            for (name, idx), color in zip(events.items(), event_color):
                plotted_elements.append(ax.scatter(time[idx], force[idx], marker='o', color=color))
                plotted_elements.append(ax.text(time[idx], force[idx] + 30, name, color=color, fontsize=9, ha='center'))
 
            # 2. Aree colorate per le fasi
            plotted_elements.append(ax.axvspan(time[0], time[start_amortization], color='yellow', alpha=0.2, label='Waiting for jump'))
            plotted_elements.append(ax.axvspan(time[start_amortization], time[start_push_off], color='orange', alpha=0.2, label='Amortization Phase'))
            plotted_elements.append(ax.axvspan(time[start_push_off], time[takeoff], color='green', alpha=0.2, label='Push-off Phase'))
            plotted_elements.append(ax.axvspan(time[takeoff], time[landing], color='blue', alpha=0.2, label='Flight Phase'))
            plotted_elements.append(ax.axvspan(time[landing], time[-1], color='purple', alpha=0.2, label='Landing Phase'))

            return result, plotted_elements
        

        def analysis_callback(force, time, start_idx, end_idx, results_analysis, entries_params):

            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]
            force_net = force - weight
            impulse = np.trapz(force_net[results_analysis["start_push_off"]:results_analysis["takeoff"]+1], time[results_analysis["start_push_off"]:results_analysis["takeoff"]+1])
            v0 = impulse / self.jump_data["mass"]
            h_force = (v0 ** 2) / (2 * self.jump_data["acceleration_of_gravity"])
            
            time_contact = time[results_analysis["takeoff"]] - time[results_analysis["start_amortization"]]
            time_flight = time[results_analysis["landing"]]-time[results_analysis["takeoff"]]

            h_time_flight = 0.5*self.jump_data["acceleration_of_gravity"]*((time_flight/2)**2)
            h_avg = (h_force+h_time_flight)/2



            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
            smart_update_label(label_indicator_jump_height_force, f"Jump height (force): {round(h_force*100,2)} cm", "black")
            smart_update_label(label_indicator_jump_height_time_flight, f"Jump height (time flight): {round(h_time_flight*100,2)} cm", "black")
            smart_update_label(label_indicator_jump_height_avg, f"Jump height (avg): {round(h_avg*100,2)} cm", "black")
            smart_update_label(label_indicator_average_reaction_force, f"Impulse: {round(impulse,2)} m*kg/s", "black")
            smart_update_label(label_indicator_jumping_speed, f"Jumping speed: {round(v0,2)} m/s", "black")
            smart_update_label(label_indicator_flight_time, f"Flight time: {round(time_flight,5)} s", "black")
            smart_update_label(label_indicator_time_contact, f"Contact time: {round(time_contact,5)} s", "black")
 
            smart_update_label(label_indicator_start_amortization, f"Start amortization: {round(self.raw_data["time"][results_analysis["start_amortization"]],5)} s", "black")
            smart_update_label(label_indicator_start_push_off, f"Start push-off: {round(self.raw_data["time"][results_analysis["start_push_off"]],5)} s", "black")
            smart_update_label(label_indicator_peak_amortization_force, f"Peak amortization force at {round(self.raw_data["time"][results_analysis["peak_amortization_force"]],5)} s with value {round(force[results_analysis["peak_amortization_force"]],2)} N", "black")
            smart_update_label(label_indicator_takeoff, f"Takeoff: {round(self.raw_data["time"][results_analysis["takeoff"]],5)} s", "black")
            smart_update_label(label_indicator_landing, f"Landing: {round(self.raw_data["time"][results_analysis["landing"]],5)} s", "black")
            smart_update_label(label_indicator_peak_landing_force, f"Peak landing force at {round(self.raw_data["time"][results_analysis["peak_landing_force"]],5)} s with value {round(force[results_analysis["peak_landing_force"]],2)} N", "black")




        def func1():
            pass


        def func2():
            if not (self.raw_data and self.raw_data["size"]>0): return 

            for i, k in enumerate(info_load_cells):
                lines[k].set_data(self.raw_data["time"], self.raw_data["lc"][i])
                axes[k].relim()
                axes[k].autoscale_view()
            canvas.draw()

            clear_frame(frame_control_test)

            params = [
                {"label":"Start Threshold (%)","type":float,"value":2,"key":"start_threshold","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0},
                {"label":"Takeoff Threshold (%)","type":float,"value":5,"key":"takeoff_threshold","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0},
                {"label":"Zero Velocity Threshold (m/s)","type":float,"value":0.1,"key":"zero_velocity_threshold","ptype":"processing","cond":lambda x: 0.0 <= x},
                {"label":"Jump height (m)","type":float,"value":0.30,"key":"jump_height","ptype":"processing","cond":lambda x: 0.0 <= x}
            ]
            analyze_btn = ttk.Button(frame_control_test, text="Analyze data", command=lambda: self.analysisJumpPhase(analysis_result_calculation, analysis_callback, params))
            analyze_btn.pack(padx=20, pady=20)
            automatic_scaling()  


        # --------------------------------------------
        label_frame_indicator_general_data = tk.LabelFrame(frame_indicator, text=f"General data", padx=10, pady=10)
        label_frame_indicator_general_data.pack(fill="both")
 
        label_frame_indicator_jump_event = tk.LabelFrame(frame_indicator, text=f"Jump event", padx=10, pady=10)
        label_frame_indicator_jump_event.pack(fill="both")

 
        label_indicator_mass = tk.Label(label_frame_indicator_general_data, text=f"Mass: {round(self.jump_data["mass"],2)} Kg", font=("Arial", 12))
        label_indicator_mass.grid(row=0, column=0, sticky='w', pady=5, padx=10)

        label_indicator_jump_height_force = tk.Label(label_frame_indicator_general_data, text=f"Jump height (force): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_force.grid(row=0, column=1, sticky='w', pady=5, padx=10)

        label_indicator_jump_height_time_flight = tk.Label(label_frame_indicator_general_data, text=f"Jump height (time flight): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_time_flight.grid(row=0, column=2, sticky='w', pady=5, padx=10)
 
        label_indicator_jump_height_avg = tk.Label(label_frame_indicator_general_data, text=f"Jump height (avg): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_avg.grid(row=0, column=3, sticky='w', pady=5, padx=10)
 
        label_indicator_average_reaction_force = tk.Label(label_frame_indicator_general_data, text=f"Impulse: {0.0} m*Kg/s", font=("Arial", 12))
        label_indicator_average_reaction_force.grid(row=1, column=0, sticky='w', pady=5, padx=10)
 
        label_indicator_jumping_speed = tk.Label(label_frame_indicator_general_data, text=f"Jumping speed: {0.0} m/s", font=("Arial", 12))
        label_indicator_jumping_speed.grid(row=1, column=1, sticky='w', pady=5, padx=10)
 
        label_indicator_flight_time = tk.Label(label_frame_indicator_general_data, text=f"Flight time: {0.0} s", font=("Arial", 12))
        label_indicator_flight_time.grid(row=1, column=2, sticky='w', pady=5, padx=10)
 
        label_indicator_time_contact = tk.Label(label_frame_indicator_general_data, text=f"Contact time: {0.0} s", font=("Arial", 12))
        label_indicator_time_contact.grid(row=1, column=3, sticky='w', pady=5, padx=10)

 
        label_indicator_start_amortization = tk.Label(label_frame_indicator_jump_event, text=f"Start amortization: {0.0} s", font=("Arial", 12))
        label_indicator_start_amortization.grid(row=0, column=0, sticky='w', pady=5, padx=10)
 
        label_indicator_start_push_off = tk.Label(label_frame_indicator_jump_event, text=f"Start push-off: {0.0} s", font=("Arial", 12))
        label_indicator_start_push_off.grid(row=0, column=1, sticky='w', pady=5, padx=10)
 
        label_indicator_takeoff = tk.Label(label_frame_indicator_jump_event, text=f"Takeoff: {0.0} s", font=("Arial", 12))
        label_indicator_takeoff.grid(row=0, column=2, sticky='w', pady=5, padx=10)
 
        label_indicator_landing = tk.Label(label_frame_indicator_jump_event, text=f"Landing: {0.0} s", font=("Arial", 12))
        label_indicator_landing.grid(row=0, column=3, sticky='w', pady=5, padx=10)
 
        label_indicator_peak_amortization_force = tk.Label(label_frame_indicator_jump_event, text=f"Peak amortization force at {0.0} s with value {0.0} N", font=("Arial", 12))
        label_indicator_peak_amortization_force.grid(row=1, column=0, sticky='w', pady=5, padx=10)
 
        label_indicator_peak_landing_force = tk.Label(label_frame_indicator_jump_event, text=f"Peak landing force at {0.0} s with value {0.0} N", font=("Arial", 12))
        label_indicator_peak_landing_force.grid(row=1, column=1, sticky='w', pady=5, padx=10)
        


        # --------------------------------------------
        n = len(info_load_cells)
        cols = 1 if n == 1 else 2
        rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 2.5 * rows))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        axes = np.atleast_1d(axes).flatten()
        axes = {k: ax for ax, k in zip(axes, info_load_cells)}

        for k in axes.keys(): axes[k].grid(True)

        # Creating empty lines
        lines = {k: axes[k].plot([], [])[0] for k in info_load_cells}

        # Axis configuration
        for k in info_load_cells:
            axes[k].set_title(f"Load Cell {k}")
            axes[k].set_xlabel("Time (s)")
            axes[k].set_ylabel("GRF (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plt.close(fig)
        # --------------------------------------------
                 

        return 50000, func1, (), func2, ()

    
    def testNormalJump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], float) and self.jump_data["mass"]>0): raise ValueError("Invalid mass data")

        # --------------------------------------------

        def analysis_result_calculation(force, time, start_idx, end_idx, ax, entries_params):
            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]

            # Event 1: Start movement
            start_movement = np.where(force <= weight*(entries_params["start_threshold"]["info"]["value"]/100))[0][0]

            # Event 3: Start deceleration = primo valore forza ~0 dopo start braking
            start_deceleration = start_movement + np.where(force[start_movement:] >= weight)[0][0]

            # Event 2: Start braking = primo zero crossing dopo start movement
            start_braking = start_movement + np.argmin(force[start_movement:start_deceleration])

            # Event 5: Takeoff = primo punto dopo peak_takeoff_force con forza sotto soglia
            takeoff = start_deceleration + np.where(force[start_deceleration:] <= weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))[0][0]

            # Event 4: Start concentric = primo zero crossing dopo deceleration
            peak_takeoff_force = start_deceleration + np.argmax(force[start_deceleration:takeoff])

            # Event 6: Landing = primo punto dopo takeoff con forza sopra soglia
            landing = takeoff + np.where(force[takeoff:] > weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))[0][0]

            # Event 7: Peak landing force = massimo dopo il landing
            peak_landing_force = landing + np.argmax(force[landing:])


            # Calcola impulso negativo
            force_net = force - weight
            negative_impulse = np.trapz(force_net[start_movement:start_deceleration], time[start_movement:start_deceleration])
            cumulative_area = cumtrapz(force_net[start_deceleration:], time[start_deceleration:], initial=0)
            # 5. Trova quando l'area cumulativa compensa l'impulso negativo
            balancing = start_deceleration + np.where(cumulative_area >= abs(negative_impulse))[0][0]

            end_push = balancing + np.where(force[balancing:] <= weight)[0][0]




            # Save result

            result = {
                "start_movement": start_movement,
                "start_deceleration": start_deceleration,
                "balancing": balancing,
                "end_push": end_push,
                "start_braking": start_braking,
                "takeoff": takeoff,
                "peak_takeoff_force": peak_takeoff_force,
                "landing": landing,
                "peak_landing_force": peak_landing_force
            }

            # ==== 3. Plot ====

            plotted_elements = []

            events = {
                "Start Movement": start_movement,
                "Start Braking": start_braking,
                "Balancing": balancing,
                "End push": end_push,
                "Start Deceleration": start_deceleration,
                "Peak Takeoff Force": peak_takeoff_force,
                "Takeoff": takeoff,
                "Landing": landing,
                "Peak Landing Force": peak_landing_force
            }
            event_color = ['red', 'orange', 'grey', 'pink', 'blue', 'green', 'brown', 'purple', 'yellow']

            for (name, idx), color in zip(events.items(), event_color):
                plotted_elements.append(ax.scatter(time[idx], force[idx], marker='o', color=color))
                plotted_elements.append(ax.text(time[idx], force[idx] + 30, name, color=color, fontsize=9, ha='center'))

            # 2. Aree colorate per le fasi
            plotted_elements.append(ax.axvspan(time[0], time[start_movement], color='yellow', alpha=0.2, label='Jump preparation'))
            plotted_elements.append(ax.axvspan(time[start_movement], time[peak_takeoff_force], color='orange', alpha=0.2, label='Eccentric Phase'))
            plotted_elements.append(ax.axvspan(time[peak_takeoff_force], time[takeoff], color='green', alpha=0.2, label='Concentric Phase'))
            plotted_elements.append(ax.axvspan(time[takeoff], time[landing], color='blue', alpha=0.2, label='Flight Phase'))
            plotted_elements.append(ax.axvspan(time[landing], time[-1], color='purple', alpha=0.2, label='Landing Phase'))

            plotted_elements.append(ax.hlines(y=weight, xmin=time[0], xmax=time[-1], color='green', alpha=0.2, label='Weight'))

            return result, plotted_elements
        

        def analysis_callback(force, time, start_idx, end_idx, results_analysis, entries_params):

            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]
            force_net = force - weight
            impulse = np.trapz(force_net[results_analysis["balancing"]:results_analysis["takeoff"]+1], time[results_analysis["balancing"]:results_analysis["takeoff"]+1])
            v0 = impulse / self.jump_data["mass"]
            h_force = (v0 ** 2) / (2 * self.jump_data["acceleration_of_gravity"])
            
            time_contact = time[results_analysis["takeoff"]] - time[results_analysis["start_movement"]]
            time_flight = time[results_analysis["landing"]]-time[results_analysis["takeoff"]]

            h_time_flight = 0.5*self.jump_data["acceleration_of_gravity"]*((time_flight/2)**2)
            h_avg = (h_force+h_time_flight)/2

            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
            smart_update_label(label_indicator_jump_height_force, f"Jump height (force): {round(h_force*100,2)} cm", "black")
            smart_update_label(label_indicator_jump_height_time_flight, f"Jump height (time flight): {round(h_time_flight*100,2)} cm", "black")
            smart_update_label(label_indicator_jump_height_avg, f"Jump height (avg): {round(h_avg*100,2)} cm", "black")
            smart_update_label(label_indicator_average_reaction_force, f"Impulse: {round(impulse,2)} m*kg/s", "black")
            smart_update_label(label_indicator_jumping_speed, f"Jumping speed: {round(v0,2)} m/s", "black")
            smart_update_label(label_indicator_flight_time, f"Flight time: {round(time_flight,5)} s", "black")
            smart_update_label(label_indicator_time_contact, f"Contact time: {round(time_contact,5)} s", "black")

            smart_update_label(label_indicator_start_movement, f"Start movement: {round(time[results_analysis["start_movement"]],5)} s", "black")
            smart_update_label(label_indicator_start_braking, f"Start braking at {round(time[results_analysis["start_braking"]],5)} s with value {round(force[results_analysis["start_braking"]],2)} N", "black")
            smart_update_label(label_indicator_start_deceleration, f"Start deceleration: {round(time[results_analysis["start_deceleration"]],5)} s", "black")
            smart_update_label(label_indicator_peak_takeoff_force, f"Peak takeoff force at {round(time[results_analysis["peak_takeoff_force"]],5)} s with value {round(force[results_analysis["peak_takeoff_force"]],2)} N", "black")
            smart_update_label(label_indicator_takeoff, f"Takeoff: {round(time[results_analysis["takeoff"]],5)} s", "black")
            smart_update_label(label_indicator_landing, f"Landing: {round(time[results_analysis["landing"]],5)} s", "black")
            smart_update_label(label_indicator_peak_landing_force, f"Peak landing force at {round(time[results_analysis["peak_landing_force"]],5)} s with value {round(force[results_analysis["peak_landing_force"]],2)} N", "black")

        
        def func1():
            pass


        def func2():
            if not (self.raw_data and self.raw_data["size"]>0): return 

            for i, k in enumerate(info_load_cells):
                lines[k].set_data(self.raw_data["time"], self.raw_data["lc"][i])
                axes[k].relim()
                axes[k].autoscale_view()
            canvas.draw()


            clear_frame(frame_control_test)
            params = [
                {"label":"Start Threshold (%)","type":float,"value":95,"key":"start_threshold","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0},
                {"label":"Takeoff Threshold (%)","type":float,"value":5,"key":"takeoff_threshold","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0}
            ]
            analyze_btn = ttk.Button(frame_control_test, text="Analyze data", command=lambda: self.analysisJumpPhase(analysis_result_calculation, analysis_callback, params))
            analyze_btn.pack(padx=20, pady=20)
            automatic_scaling()  


        # --------------------------------------------
        label_frame_indicator_general_data = tk.LabelFrame(frame_indicator, text=f"General data", padx=10, pady=10)
        label_frame_indicator_general_data.pack(fill="both")

        label_frame_indicator_jump_event = tk.LabelFrame(frame_indicator, text=f"Jump event", padx=10, pady=10)
        label_frame_indicator_jump_event.pack(fill="both")

        label_indicator_mass = tk.Label(label_frame_indicator_general_data, text=f"Mass: {round(self.jump_data["mass"],2)} Kg", font=("Arial", 12))
        label_indicator_mass.grid(row=0, column=0, sticky='w', pady=5, padx=10)

        label_indicator_jump_height_force = tk.Label(label_frame_indicator_general_data, text=f"Jump height (force): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_force.grid(row=0, column=1, sticky='w', pady=5, padx=10)

        label_indicator_jump_height_time_flight = tk.Label(label_frame_indicator_general_data, text=f"Jump height (time flight): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_time_flight.grid(row=0, column=2, sticky='w', pady=5, padx=10)
 
        label_indicator_jump_height_avg = tk.Label(label_frame_indicator_general_data, text=f"Jump height (avg): {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height_avg.grid(row=0, column=3, sticky='w', pady=5, padx=10)

        label_indicator_average_reaction_force = tk.Label(label_frame_indicator_general_data, text=f"Impulse: {0.0} m*Kg/s", font=("Arial", 12))
        label_indicator_average_reaction_force.grid(row=1, column=0, sticky='w', pady=5, padx=10)

        label_indicator_jumping_speed = tk.Label(label_frame_indicator_general_data, text=f"Jumping speed: {0.0} m/s", font=("Arial", 12))
        label_indicator_jumping_speed.grid(row=1, column=1, sticky='w', pady=5, padx=10)

        label_indicator_flight_time = tk.Label(label_frame_indicator_general_data, text=f"Flight time: {0.0} s", font=("Arial", 12))
        label_indicator_flight_time.grid(row=1, column=2, sticky='w', pady=5, padx=10)

        label_indicator_time_contact = tk.Label(label_frame_indicator_general_data, text=f"Contact time: {0.0} s", font=("Arial", 12))
        label_indicator_time_contact.grid(row=1, column=3, sticky='w', pady=5, padx=10)


        label_indicator_start_movement = tk.Label(label_frame_indicator_jump_event, text=f"Start movement: {0.0} s", font=("Arial", 12))
        label_indicator_start_movement.grid(row=0, column=0, sticky='w', pady=5, padx=10)

        label_indicator_start_deceleration = tk.Label(label_frame_indicator_jump_event, text=f"Start deceleration: {0.0} s", font=("Arial", 12))
        label_indicator_start_deceleration.grid(row=0, column=1, sticky='w', pady=5, padx=10)

        label_indicator_takeoff = tk.Label(label_frame_indicator_jump_event, text=f"Takeoff: {0.0} s", font=("Arial", 12))
        label_indicator_takeoff.grid(row=0, column=2, sticky='w', pady=5, padx=10)

        label_indicator_landing = tk.Label(label_frame_indicator_jump_event, text=f"Landing: {0.0} s", font=("Arial", 12))
        label_indicator_landing.grid(row=0, column=3, sticky='w', pady=5, padx=10)

        label_indicator_peak_takeoff_force = tk.Label(label_frame_indicator_jump_event, text=f"Peak takeoff force at {0.0} s with value {0.0} N", font=("Arial", 12))
        label_indicator_peak_takeoff_force.grid(row=1, column=0, sticky='w', pady=5, padx=10)

        label_indicator_start_braking = tk.Label(label_frame_indicator_jump_event, text=f"Start braking at {0.0} s with value {0.0} N", font=("Arial", 12))
        label_indicator_start_braking.grid(row=1, column=1, sticky='w', pady=5, padx=10)

        label_indicator_peak_landing_force = tk.Label(label_frame_indicator_jump_event, text=f"Peak landing force at {0.0} s with value {0.0} N", font=("Arial", 12))
        label_indicator_peak_landing_force.grid(row=1, column=2, sticky='w', pady=5, padx=10)



        # --------------------------------------------
        n = len(info_load_cells)
        cols = 1 if n == 1 else 2
        rows = math.ceil(n / cols)
        fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 2.5 * rows))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        axes = np.atleast_1d(axes).flatten()
        axes = {k: ax for ax, k in zip(axes, info_load_cells)}

        for k in axes.keys(): axes[k].grid(True)

        # Creating empty lines
        lines = {k: axes[k].plot([], [])[0] for k in info_load_cells}

        # Axis configuration
        for k in info_load_cells:
            axes[k].set_title(f"Load Cell {k}")
            axes[k].set_xlabel("Time (s)")
            axes[k].set_ylabel("GRF (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plt.close(fig)
        # --------------------------------------------
                 

        return 50000, func1, (), func2, ()

    
    def testCalculateForce(self):
        # --------------------------------------------

        def analysis_result_calculation(force, time, start_idx, end_idx, ax, entries_params):

            force_avg = np.mean(force)
            force_max = np.argmax(force)


            # Save result

            result = {
                "force_max": force_max
            }

            # ==== 3. Plot ====

            plotted_elements = []

            events = {
                "Force Max": force_max,
            }
            event_color = ['red']

            for (name, idx), color in zip(events.items(), event_color):
                plotted_elements.append(ax.scatter(time[idx], force[idx], marker='o', color=color))
                plotted_elements.append(ax.text(time[idx], force[idx] + 30, name, color=color, fontsize=9, ha='center'))

            plotted_elements.append(ax.hlines(y=force_avg, xmin=time[0], xmax=time[-1], color='green', alpha=0.2, label='Force avg'))

            return result, plotted_elements
        

        def analysis_callback(force, time, start_idx, end_idx, results_analysis, entries_params):

            force_avg = np.mean(force)
            resulting_mass = force_avg / self.jump_data["acceleration_of_gravity"]
            max_mass = force[results_analysis["force_max"]] / self.jump_data["acceleration_of_gravity"]

            smart_update_label(label_indicator_resulting_mass, f"Resulting Mass: {round(resulting_mass,5)} Kg", "black")
            smart_update_label(label_indicator_max_mass, f"Max Mass: {round(max_mass,5)} Kg", "black")
            
        
        def func1():
            pass


        def func2():
            if not (self.raw_data and self.raw_data["size"]>0): return 

            line.set_data(self.raw_data["time"], np.sum(self.raw_data["lc"], axis=0))
            axis.relim()
            axis.autoscale_view()
            canvas.draw()

            clear_frame(frame_control_test)
            params = []
            analyze_btn = ttk.Button(frame_control_test, text="Analyze data", command=lambda: self.analysisJumpPhase(analysis_result_calculation, analysis_callback, params))
            analyze_btn.pack(padx=20, pady=20)
            automatic_scaling()  


        # --------------------------------------------

        label_indicator_resulting_mass = tk.Label(frame_indicator, text=f"Resulting Mass: {round(0.0,5)} Kg", font=("Arial", 12))
        label_indicator_resulting_mass.grid(row=0, column=0, sticky='w', pady=5, padx=10)

        label_indicator_max_mass = tk.Label(frame_indicator, text=f"Max Mass: {round(0.0,5)} Kg", font=("Arial", 12))
        label_indicator_max_mass.grid(row=0, column=1, sticky='w', pady=5, padx=10)

       


        # --------------------------------------------
        fig, axis = plt.subplots(1, 1, figsize=(5, 2))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts
        axis.grid(True)

        # Creating empty lines
        line = axis.plot([], [])[0]

        # Axis configuration
        axis.set_title(f"Mass chart")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("GRF (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plt.close(fig)
        # --------------------------------------------
                 

        return 50000, func1, (), func2, ()
     

    def testCalculateMass(self):

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

        # --------------------------------------------

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
        axis.grid(True)

        # Creating empty lines
        line = axis.plot([], [])[0]

        # Axis configuration
        axis.set_title(f"Mass chart")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("GRF (N)")

        # Tkinter Integration
        canvas = FigureCanvasTkAgg(fig, master=frame_chart)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        canvas.draw()

        # Adding the toolbar above the chart
        toolbar = NavigationToolbar2Tk(canvas, frame_chart)
        toolbar.pack(side=tk.TOP, fill=tk.X)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plt.close(fig)
        # --------------------------------------------
     
        return 50000, func1, (), func2, ()


    def startRepresentation(self, test):
        smart_update_label(label_status_acquisition, "Initialization...", "black")

        close_open_windows()
        
        self.stopRepresentation()

        clear_frame(frame_control_test)
        clear_frame(frame_indicator)
        clear_frame(frame_chart)
        try:
            max_num_samples, func1, args1, func2, args2 = test()
            automatic_scaling()

            self.initializationRawData(max_num_samples)
            if not data_acquirer.startAcquisition(): raise Exception("Problems with acquisition")

            self.stop_representation=False
            self.request_stop_representation=False

            func = partial(self.run, func1, args1, func2, args2)
            self.after_id = root.after(self.function_recall_time, func)

        except Exception as e: 
            self.analysisError()
            show_warning("Attention!", e)

        
    def requestStopRepresentation(self):
        data_acquirer.requestStopAcquisition()
        self.request_stop_representation=True

    def stopRepresentation(self):
        try: root.after_cancel(self.after_id)
        except: pass

        self.stop_representation=True
        self.request_stop_representation=True

        data_acquirer.stopAcquisition()
        
        while not data_queue.empty():
            try: data_queue.get_nowait()
            except queue.Empty: break
        
        smart_update_label(label_status_acquisition, "Acquisition stopped", "black")

    
    def addData(self, data):
        if self.raw_data and data:
            if None not in list(data["values"].values()):
                lc_values = [data["values"][k] for k in info_load_cells]
                self.raw_data.append(data["time"], lc_values)






class DataAcquirer(threading.Thread):
    def __init__(self):
        super().__init__()

        self.acquisition_activity=True
        self.acquisition=False

        self.acquiring=threading.RLock()

        self.events = queue.Queue()
        self.next_state = queue.Queue()
        self.reader=None

        self.natual_next_state = None

        self.time_acquiring = 0.0005
        self.pause_search_problems = 0.0001
    

    def run(self):
        while self.acquisition_activity:
            with self.acquiring:

                try:
                   
                    state_acquisition = self.next_state.get(block=False)

                    #print(f"State acquisition: {state_acquisition}")

                    ns = None

                    if state_acquisition == StateDataAcquirer.START:
                        self.events.put(StateDataAcquirer.START)
                        
                        try:
                            self.reader = arduino.multiple_readings(silent_time_reset_buffer=0.5)
                            next(self.reader)
                            self.reader.send(StateMultipleReadings.START)
                            self.reader.send({"command":"start_reading"})
                            
                            ns = StateDataAcquirer.GET
                        except Exception as e: 
                            ns = StateDataAcquirer.STOP
                            self.events.put(e)

                    elif state_acquisition == StateDataAcquirer.STOP:
                        self.events.put(StateDataAcquirer.STOP)

                        try:
                            if self.reader is not None:
                                self.reader.send(StateMultipleReadings.STOP)
                                self.reader.send({"command":"stop_reading"})
                                ns = StateDataAcquirer.GET_LAST
                            else: raise Exception("Reader is None")
                        except Exception as e: 
                            ns = StateDataAcquirer.CLOSE
                            self.events.put(e)

                    elif state_acquisition == StateDataAcquirer.CLOSE:
                        self.events.put(StateDataAcquirer.CLOSE)
                        try:
                            try: 
                                if self.reader is not None: self.reader.send(StateMultipleReadings.CLOSE)
                                else: raise Exception("Reader is None")
                            except StopIteration: pass
                        except Exception as e: self.events.put(e)
                        finally: self.reader = None

                    elif state_acquisition == StateDataAcquirer.GET:
                        
                        try:
                            if self.reader is not None:
                                result = self.reader.send(StateMultipleReadings.NEXT)
                                self.reader.send(None)
                                data_queue.put(result)
                                ns = StateDataAcquirer.GET
                            else: raise Exception("Reader is None")
                        except Exception as e:
                            data_queue.put(None)
                            ns = StateDataAcquirer.STOP

                    elif state_acquisition == StateDataAcquirer.GET_LAST:

                        try:
                            if self.reader is not None:
                                result = self.reader.send(StateMultipleReadings.NEXT)
                                self.reader.send(None)
                                
                                if "command" in result and result["command"] == "stop_reading": 
                                    data_queue.put(StateDataAcquirer.FINISH)
                                    ns = StateDataAcquirer.CLOSE
                                elif "command" in result and result["command"] != "stop_reading": 
                                    raise Exception("Problems reading")
                                else: 
                                    data_queue.put(result)
                                    ns = StateDataAcquirer.GET_LAST
                            else: raise Exception("Reader is None")
                        except Exception as e:
                            data_queue.put(None)
                            ns = StateDataAcquirer.CLOSE

                    elif state_acquisition == StateDataAcquirer.START_REQUEST:
                        self.events.put(StateDataAcquirer.START_REQUEST)

                    elif state_acquisition == StateDataAcquirer.END_REQUEST:
                        self.events.put(StateDataAcquirer.END_REQUEST)
                        if self.natual_next_state is not None:
                            self.next_state.put(self.natual_next_state)
                            self.natual_next_state = None

                    else: pass


                    if self.next_state.empty(): self.next_state.put(ns)
                    elif self.next_state.qsize()==1: self.natual_next_state = ns
                    else: self.natual_next_state = None
                
                            
                except queue.Empty: pass

            time.sleep(self.time_acquiring)


    def startAcquisition(self):
        with self.acquiring: 
            self.next_state.put(StateDataAcquirer.START_REQUEST)
            self.next_state.put(StateDataAcquirer.STOP)
            self.next_state.put(StateDataAcquirer.CLOSE)
            self.next_state.put(StateDataAcquirer.START)
            self.next_state.put(StateDataAcquirer.END_REQUEST)
        return not self.check_problem(StateDataAcquirer.START)

    def requestStopAcquisition(self):
        with self.acquiring: 
            self.next_state.put(StateDataAcquirer.START_REQUEST)
            self.next_state.put(StateDataAcquirer.STOP)
            self.next_state.put(StateDataAcquirer.END_REQUEST)
        return not self.check_problem(StateDataAcquirer.STOP)

    def stopAcquisition(self):
        with self.acquiring:
            self.next_state.put(StateDataAcquirer.START_REQUEST)
            self.next_state.put(StateDataAcquirer.STOP)
            self.next_state.put(StateDataAcquirer.CLOSE)
            self.next_state.put(StateDataAcquirer.END_REQUEST)
        return not self.check_problem(StateDataAcquirer.CLOSE)

    def stopAcquisitionActivity(self):
        self.stopAcquisition()
        self.acquisition_activity=False
        

    def check_problem(self, state, num=0):
        problem = {}
        start_request = False
        last_state = None
        while True:
            event = self.events.get()
            #print(event)
            if isinstance(event, StateDataAcquirer) and event==StateDataAcquirer.START_REQUEST: start_request = True
            elif isinstance(event, StateDataAcquirer) and event==StateDataAcquirer.END_REQUEST and start_request: break
            elif isinstance(event, StateDataAcquirer) and start_request: 
                if event not in problem.keys(): problem[event] = [[]]
                elif event in problem.keys(): problem[event].append([])
                last_state = event
            elif not isinstance(event, StateDataAcquirer) and start_request: 
                problem[last_state][len(problem[last_state])-1].append(event)
            time.sleep(self.pause_search_problems)

        return len(problem[state][num])!=0
    









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

    list_info_device = list_device

def open_device_connection(info_device, timeout):
    global arduino, info_load_cells
    close_device_connection()
    smart_update_label(label_status_connection, "Connecting...", "orange")
    try:
        arduino = Arduino(info_device, timeout) 
        arduino.open_connection()
        info_load_cells = arduino.write_and_read({"command":"get_info"})["lc"]
        command_button_scale_tare()
        verify_the_possibility_of_communicating()
    except:
        close_device_connection()
        func_Focus_combobox_menu_device(None)

def clear_frame(frame):
    #print(f"CLEAR: {frame}")
    for widget in frame.winfo_children(): widget.destroy()
    automatic_scaling()
    

def close_device_connection():
    global arduino, info_load_cells
    info_load_cells = []

    smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
    smart_update_label(label_status_connection, "Disconnecting...", "black")
    smart_update_label(label_scale_tare, "Not calibrated!", "red")

    data_rappresentor.stopRepresentation()

    if arduino is not None: arduino.close_connection()
    arduino = None

    verify_the_possibility_of_communicating()

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
            info_device["baudrate"] = 921600 # 460800
            open_device_connection(info_device, 2)
            break
        
def verify_the_possibility_of_communicating():
    try:
        if arduino is not None and arduino.check_connection(): 
            smart_update_label(label_status_connection, "Available and connected", "green")
            return StateCommunicationDevice.AVAILABLE_CONNECTED
        else:
            smart_update_label(label_status_connection, "Available but not connected", "red")
            return StateCommunicationDevice.AVAILABLE_NOT_CONNECTED
    except ResourceBusyException: 
        smart_update_label(label_status_connection, "Not available", "red")
        return StateCommunicationDevice.NOT_AVAILABLE


def command_button_test(test):
    if verify_the_possibility_of_communicating()==StateCommunicationDevice.AVAILABLE_CONNECTED and info_load_cells: data_rappresentor.startRepresentation(test)

def func_WM_DELETE_WINDOW():
    data_rappresentor.stopRepresentation()
    close_device_connection()
    data_acquirer.stopAcquisitionActivity()
    data_acquirer.join()
    root.quit()
    root.destroy()

def command_button_scale_tare():
    smart_update_label(label_scale_tare, "Calibration...", "black")

    try:
        result = arduino.write_and_read({"command":"scale_tare"})["lc"]
        result_string = ""
        for i, k in enumerate(info_load_cells):
            result_string += f"{k}: {result[k]}"
            if i < len(info_load_cells)-1: result_string += " | "
        smart_update_label(label_scale_tare, result_string, "black")
    except Exception as e:
        close_device_connection()
        func_Focus_combobox_menu_device(None)

                
def smart_update_label(label, text=None, fg=None):
    current_text = label.cget("text")
    current_fg = label.cget("fg")

    if (text is not None and text != current_text) or (fg is not None and fg != current_fg):
        label.config(text=text if text is not None else current_text, fg=fg if fg is not None else current_fg)
        label.update_idletasks()


def show_warning(title, message):
    warn = tk.Tk()

    if "win_warning" not in open_windows: open_windows["win_warning"] = []
    open_windows["win_warning"].append(warn)

    warn.withdraw()
    messagebox.showwarning(title, message)


def close_open_windows():
    for w in open_windows["win_analysis"]: 
        try: w.destroy()
        except: pass
    open_windows["win_warning"].clear()

    for w in open_windows["win_warning"]:
        try: w.destroy()
        except: pass
    open_windows["win_warning"].clear()



# -------------------------------------
open_windows = {
    "root":None,
    "win_warning": [],
    "win_analysis":[]
}
info_load_cells = []
list_info_device = []

root = tk.Tk()
open_windows["root"] = root

arduino = None
data_queue = queue.Queue()

data_acquirer = DataAcquirer()
data_acquirer.start()

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
button_test_depth_jump = tk.Button(frame_acquisition, text="Depth jump", command=lambda: command_button_test(data_rappresentor.testDepthJump))
button_test_depth_jump.pack(side=tk.LEFT, padx=20)
button_test_calculate_mass = tk.Button(frame_acquisition, text="Calculate mass", command=lambda: command_button_test(data_rappresentor.testCalculateMass))
button_test_calculate_mass.pack(side=tk.LEFT, padx=20)
button_test_calculate_force = tk.Button(frame_acquisition, text="Calculate force", command=lambda: command_button_test(data_rappresentor.testCalculateForce))
button_test_calculate_force.pack(side=tk.LEFT, padx=20)
button_stop_test = tk.Button(frame_acquisition, text="Stop test", command=data_rappresentor.requestStopRepresentation)
button_stop_test.pack(side=tk.LEFT, padx=20)
button_force_stop = tk.Button(frame_acquisition, text="Force stop", command=data_rappresentor.stopRepresentation)
button_force_stop.pack(side=tk.LEFT, padx=20)

frame_acquisition_status = tk.Frame(root)
frame_acquisition_status.pack(pady=10)
label_status_acquisition = tk.Label(frame_acquisition_status, text=f"Acquisition stopped", font=("Arial", 15))
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


automatic_scaling()
# Starting root mainloop
root.mainloop()


