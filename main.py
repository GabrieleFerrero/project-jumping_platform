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
from scipy.signal import savgol_filter
from matplotlib.patches import Rectangle
import math
from datetime import datetime



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
            if message_arduino_read is not None: return True
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
            if "code" in result and result["code"] == "ERROR": raise ArduinoCommunicationException("Bad json")
            return result
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
    
        self.function_recall_time=2 # ms (milliseconds)
        
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
                func1(*args1)
                func2(*args2)

            func = partial(self.run, func1, args1, func2, args2)
            self.after_id = root.after(self.function_recall_time, func)


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
                                    
                results_analysis, plot_el = analysis_result_calculation(selected_force, selected_time, selection_area["start"], ax, entries_params)
                plotted_elements["data_analysis"] += plot_el

                update_plot()

            except: show_warning("Error", "Problem in the analyses")


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
            scroll.config(to=self.raw_data["time"][-1]-entries_params["window_duration"]["info"]["value"])

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


        def confirm_and_close():
            if selection_area["start"] is None or selection_area["end"] is None or results_analysis is None: return
            analysis_callback(selection_area["start"], selection_area["end"], results_analysis)
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
            {"label":"Savgol Window Length","type":int,"value":11,"key":"savgol_filter_window_length","ptype":"processing","cond":lambda x: 1 <= x <= np.iinfo(np.uint32).max and x % 2 == 1},
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
        scroll = ttk.Scale(plot_frame, from_=0, orient='horizontal', length=800, command=lambda val: update_view_x(float(val)))
        scroll.pack(fill=tk.X, expand=False)

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
        ttk.Button(win, text="Calculating values", command=confirm_and_close).grid(row=2, column=1, pady=20)

        # ------------ Default initialization ----------
        update_view_x(0)


    def testDepthJump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], float) and self.jump_data["mass"]>0): raise ValueError("Invalid mass data")

        # --------------------------------------------


        def analysis_result_calculation(force, time, start_idx, ax, entries_params):
            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]

            dFdt = np.gradient(force, time)

            start_amortization = np.argmax(force >= weight*(entries_params["start_threshold"]["info"]["value"]/100))
            takeoff = start_amortization + np.argmax(force[start_amortization:] <= weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))
            peak_amortization_force = start_amortization + np.argmax(force[start_amortization:takeoff])

            start_push_off_threshold_max = peak_amortization_force + np.argmax(force[peak_amortization_force:takeoff] <= force[peak_amortization_force]*(entries_params["start_push_off_threshold_max"]["info"]["value"]/100))
            start_push_off_threshold_min = peak_amortization_force + np.argmax(force[peak_amortization_force:takeoff] <= force[peak_amortization_force]*(entries_params["start_push_off_threshold_min"]["info"]["value"]/100))
            start_push_off = start_push_off_threshold_max + np.abs(dFdt[start_push_off_threshold_max:start_push_off_threshold_min] - 0).argmin()

            landing = takeoff + np.argmax(force[takeoff:] > weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))
            peak_landing_force = landing + np.argmax(force[landing:])

            # Save result

            result = {
                "start_amortization": start_idx + start_amortization,
                "start_push_off": start_idx + start_push_off,
                "peak_amortization_force": start_idx + peak_amortization_force,
                "takeoff": start_idx + takeoff,
                "landing": start_idx + landing,
                "peak_landing_force": start_idx + peak_landing_force
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
        

        def analysis_callback(start_idx, end_idx, results_analysis):
            
            force = np.sum(self.raw_data["lc"], axis=0)

            time_contact = self.raw_data["time"][results_analysis["takeoff"]] - self.raw_data["time"][results_analysis["start_amortization"]]
            time_flight = self.raw_data["time"][results_analysis["landing"]]-self.raw_data["time"][results_analysis["takeoff"]]

            F_avg = np.mean(force[results_analysis["start_push_off"]:results_analysis["takeoff"]])
            v0 = math.sqrt((2* F_avg * time_contact) / self.jump_data["mass"])
            h = (v0 ** 2) / (2 * self.jump_data["acceleration_of_gravity"])

            
            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
            smart_update_label(label_indicator_jump_height, f"Jump height: {round(h*100,2)} cm", "black")
            smart_update_label(label_indicator_average_reaction_force, f"Average reaction force: {round(F_avg,2)} N", "black")
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
                {"label":"Start Push-off Threshold Max (%)","type":float,"value":70,"key":"start_push_off_threshold_max","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0},
                {"label":"Start Push-off Threshold Min (%)","type":float,"value":30,"key":"start_push_off_threshold_min","ptype":"processing","cond":lambda x: 0.0 <= x <= 100.0}
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
        label_indicator_mass.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_height = tk.Label(label_frame_indicator_general_data, text=f"Jump height: {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height.pack(side=tk.LEFT, padx=20)

        label_indicator_average_reaction_force = tk.Label(label_frame_indicator_general_data, text=f"Average reaction force: {0.0} N", font=("Arial", 12))
        label_indicator_average_reaction_force.pack(side=tk.LEFT, padx=20)

        label_indicator_jumping_speed = tk.Label(label_frame_indicator_general_data, text=f"Jumping speed: {0.0} m/s", font=("Arial", 12))
        label_indicator_jumping_speed.pack(side=tk.LEFT, padx=20)

        label_indicator_flight_time = tk.Label(label_frame_indicator_general_data, text=f"Flight time: {0.0} s", font=("Arial", 12))
        label_indicator_flight_time.pack(side=tk.LEFT, padx=20)

        label_indicator_time_contact = tk.Label(label_frame_indicator_general_data, text=f"Contact time: {0.0} s", font=("Arial", 12))
        label_indicator_time_contact.pack(side=tk.LEFT, padx=20)


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
        # --------------------------------------------
                 

        return 50000, func1, (), func2, ()

    
    def testNormalJump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], float) and self.jump_data["mass"]>0): raise ValueError("Invalid mass data")

        # --------------------------------------------

        def analysis_result_calculation(force, time, start_idx, ax, entries_params):
            weight = self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]

            # Event 1: Start movement
            start_movement = np.argmax(force <= weight*(entries_params["start_threshold"]["info"]["value"]/100))

            # Event 3: Start deceleration = primo valore forza ~0 dopo start braking
            start_deceleration = start_movement + np.argmax(force[start_movement:] >= weight)

            # Event 2: Start braking = primo zero crossing dopo start movement
            start_braking = start_movement + np.argmin(force[start_movement:start_deceleration])

            # Event 5: Takeoff = primo punto dopo peak_takeoff_force con forza sotto soglia
            takeoff = start_deceleration + np.argmax(force[start_deceleration:] <= weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))

            # Event 4: Start concentric = primo zero crossing dopo deceleration
            peak_takeoff_force = start_deceleration + np.argmax(force[start_deceleration:takeoff])

            # Event 6: Landing = primo punto dopo takeoff con forza sopra soglia
            landing = takeoff + np.argmax(force[takeoff:] > weight*(entries_params["takeoff_threshold"]["info"]["value"]/100))

            # Event 7: Peak landing force = massimo dopo il landing
            peak_landing_force = landing + np.argmax(force[landing:])

            # Save result

            result = {
                "start_movement": start_idx + start_movement,
                "start_deceleration": start_idx + start_deceleration,
                "start_braking": start_idx + start_braking,
                "takeoff": start_idx + takeoff,
                "peak_takeoff_force": start_idx + peak_takeoff_force,
                "landing": start_idx + landing,
                "peak_landing_force": start_idx + peak_landing_force
            }

            # ==== 3. Plot ====

            plotted_elements = []

            events = {
                "Start Movement": start_movement,
                "Start Braking": start_braking,
                "Start Deceleration": start_deceleration,
                "Peak Takeoff Force": peak_takeoff_force,
                "Takeoff": takeoff,
                "Landing": landing,
                "Peak Landing Force": peak_landing_force
            }
            event_color = ['red', 'orange', 'blue', 'green', 'brown', 'purple', 'yellow']

            for (name, idx), color in zip(events.items(), event_color):
                plotted_elements.append(ax.scatter(time[idx], force[idx], marker='o', color=color))
                plotted_elements.append(ax.text(time[idx], force[idx] + 30, name, color=color, fontsize=9, ha='center'))

            # 2. Aree colorate per le fasi
            plotted_elements.append(ax.axvspan(time[0], time[start_movement], color='yellow', alpha=0.2, label='Jump preparation'))
            plotted_elements.append(ax.axvspan(time[start_movement], time[peak_takeoff_force], color='orange', alpha=0.2, label='Eccentric Phase'))
            plotted_elements.append(ax.axvspan(time[peak_takeoff_force], time[takeoff], color='green', alpha=0.2, label='Concentric Phase'))
            plotted_elements.append(ax.axvspan(time[takeoff], time[landing], color='blue', alpha=0.2, label='Flight Phase'))
            plotted_elements.append(ax.axvspan(time[landing], time[-1], color='purple', alpha=0.2, label='Landing Phase'))

            return result, plotted_elements
        

        def analysis_callback(start_idx, end_idx, results_analysis):

            force = np.sum(self.raw_data["lc"], axis=0)

            time_contact = self.raw_data["time"][results_analysis["takeoff"]] - self.raw_data["time"][results_analysis["start_movement"]]
            time_flight = self.raw_data["time"][results_analysis["landing"]]-self.raw_data["time"][results_analysis["takeoff"]]

            F_avg = np.mean(force[results_analysis["start_movement"]:results_analysis["takeoff"]])
            v0 = math.sqrt((2* F_avg * time_contact) / self.jump_data["mass"])
            h = (v0 ** 2) / (2 * self.jump_data["acceleration_of_gravity"])

            
            smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
            smart_update_label(label_indicator_jump_height, f"Jump height: {round(h*100,2)} cm", "black")
            smart_update_label(label_indicator_average_reaction_force, f"Average reaction force: {round(F_avg,2)} N", "black")
            smart_update_label(label_indicator_jumping_speed, f"Jumping speed: {round(v0,2)} m/s", "black")
            smart_update_label(label_indicator_flight_time, f"Flight time: {round(time_flight,5)} s", "black")
            smart_update_label(label_indicator_time_contact, f"Contact time: {round(time_contact,5)} s", "black")

            smart_update_label(label_indicator_start_movement, f"Start movement: {round(self.raw_data["time"][results_analysis["start_movement"]],5)} s", "black")
            smart_update_label(label_indicator_start_braking, f"Start braking at {round(self.raw_data["time"][results_analysis["start_braking"]],5)} s with value {round(force[results_analysis["start_braking"]],2)} N", "black")
            smart_update_label(label_indicator_start_deceleration, f"Start deceleration: {round(self.raw_data["time"][results_analysis["start_deceleration"]],5)} s", "black")
            smart_update_label(label_indicator_peak_takeoff_force, f"Peak takeoff force at {round(self.raw_data["time"][results_analysis["peak_takeoff_force"]],5)} s with value {round(force[results_analysis["peak_takeoff_force"]],2)} N", "black")
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
        label_indicator_mass.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_height = tk.Label(label_frame_indicator_general_data, text=f"Jump height: {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height.pack(side=tk.LEFT, padx=20)

        label_indicator_average_reaction_force = tk.Label(label_frame_indicator_general_data, text=f"Average reaction force: {0.0} N", font=("Arial", 12))
        label_indicator_average_reaction_force.pack(side=tk.LEFT, padx=20)

        label_indicator_jumping_speed = tk.Label(label_frame_indicator_general_data, text=f"Jumping speed: {0.0} m/s", font=("Arial", 12))
        label_indicator_jumping_speed.pack(side=tk.LEFT, padx=20)

        label_indicator_flight_time = tk.Label(label_frame_indicator_general_data, text=f"Flight time: {0.0} s", font=("Arial", 12))
        label_indicator_flight_time.pack(side=tk.LEFT, padx=20)

        label_indicator_time_contact = tk.Label(label_frame_indicator_general_data, text=f"Contact time: {0.0} s", font=("Arial", 12))
        label_indicator_time_contact.pack(side=tk.LEFT, padx=20)


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
        # --------------------------------------------
     
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
            self.after_id = root.after(self.function_recall_time, func)
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
            if None not in list(data["values"].values()):
                if self.raw_data["size"]<self.raw_data["max_size"]:
                    if self.raw_data["size"]==0:  self.raw_data["initial_time"] = data["time"]
                    self.raw_data["unsliced_time"][self.raw_data["size"]] = data["time"]-self.raw_data["initial_time"]
                    for i, k in enumerate(info_load_cells): self.raw_data["unsliced_lc"][i, self.raw_data["size"]] = data["values"][k]
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

        self.time_sleep_acquiring = 0.0005
        self.time_sleep_not_acquiring=0.3

        self._stopAcquisition()
    
    def run(self):
        while self.acquisition_activity:
            with self.acquiring:
                if self.acquisition:
                    try: data_dq.append(arduino.read())
                    except Exception as e: 
                        #print(e)
                        data_dq.append(None)
                        self._stopAcquisition()
                    

            if self.acquisition: time.sleep(self.time_sleep_acquiring)
            else: time.sleep(self.time_sleep_not_acquiring)


    def _startAcquisition(self):
        self.acquisition=True
        try: 
            if arduino is not None: arduino.write_and_read({"command":"start_reading"})
        except: pass

    def startAcquisition(self):
        with self.acquiring:
            self._startAcquisition()

    def _stopAcquisition(self):
        self.acquisition=False
        try: 
            if arduino is not None: arduino.write_and_read({"command":"stop_reading"})
        except: pass

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

    list_info_device = list_device

def open_device_connection(info_device, timeout):
    global arduino
    close_device_connection()
    smart_update_label(label_status_connection, "Connecting...", "orange")
    try: 
        arduino = Arduino(info_device, timeout) 
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
            result = arduino.write_and_read({"command":"scale_tare"})["lc"]
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




# -------------------------------------

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
button_test_depth_jump = tk.Button(frame_acquisition, text="Depth jump", command=lambda: command_button_test(data_rappresentor.testDepthJump))
button_test_depth_jump.pack(side=tk.LEFT, padx=20)
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


