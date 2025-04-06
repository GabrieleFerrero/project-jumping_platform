import tkinter as tk
from tkinter import ttk
from tkinter import filedialog, messagebox
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import serial
from serial.tools import list_ports
from datetime import datetime
import json
from abc import ABC, abstractmethod
import threading
import time
import queue
import pandas as pd
import numpy as np
from collections import deque
from functools import partial



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


class Message():
    def __init__(self):
        self.messagge = {
            'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S.%f'),
            'action': '',
            'code': Code("NULL"),
            'code_description': '',
            'recalled_actions': [],
            'response': None
        }

    def __str__(self):
        
        messagge_str = {
            'timestamp': str(self.messagge['timestamp']),
            'action': str(self.messagge['action']),
            'code': str(self.messagge['code']),
            'code_description': str(self.messagge['code_description']),
            'recalled_actions': [json.loads(str(action)) for action in self.messagge['recalled_actions']],
            'response': str(self.messagge['response'])
        }
        
        return json.dumps(messagge_str, indent=4)
    
    def set_action(self, action):
        self.messagge["action"]=action

    def set_code(self, code):
        self.messagge["code"]=code
    
    def set_code_description(self, code_description):
        self.messagge["code_description"]=code_description
    
    def set_response(self, response):
        self.messagge["response"]=response
    
    def add_recalled_actions(self, recalled_actions):
        self.messagge["recalled_actions"].append(recalled_actions)
    
    def get_code(self):
        return self.messagge["code"]

    def get_response(self):
        return self.messagge["response"]
    

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






class DataProcessor():
    def __init__(self):

        self.after_id=None
        self.raw_data = None
        self.jump_data={
            "mass": 0.0,
            "acceleration_of_gravity": 9.81
        }
        self.fs_representation=True # force stop representation
        self.representation=False 
    
        self.time_sleep=10 # ms (milliseconds)
        self.max_number_of_samples = 50000

        self.initialization_raw_data()
        self.force_stop_representation()


    def run(self, func1, args1, func2, args2):
        if not self.fs_representation:
            try:
                data = data_dq.popleft()
                if data is not None: 
                    self.add_data(data)
                    if self.representation:
                        smart_update_label(label_status_acquisition, "Acquisition in progress...", "green")
                    elif not self.representation:
                        smart_update_label(label_status_acquisition, "Saving the last samples...", "orange")
                    func1(*args1)
                else: self.analysis_error()
            except IndexError: # deque is empty
                if self.representation:
                    smart_update_label(label_status_acquisition, "Waiting for data...", "blue")
                elif not self.representation:
                    self.force_stop_representation()
                    smart_update_label(label_status_acquisition, "Analysis in progress...", "purple")
                    func2(*args2)
                    smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
            except:
                self.analysis_error()

        func = partial(self.run, func1, args1, func2, args2)
        self.after_id = root.after(self.time_sleep, func)


    def analysis_error(self):
        smart_update_label(label_status_acquisition, "Error in analysis!", "red")
        if not check_device_connection():
            self.force_stop_representation()
            close_device_connection()
            func_Focus_combobox_menu_device(None)


    def initialization_raw_data(self):
        self.raw_data={
            "initial_time":0.0,
            "time":np.zeros(self.max_number_of_samples),
            "lc":{k: np.zeros(self.max_number_of_samples) for k in info_load_cells},
            "size": 0
        }
        
    

    def test_normal_jump(self):
        if not (self.jump_data and isinstance(self.jump_data["mass"], int) and self.jump_data["mass"]>0): return

        # --------------------------------------------
        clear_frame(frame_chart)

        fig, axes = plt.subplots(len(info_load_cells), 1, figsize=(5, 2 * len(info_load_cells)))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        if len(info_load_cells) == 1:
            axes = [axes]  # If there is only one graph, convert it to a list for consistency

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

        automatic_scaling()
        # --------------------------------------------
        clear_frame(frame_indicator)

        label_frame_indicator_general_data = tk.LabelFrame(frame_indicator, text=f"General data", padx=10, pady=10)
        label_frame_indicator_general_data.pack(fill="both")

        label_frame_indicator_jump_time = tk.LabelFrame(frame_indicator, text=f"Jump time", padx=10, pady=10)
        label_frame_indicator_jump_time.pack(fill="both")

        label_frame_indicator_first_touch = tk.LabelFrame(frame_indicator, text=f"First touch", padx=10, pady=10)
        label_frame_indicator_first_touch.pack(fill="both")

        label_frame_indicator_jump_power = tk.LabelFrame(frame_indicator, text=f"Jump power", padx=10, pady=10)
        label_frame_indicator_jump_power.pack(fill="both")

        label_indicator_mass = tk.Label(label_frame_indicator_general_data, text=f"Mass: {self.jump_data['mass']} Kg", font=("Arial", 12))
        label_indicator_mass.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_height = tk.Label(label_frame_indicator_general_data, text=f"Jump height: {0.0} cm", font=("Arial", 12))
        label_indicator_jump_height.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_time_AVG = tk.Label(label_frame_indicator_jump_time, text=f"Jump time AVG: {0.0} s", font=("Arial", 12))
        label_indicator_jump_time_AVG.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_time = {k: tk.Label(label_frame_indicator_jump_time, text=f"Jump time {k}: {0.0} s", font=("Arial", 12)) for k in info_load_cells}
        for k in info_load_cells:
            label_indicator_jump_time[k].pack(side=tk.LEFT, padx=20)

        label_indicator_first_touch = {k: tk.Label(label_frame_indicator_first_touch, text=f"First touch {k}: {0.0} s", font=("Arial", 12)) for k in info_load_cells}
        for k in info_load_cells:
            label_indicator_first_touch[k].pack(side=tk.LEFT, padx=20)

        label_indicator_jump_power_AVG = tk.Label(label_frame_indicator_jump_power, text=f"Jump power AVG: {0.0} N", font=("Arial", 12))
        label_indicator_jump_power_AVG.pack(side=tk.LEFT, padx=20)

        label_indicator_jump_power = {k: tk.Label(label_frame_indicator_jump_power, text=f"Jump power {k}: {0.0} N", font=("Arial", 12)) for k in info_load_cells}
        for k in info_load_cells:
            label_indicator_jump_power[k].pack(side=tk.LEFT, padx=20)
    

        automatic_scaling()
        # --------------------------------------------


        def func1():
            for k in info_load_cells:
                lines[k].set_data(self.raw_data["time"], self.raw_data["lc"][k])
                axes[k].relim()
                axes[k].autoscale_view()
                canvas.draw()


        def func2():
            if not (self.raw_data): return 

            theshold_jump = 60 # Newton

            if self.raw_data["time"] and self.raw_data["lc"]:
    
                index_first_moment_jump = {k: 0.0 for k in info_load_cells}
                index_last_moment_jump = {k: 0.0 for k in info_load_cells}


                for k in info_load_cells:
                    mask = np.array([(val <= theshold_jump) for val in self.raw_data["lc"][k]])
                    indices = np.where(mask)[0]
                    intervals = []
                    if len(indices) > 0:
                        start = indices[0]
                        for i in range(1, len(indices)):
                            if indices[i] != indices[i - 1] + 1: 
                                intervals.append((start, indices[i - 1]))
                                start = indices[i]
                        intervals.append((start, indices[-1]))

                    if len(intervals)>0:
                        index_first_moment_jump[k] = intervals[0][0]
                        index_last_moment_jump[k] = intervals[0][1]
                    else: return


                if -1 not in index_first_moment_jump.values() and -1 not in index_last_moment_jump.values():
                    data = {}
                    for k in info_load_cells: 
                        data["first_touch"][k]=self.raw_data["time"][index_last_moment_jump[k]]

                    for k in info_load_cells: 
                        data["jump_time"][k]=data["first_touch"][k]-self.raw_data["time"][index_first_moment_jump[k]]

                    data["jump_time_AVG"]=sum(data["jump_time"].values())/len(data["jump_time"])

                    data["jump_height"]=(1/2)*self.jump_data["acceleration_of_gravity"]*((data["jump_time_AVG"]/2)**2)

                    for k in info_load_cells: 
                        data["jump_power"][k] = (self.jump_data["mass"]*self.jump_data["acceleration_of_gravity"]*data["jump_height"])/(data["jump_time"][k]/2)
                    
                    data["jump_power_AVG"]=sum(data["jump_power"].values())/len(data["jump_power"])


                    smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")
                    smart_update_label(label_indicator_jump_height, f"Jump height: {round(data["jump_height"]*100,2)} cm", "black")

                    for k in info_load_cells: smart_update_label(label_indicator_jump_time[k], f"Jump time {k}: {round(data["jump_time"][k], 5)} s", "black")
                    smart_update_label(label_indicator_jump_time_AVG, f"Jump time AVG: {round(data["jump_time_AVG"], 5)} s", "black")

                    for k in info_load_cells: smart_update_label(label_indicator_first_touch[k], f"First touch {k}: {round(data["first_touch"][k], 5)} s", "black")

                    for k in info_load_cells: smart_update_label(label_indicator_jump_power[k], f"Jump power {k}: {round(data["jump_power"][k], 3)} N", "black")

                    smart_update_label(label_indicator_jump_power_AVG, f"Jump power AVG: {round(data["jump_power_AVG"], 3)} N", "black")
     


        self.start_representation(func1, (), func2, ())
    


    def test_calculate_mass(self):
        self.jump_data["mass"] = 0.0

        # --------------------------------------------
        clear_frame(frame_chart)

        fig, axis = plt.subplots(1, 1, figsize=(5, 2))
        fig.tight_layout()  # Improve the layout
        fig.subplots_adjust(hspace=0.50)  # Increase vertical space between charts

        # Creating empty lines
        line = axis.plot([], [])[0]

        # Axis configuration
        axis.set_title(f"Load Cell {k}")
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


        scrollbar = ttk.Scale(root, from_=0, to=0, orient="horizontal", command=lambda val: update_view(int(float(val))))
        scrollbar.pack(fill="x")

        automatic_scaling()
        # --------------------------------------------
        clear_frame(frame_indicator)

        label_indicator_number_samples = tk.Label(frame_indicator, text=f"Number of samples collected: {0}", font=("Arial", 12))
        label_indicator_number_samples.grid(row=0, column=0)
        label_indicator_mass = tk.Label(frame_indicator, text=f"Mass: {self.jump_data['mass']} Kg", font=("Arial", 12))
        label_indicator_mass.grid(row=1, column=0, pady=10)
    

        automatic_scaling()
        # --------------------------------------------


        def func1():
            smart_update_label(label_indicator_number_samples, f"Number of samples collected: {self.raw_data["time"].shape[0]}", "black")
            line.set_data(self.raw_data["time"], np.sum(self.raw_data["lc"].values(), axis=0))
            axis.relim()
            axis.autoscale_view()
            canvas.draw()



        def func2():
            if not (self.raw_data): return 

            if self.raw_data["time"] and self.raw_data["lc"]:
                self.jump_data["mass"] = np.mean(np.sum(self.raw_data["lc"].values(), axis=0))/self.jump_data["acceleration_of_gravity"]
                smart_update_label(label_indicator_mass, f"Mass: {round(self.jump_data["mass"],2)} Kg", "black")


        self.start_representation(func1, (), func2, ())
    

    def start_representation(self, func1, args1, func2, args2):
        smart_update_label(label_status_acquisition, "Initialization...", "black")
        self.force_stop_representation()
        data_acquirer.stop_acquisition()
        data_dq.clear()
        self.initialization_raw_data()
        data_acquirer.start_acquisition()
        self.fs_representation=False
        self.representation=True

        func = partial(self.run, func1, args1, func2, args2)
        self.after_id = root.after(self.time_sleep, func)

    def stop_representation(self):
        data_acquirer.stop_acquisition()
        self.representation=False

    def force_stop_representation(self):
        self.fs_representation=True
        self.stop_representation()

    def stop_representation_activity(self):
        try: root.after_cancel(self.after_id)
        except: pass
        self.force_stop_representation()
    
    def add_data(self, data):
        if self.raw_data and data:
            data_lc = data["lc"]
            if None not in data_lc["values"].values():
                if self.raw_data["size"]<self.max_number_of_samples:
                    if len(self.raw_data["time"])==0:  self.raw_data["initial_time"] = data_lc["time"]
                    self.raw_data["time"][self.raw_data["size"]] = data_lc["time"]-self.raw_data["initial_time"]
                    for k in info_load_cells: self.raw_data["lc"][k][self.raw_data["size"]] = data_lc["values"][k]
                    self.raw_data["size"] += 1
                else:
                    show_warning("Attention!", "You have reached the maximum number of saveable champions.")
                    self.force_stop_representation()






class DataAcquirer(threading.Thread):
    def __init__(self):
        super().__init__()

        self.acquisition_activity=True
        self.acquisition=False
        self.acquiring=threading.Lock()

        self.time_sleep_acquiring=0.0
        self.time_sleep_not_acquiring=0.3

        self._stop_acquisition()
    
    def run(self):
        while self.acquisition_activity:
            with self.acquiring:
                if self.acquisition:
                    try: data_dq.append(arduino.write_and_read({"command":"get_data"}))
                    except: 
                        data_dq.append(None)
                        self._stop_acquisition()

            if self.acquisition: time.sleep(self.time_sleep_acquiring)
            else: time.sleep(self.time_sleep_not_acquiring)


    def _start_acquisition(self):
        self.acquisition=True

    def start_acquisition(self):
        with self.acquiring:
            self._start_acquisition()

    def _stop_acquisition(self):
        self.acquisition=False

    def stop_acquisition(self):
        with self.acquiring: 
            self._stop_acquisition()

    def stop_acquisition_activity(self):
        self.acquisition_activity=False
        self._stop_acquisition()
    






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
    global arduino, data_jump
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
    for widget in frame.winfo_children(): widget.destroy()
    automatic_scaling()
    

def close_device_connection():
    smart_update_label(label_status_acquisition, "Acquisition stopped", "black")
    smart_update_label(label_status_connection, "Disconnecting...", "black")
    smart_update_label(label_scale_tare, "Not calibrated!", "red")
    data_processor.force_stop_representation()
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
            #info_device["baudrate"] = baudrate
            #open_device_connection(info_device, timeout)
            break
        
def check_device_connection():
    if arduino is not None and arduino.check_connection(): 
        smart_update_label(label_status_connection, "Connected", "green")
        return True

    smart_update_label(label_status_connection, "Not Connected", "red")
    return False

def command_button_test(test, args):
    if check_device_connection() and info_load_cells: test(*args)
    else: 
        close_device_connection()
        func_Focus_combobox_menu_device(None)

def command_button_stop_test():
    data_processor.stop_representation()

def func_WM_DELETE_WINDOW():
    data_processor.force_stop_representation()
    data_processor.stop_representation_activity()
    data_acquirer.stop_acquisition_activity()
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
            for i, k in enumerate(info_load_cells.keys()):
                result_string += f"{k}: {result[k]}"
                if i < len(info_load_cells)-1: result_string += " | "
            smart_update_label(label_scale_tare, result_string, "black")
        except:
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

info_load_cells = []
list_info_device = []

root = tk.Tk()
arduino = None
data_dq = deque()
data_acquirer = DataAcquirer()
data_processor = DataProcessor()


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

button_test_normal_jump = tk.Button(frame_acquisition, text="Normal jump", command=lambda: command_button_test(data_processor.test_normal_jump, ()))
button_test_normal_jump.grid(row=0, column=0)
button_test_calculate_mass = tk.Button(frame_acquisition, text="Calculate mass", command=lambda: command_button_test(data_processor.test_calculate_mass, ()))
button_test_calculate_mass.grid(row=0, column=1)
button_stop_test = tk.Button(frame_acquisition, text="Stop test", command=command_button_stop_test)
button_stop_test.grid(row=0, column=2)
label_status_acquisition = tk.Label(frame_acquisition, text=f"Acquisition status: Stop", font=("Arial", 15))
label_status_acquisition.grid(row=1, column=1, pady=10)

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