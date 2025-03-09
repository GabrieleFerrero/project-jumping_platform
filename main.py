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




class TimeoutException(Exception):
    """Custom exception for timeout"""
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
    

    def reset_buffer(self):
        with self.lock_IO:
            message = Message()
            message.set_action(f"reset_buffer [{self.info_device}]")

            try: 
                self.function_execution_with_timeout(self.device_timeout, self._reset_buffer)
                message.set_code(Code("OK"))
                message.set_code_description("Buffer empty")
            except Exception as e:
                message.set_code(Code("ERROR"))
                message.set_code_description(str(e))

            return message


    def check_connection(self):
        with self.lock_IO:
            if self.device_connection==None: return False
            else: 
                try: return self.function_execution_with_timeout(self.device_timeout, self._check_connection)
                except Exception as e: 
                    #print(f"{e}")
                    return False

    
    def close_connection(self):
        with self.lock_IO:
            message = Message()
            message.set_action(f"close_connection [{self.info_device}]")

            
            try: self.function_execution_with_timeout(self.device_timeout, self._close_connection)
            except: pass
            self.device_connection = None

            message.set_code(Code("OK"))
            message.set_code_description("Connection closed")

            return message
    
    def open_connection(self):
        with self.lock_IO:
            message = Message()
            message.set_action(f"open_connection [{self.info_device}]")

            message.add_recalled_actions(self.close_connection())

            if self.info_device:
                try:
                    self.device_connection = self.function_execution_with_timeout(self.device_timeout, self._open_connection)
                    message.add_recalled_actions(self.reset_buffer())

                    if self.check_connection():
                        message.set_code(Code("OK"))
                    else:
                        message.set_code(Code("ERROR"))
                        message.set_code_description("Connection failed")
                        
                except TimeoutException as e:
                    message.set_code(Code("ERROR"))
                    message.set_code_description("Timeout Error")
                except Exception as e:
                    message.set_code(Code("ERROR"))
                    message.set_code_description(str(e))
            else:
                message.set_code(Code("ERROR"))
                message.set_code_description("info_device is empty")

            return message
    

    
    def read(self):
        with self.lock_IO:
            message = Message()
            message.set_action(f"read [{self.info_device}]")

            try:
                result = self.function_execution_with_timeout(self.device_timeout, self._read)
                message.set_code(Code("OK"))
                message.set_response(result)
            except TimeoutException as e:
                message.set_code(Code("ERROR"))
                message.set_code_description("Timeout Error")
                message.set_response(None)
            except Exception as e:
                message.set_code(Code("ERROR"))
                message.set_code_description(f"{e}")
                message.set_response(None)

            return message

    
    def write(self, data):
        with self.lock_IO:
            message = Message()
            message.set_action(f"write [{self.info_device}]")

            try:
                result = self.function_execution_with_timeout(self.device_timeout, self._write, data)
                message.set_code(Code("OK"))
                message.set_response(result)
            except TimeoutException as e:
                message.set_code(Code("ERROR"))
                message.set_code_description("Timeout Error")
            except Exception as e:
                message.set_code(Code("ERROR"))
                message.set_code_description(f"{e}")

            return message
    
    def write_and_read(self, data):
        with self.lock_IO:
            message = Message()
            message.set_action(f"write_and_read [{self.info_device}]")
            
            message_send = self.write(data)
            message.add_recalled_actions(message_send)
            if message_send.get_code()==Code('OK'):
                message_read = self.read()
                message.add_recalled_actions(message_read)
                if message_read.get_code()==Code('OK'):
                    message.set_code(Code("OK"))
                    message.set_response(message_read.get_response())
                else:
                    message.set_code(Code("ERROR"))
                    message.set_response(None)
            else:
                message.set_code(Code("ERROR"))
                message.set_response(None)

            return message
    

class USBIODevice(IODevice):
    pass
    

class Arduino(USBIODevice):
    def __init__(self, info_device={}, device_timeout=5, baudrate=115200):
        super().__init__(info_device, device_timeout)

        self.baudrate=baudrate

    
    def _reset_buffer(self):
        self.device_connection.reset_input_buffer() # empty the buffer

    def _check_connection(self):
        if self.device_connection.is_open:
            message_to_send_to_arduino = {"command":"is_alive"}
            self._write(message_to_send_to_arduino)
            message_arduino_read = self._read()
            if message_arduino_read is not None:
                if message_arduino_read["code"]=="OK":
                    return True
                else:
                    return False
            else:
                False
        else:
            return False
    
    def _close_connection(self):
        self.device_connection.close()
    
    def _open_connection(self):
        if "port" in self.info_device and self.info_device["port"] and self.baudrate>0:
            device_connection=serial.Serial(port=self.info_device["port"], baudrate=self.baudrate, timeout=self.device_timeout+1)
            time.sleep(1)
            return device_connection
        else:
            return None

    
    def _read(self):

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
                exit_while=True
                response=None

        #print(response)
        if response is not None:
            try: 
                #print(json.loads(response))
                return json.loads(response) #.strip('\n\r')
            except: return None
        else: 
            return None
    
    def _write(self, data):
        self.device_connection.write(f"?{json.dumps(data)}!".encode())
        #print(f"?{json.dumps(data)}!")


class DataRepresenter():
    def __init__(self):

        self.after_id=None

        self.data_device = self.value_initialization_data_device()
        self.data_processed = self.value_initialization_data_processed()

        self.fs_representation=True # force stop representation
        self.representation=False        

        self.number_of_samples_for_mass_calculation=200

        self.timeout_empty_queue=1 # s (seconds)
        self.time_sleep=1 # ms (milliseconds)

        self.force_stop_representation()


    
    def run(self):
        error=False
        if not self.fs_representation:
            data, empty = self.get_data()
            if self.representation and not empty:
                error = not self.add_data(data)
                if not error:
                    #self.draw_chart()
                    if len(self.data_device["time"])<=self.number_of_samples_for_mass_calculation:
                        label_acquisition_status.config(text=f"Stay still! Mass calculation {int((len(self.data_device["time"])/self.number_of_samples_for_mass_calculation)*100)}%", fg="orange",  font=("Arial", 15))
                        root.update_idletasks()
                    else:
                        label_acquisition_status.config(text=f"Jump!", fg="green",  font=("Arial", 15))
                        root.update_idletasks() 
            elif not self.representation and not empty:
                error = not self.add_data(data)
                label_acquisition_status.config(text="Analysis in progress...", font=("Arial", 15), fg="purple")
                root.update_idletasks()
            elif self.representation and empty:
                label_acquisition_status.config(text="Waiting for data...", font=("Arial", 15), fg="blue")
                root.update_idletasks()
            elif not self.representation and empty:
                label_acquisition_status.config(text="Acquisition status: Stop", font=("Arial", 15), fg="black")
                root.update_idletasks()

                self.fs_representation=True
                self.data_processing()
                self.show_data_processed()
                self.save_data()
                self.show_data_device()

        if error:
            label_acquisition_status.config(text="Error in analysis!", font=("Arial", 15), fg="red")
            root.update_idletasks()
            if not check_device_connection():
                self.force_stop_representation()
                disconnect_device()
                func_Focus_combobox_menu_device(None)

        self.after_id = root.after(self.time_sleep, self.run)


    def value_initialization_data_device(self):
        data_device={
            "time":[],
            "jpLX":[],
            "jpRX":[]
        }

        return data_device

    def set_data_device(self, data_device):
        line_jpLX.set_data(data_device["time"], data_device["jpLX"])
        line_jpRX.set_data(data_device["time"], data_device["jpRX"])
        ax_jpLX.relim()
        ax_jpRX.relim()
        ax_jpLX.autoscale_view()
        ax_jpRX.autoscale_view()
        canvas.draw()

    def show_default_data_device(self):
        data_device = self.value_initialization_data_device()
        self.set_data_device(data_device)

    def show_data_device(self):
        self.set_data_device(self.data_device)

    def value_initialization_data_processed(self):
        data = {
            "mass": 0.0,
            "jump_height": 0.0,
            "jump_time_LX_value": 0.0,
            "jump_time_RX_value": 0.0,
            "jump_time_AVG_value": 0.0,
            "first_touch_LX_value": 0.0,
            "first_touch_RX_value": 0.0,
            "jump_power_LX_value": 0.0,
            "jump_power_RX_value": 0.0,
            "jump_power_AVG_value": 0.0
        }

        return {"data":data, "state":{"value":False, "color":"black"}, "date":datetime.now()}

    def set_processed_data(self, data_processed):
        label_indicator_mass.config(text=f"Mass: {round(data_processed["data"]["mass"],2)} Kg", fg=data_processed["state"]["color"])
        label_indicator_jump_height.config(text=f"Jump height: {round(data_processed["data"]["jump_height"],2)} cm", fg=data_processed["state"]["color"])
        label_indicator_jump_time_LX.config(text=f"Jump time LX: {round(data_processed["data"]["jump_time_LX_value"], 5)} s", fg=data_processed["state"]["color"])
        label_indicator_jump_time_RX.config(text=f"Jump time RX: {round(data_processed["data"]["jump_time_RX_value"], 5)} s", fg=data_processed["state"]["color"])
        label_indicator_jump_time_AVG.config(text=f"Jump time AVG: {round(data_processed["data"]["jump_time_AVG_value"], 5)} s", fg=data_processed["state"]["color"])
        label_indicator_first_touch_LX.config(text=f"First touch LX: {round(data_processed["data"]["first_touch_LX_value"], 5)} s", fg=data_processed["state"]["color"])
        label_indicator_first_touch_RX.config(text=f"First touch RX: {round(data_processed["data"]["first_touch_RX_value"], 5)} s", fg=data_processed["state"]["color"])
        label_indicator_jump_power_LX.config(text=f"Jump power LX: {round(data_processed["data"]["jump_power_LX_value"], 3)} N", fg=data_processed["state"]["color"])
        label_indicator_jump_power_RX.config(text=f"Jump power RX: {round(data_processed["data"]["jump_power_RX_value"], 3)} N", fg=data_processed["state"]["color"])
        label_indicator_jump_power_AVG.config(text=f"Jump power AVG: {round(data_processed["data"]["jump_power_AVG_value"], 3)} N", fg=data_processed["state"]["color"])
        root.update_idletasks()

    def data_processing(self):
        data_processed = self.value_initialization_data_processed()
        acceleration_of_gravity = 9.81

        if self.data_device["time"]:
            if len(self.data_device["time"])<self.number_of_samples_for_mass_calculation:
                data_processed["state"]["color"]="red"
                data_processed["data"]["mass"]=-1
            else:

                mass_LX = (sum(self.data_device["jpLX"][0:self.number_of_samples_for_mass_calculation])/self.number_of_samples_for_mass_calculation)/acceleration_of_gravity
                mass_RX = (sum(self.data_device["jpRX"][0:self.number_of_samples_for_mass_calculation])/self.number_of_samples_for_mass_calculation)/acceleration_of_gravity
                data_processed["data"]["mass"] = mass_LX+mass_RX


                first_touch_LX_index_start = next((i for i, strength_value in enumerate(self.data_device["jpLX"]) if strength_value!=0), None)
                first_touch_RX_index_start = next((i for i, strength_value in enumerate(self.data_device["jpRX"]) if strength_value!=0), None)
                first_touch_LX_index_end = next((i for i, strength_value in enumerate(self.data_device["jpLX"][::-1]) if strength_value!=0), None)
                first_touch_RX_index_end = next((i for i, strength_value in enumerate(self.data_device["jpRX"][::-1]) if strength_value!=0), None)

                if first_touch_RX_index_start is not None and first_touch_LX_index_start is not None:
                    first_leave_LX_value=self.data_device["time"][first_touch_LX_index_start]
                    first_leave_RX_value=self.data_device["time"][first_touch_RX_index_start]

                    if first_touch_RX_index_end is not None and first_touch_LX_index_end is not None:
                        data_processed["data"]["first_touch_LX_value"]=self.data_device["time"][first_touch_LX_index_end]
                        data_processed["data"]["first_touch_RX_value"]=self.data_device["time"][first_touch_RX_index_end]

                        data_processed["data"]["jump_time_LX_value"]=data_processed["data"]["first_touch_LX_value"]-first_leave_LX_value
                        data_processed["data"]["jump_time_RX_value"]=data_processed["data"]["first_touch_RX_value"]-first_leave_RX_value
                        data_processed["data"]["jump_time_AVG_value"]=(data_processed["data"]["jump_time_LX_value"]+data_processed["data"]["jump_time_RX_value"])/2

                        data_processed["data"]["jump_height"]=((1/2)*acceleration_of_gravity*((data_processed["data"]["jump_time_AVG_value"]/2)**2))*100 # cm

                        data_processed["data"]["jump_power_LX_value"] = (data_processed["data"]["mass"]*acceleration_of_gravity*data_processed["data"]["jump_height"])/(data_processed["data"]["jump_time_LX_value"]/2)
                        data_processed["data"]["jump_power_RX_value"] = (data_processed["data"]["mass"]*acceleration_of_gravity*data_processed["data"]["jump_height"])/(data_processed["data"]["jump_time_RX_value"]/2)
                        data_processed["data"]["jump_power_AVG_value"]=(data_processed["data"]["jump_power_LX_value"]+data_processed["data"]["jump_power_RX_value"])/2

                data_processed["state"]["value"]=True

        else:
            data_processed["state"]["color"]="red"

        self.data_processed=data_processed
   

    def show_default_data_processed(self):
        data_processed = self.value_initialization_data_processed()
        self.set_processed_data(data_processed)

    def show_data_processed(self):
        self.set_processed_data(self.data_processed)

    def initialization_data(self):
        self.data_device = self.value_initialization_data_device()
        self.data_processed = self.value_initialization_data_processed()

    def data_is_saved(self):
        if self.data_processed["state"]["value"]:
            if data_saving_file_path:    
                try:
                    if os.path.exists(data_saving_file_path):
                        df_old = pd.read_csv(data_saving_file_path)
                        if (not df_old.empty) and ("date" in df_old.columns) and (self.data_processed["date"].strftime('%Y-%m-%d %H:%M:%S') in df_old["date"].values): return True
                except: pass
        else: return True # it means that the data does not need to be saved, so I consider it as if it were already saved
        
        return False

    
    def save_data(self):
        if self.data_processed["state"]["value"]:
            if data_saving_file_path:    
                try:
                    df_to_save = pd.DataFrame([{"date":self.data_processed["date"].strftime('%Y-%m-%d %H:%M:%S'), **self.data_processed["data"]}])

                    if os.path.exists(data_saving_file_path):
                        try: df_old = pd.read_csv(data_saving_file_path)
                        except pd.errors.EmptyDataError: df_old = pd.DataFrame()

                        if (df_old.empty) or ("date" not in df_old.columns):
                            df_to_save.to_csv(data_saving_file_path, index=False, header=True)
                        
                        elif df_to_save["date"].iloc[0] not in df_old["date"].values:
                            df = pd.concat([df_old, df_to_save], ignore_index=True)
                            df.to_csv(data_saving_file_path, index=False, header=True)
                        else:
                            #print("Data already present, not added.")
                            pass
                    else:
                        df_to_save.to_csv(data_saving_file_path, index=False, header=True)
                    
                    return True
                except: show_warning("Warning", "Problems saving data!")
            else: show_warning("Warning", "File not selected for saving! Please choose a file.")
        else: return True # it means that the data does not need to be saved, so I consider it as if it were already saved
        
        return False

                
    
    def _start_representation(self):
        self.force_stop_representation()
        # The acquisition is at a standstill:

        # flag management
        self.fs_representation=False
        self.representation=True
        
        
        # empty the queue
        data_acquirer.stop_acquisition()
        while not self.get_data()[1]: pass
        data_acquirer.start_acquisition()
        
        self.after_id = root.after(self.time_sleep, self.run)

    def start_representation(self):
        start=False
        if not self.data_is_saved():
            confirm = confirm_unsaved_data()
            if (confirm is not None) and confirm:
                start=True
            else:
                start=False
        else:
            start=True

        if start:
            label_acquisition_status.config(text="Initialization...", font=("Arial", 15), fg="black")
            root.update_idletasks()
            self.initialization_data()
            self.show_default_data_processed()
            self.show_default_data_device()
            self._start_representation()

    def _stop_representation(self):
        data_acquirer.stop_acquisition()
        self.representation=False

    def stop_representation(self):
        self._stop_representation()

    def force_stop_representation(self):
        self.fs_representation=True
        self._stop_representation()

    def stop_representation_activity(self):
        try: root.after_cancel(self.after_id)
        except: pass
        self.force_stop_representation()

    def get_data(self):
        try: return (data_queue.get(timeout=self.timeout_empty_queue), False)
        except: return (None, True)
    
    def add_data(self, data):
        if data is not None and data[0] is not None:
            if data[0]["code"]=="OK" and data[0]["jpLX"] is not None and data[0]["jpRX"] is not None:
                self.data_device["time"].append(data[1])
                self.data_device["jpLX"].append(data[0]["jpLX"])
                self.data_device["jpRX"].append(data[0]["jpRX"])
            return True
        else: return False
            
    





class DataAcquirer(threading.Thread):
    def __init__(self, device, data_queue):
        super().__init__()

        self.device=device
        self.data_queue=data_queue

        self.acquisition_activity=True
        self.acquisition=False
        self.acquiring=threading.Lock()

        self.acquisition_start_time=time.time()

        self.time_sleep_acquiring=0.0001
        self.time_sleep_not_acquiring=0.3

        self._stop_acquisition()
    
    def run(self):
        while self.acquisition_activity:
            with self.acquiring:
                error=False
                if self.acquisition:
                    data = self.get_data()

                    if data is None: error=True

                    self.data_queue.put((data, time.time()-self.acquisition_start_time))

                if error: self._stop_acquisition()

            if self.acquisition:
                time.sleep(self.time_sleep_acquiring)
            else:
                time.sleep(self.time_sleep_not_acquiring)


    def _start_acquisition(self):
        self.acquisition=True

    def start_acquisition(self):
        with self.acquiring: 
            self.acquisition_start_time = time.time()
            self._start_acquisition()

    def _stop_acquisition(self):
        self.acquisition=False

    def stop_acquisition(self):
        with self.acquiring: 
            self._stop_acquisition()

    def stop_acquisition_activity(self):
        self.acquisition_activity=False
        self._stop_acquisition()
    
    def get_data(self):
        message_to_send_to_device = {"command":"get_data"}
        message_device = self.device.write_and_read(message_to_send_to_device)
        if message_device.get_code()==Code('OK'): 
            #print(message_device.get_response())
            return message_device.get_response()
        else: return None












def get_usb_devices():
    device_list = []
    valid_devices = []
    devices_found = list_ports.comports()
    for device in devices_found: 
        if device.vid is not None and device.pid is not None and device.device: valid_devices.append(device)

    if valid_devices:
        for i, device in enumerate(valid_devices):
            info_device = {"port": device.device}
            device_list.append(info_device)
    else:
        #print("No valid USB device found.")
        pass

    return device_list

def connect_device(info_device):
    disconnect_device()
    label_status_connection.config(text="Connecting...", fg="orange")
    root.update_idletasks() 
    arduino.info_device=info_device
    arduino.open_connection()
    if check_device_connection():
        label_acquisition_status.config(text="Acquisition status: Stop", font=("Arial", 15), fg="black")
        root.update_idletasks()
        command_button_scale_tare()
    else:
        disconnect_device()
        func_Focus_combobox_menu_device(None)

def disconnect_device():
    label_status_connection.config(text="Disconnecting...", fg="orange")
    label_scale_tare.config(text="Not calibrated!", fg="red")
    root.update_idletasks() 
    data_representer.force_stop_representation()
    data_representer.show_default_data_processed()
    data_representer.show_default_data_device()
    arduino.close_connection()
    check_device_connection()

def func_Focus_combobox_menu_device(event):
    data_device = get_usb_devices()
    if data_device: combobox_menu_device['values'] = [device["port"] for device in data_device]
    else:
        combobox_menu_device['values'] = []
        combobox_menu_device.set('')  # Removes selected text
    root.update_idletasks() 
    return len(data_device)
    
def func_ComboboxSelected_combobox_menu_device(event):
    selected = stringvar_menu_device.get()
    info_device = {"port":selected}
    connect_device(info_device)
        
def check_device_connection():
    status_connection=True
    if arduino.check_connection(): 
        label_status_connection.config(text="Connected", fg="green")
        status_connection=True
    else: 
        label_status_connection.config(text="Not Connected", fg="red")
        status_connection=False
    root.update_idletasks() 
    return status_connection

def command_button_start():
    if check_device_connection(): data_representer.start_representation()
    else: 
        disconnect_device()
        func_Focus_combobox_menu_device(None)

def command_button_stop():
    data_representer.stop_representation()

def func_WM_DELETE_WINDOW():
    data_representer.stop_representation_activity()
    data_acquirer.stop_acquisition_activity()

    finish=False
    if not data_representer.data_is_saved():
        confirm = confirm_unsaved_data()
        if (confirm is not None) and confirm:
            finish=True
        else:
            finish=False
    else:
        finish=True
    
    if finish:
        data_acquirer.join()
        disconnect_device()
        root.quit()
        root.destroy()

def command_button_scale_tare():
    label_scale_tare.config(text="Calibration...", fg="black")
    root.update_idletasks() 
    if check_device_connection():
        message_to_send_to_device = {"command":"scale_tare"}
        message_device = arduino.write_and_read(message_to_send_to_device)
        if message_device.get_code()==Code('OK') and message_device.get_response() is not None: 
            result = message_device.get_response()
            if result["jpLX"]=="OK" and result["jpRX"]=="OK": 
                label_scale_tare.config(text="Calibrated", fg="green")
            elif result["jpLX"]=="OK" and result["jpRX"]=="ERROR":
                label_scale_tare.config(text="LX:yes | RX:no", fg="purple")
            elif result["jpLX"]=="ERROR" and result["jpRX"]=="OK":
                label_scale_tare.config(text="LX:no | RX:yes", fg="purple")
            elif result["jpLX"]=="ERROR" and result["jpRX"]=="ERROR":
                label_scale_tare.config(text="Not calibrated!", fg="red")
        else: 
            label_scale_tare.config(text="Not calibrated!", fg="red")
        root.update_idletasks() 
    else: 
        disconnect_device()
        func_Focus_combobox_menu_device(None)

def update_label_open_file():
    label_open_file.config(text=f"Open file: {data_saving_file_path}", fg="black")
    root.update_idletasks() 

def open_file():
    """Allows the user to select an existing file."""
    global data_saving_file_path
    file_path = filedialog.askopenfilename(title="Select a CSV file", filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")])
    if file_path: data_saving_file_path=file_path
    update_label_open_file()


def new_file():
    """Allows the user to choose a name and folder for a new file."""
    global data_saving_file_path
    file_path = filedialog.asksaveasfilename(title="Create a new CSV file",
                                             defaultextension=".csv",
                                             filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")])
    
    if file_path: data_saving_file_path=file_path
    update_label_open_file()


def command_button_save():
    data_representer.save_data()

def confirm_unsaved_data():
    """Opens a dialog asking the user whether to save before proceeding.
    
    Returns:
        True if "Save" is selected,
        False if "Don't Save" is selected,
        None if "Cancel" is selected.
    """
    root = tk.Tk()
    root.withdraw()  # Hide the main window
    response = messagebox.askyesnocancel("Unsaved Changes", "Are you sure you want to proceed without saving?")
    return response 
                

    
def show_warning(title, message):
    root = tk.Tk()
    root.withdraw()
    messagebox.showwarning(title, message)

# ----------------------

root = tk.Tk()
arduino = Arduino(device_timeout=2, baudrate=460800)
data_queue=queue.Queue()
data_acquirer = DataAcquirer(arduino, data_queue)
data_representer = DataRepresenter()
data_saving_file_path = ""

# Creation of main window
root.title("Jumping platform")
root.geometry("1000x1000")
root.protocol("WM_DELETE_WINDOW", func_WM_DELETE_WINDOW)

menu_bar = tk.Menu(root)
file_menu = tk.Menu(menu_bar, tearoff=0)
file_menu.add_command(label="New", command=new_file)
file_menu.add_command(label="Open", command=open_file)
menu_bar.add_cascade(label="File", menu=file_menu)
root.config(menu=menu_bar)

frame_open_file = tk.Frame(root)
frame_open_file.pack(fill="x")
label_open_file = tk.Label(frame_open_file, text=f"Open file: {data_saving_file_path}", padx=10)
label_open_file.pack(side=tk.LEFT, padx=10)

# Device Selection and Status
frame_menu_device = tk.Frame(root)
frame_menu_device.pack(pady=10)

tk.Label(frame_menu_device, text="Select Device:").pack(side=tk.LEFT)
stringvar_menu_device = tk.StringVar()
combobox_menu_device = ttk.Combobox(frame_menu_device, textvariable=stringvar_menu_device, state="readonly")
combobox_menu_device.pack(side=tk.LEFT, padx=10)
combobox_menu_device.bind("<<ComboboxSelected>>", func_ComboboxSelected_combobox_menu_device)
combobox_menu_device.bind("<FocusIn>", func_Focus_combobox_menu_device)
combobox_menu_device.bind("<FocusOut>", func_Focus_combobox_menu_device)

label_status_connection = tk.Label(frame_menu_device, text="Not Connected", fg="red")
label_status_connection.pack(side=tk.LEFT, padx=10)

func_Focus_combobox_menu_device(None)

# Scale tare
frame_scale_tare = tk.Frame(root)
frame_scale_tare.pack(pady=10)
button_scale_tare = tk.Button(frame_scale_tare, text="Tare", command=command_button_scale_tare)
button_scale_tare.pack(side=tk.LEFT, padx=10)
label_scale_tare = tk.Label(frame_scale_tare, text=f"Not calibrated!", font=("Arial", 12), fg="red")
label_scale_tare.pack(side=tk.LEFT, padx=20)

# Button section
frame_acquisition = tk.Frame(root)
frame_acquisition.pack(pady=10)

button_start = tk.Button(frame_acquisition, text="Start", command=command_button_start)
button_start.pack(side=tk.LEFT, padx=10)
button_stop = tk.Button(frame_acquisition, text="Stop", command=command_button_stop)
button_stop.pack(side=tk.LEFT, padx=10)
button_save = tk.Button(frame_acquisition, text="Save", command=command_button_save)
button_save.pack(side=tk.LEFT, padx=10)


# Indicators section
frame_acquisition_status = tk.LabelFrame(root)
frame_acquisition_status.pack(pady=10)
label_acquisition_status = tk.Label(frame_acquisition_status, text=f"Acquisition status: Stop", font=("Arial", 15))
label_acquisition_status.pack(side=tk.LEFT, padx=20)

frame_indicator_general_data = tk.LabelFrame(root, text=f"General data", padx=10, pady=10)
frame_indicator_general_data.pack(pady=10, padx=10, fill="both")
label_indicator_mass = tk.Label(frame_indicator_general_data, text=f"Mass: {0.0} Kg", font=("Arial", 12))
label_indicator_mass.pack(side=tk.LEFT, padx=20)
label_indicator_jump_height = tk.Label(frame_indicator_general_data, text=f"Jump height: {0.0} cm", font=("Arial", 12))
label_indicator_jump_height.pack(side=tk.LEFT, padx=20)

frame_indicator_jump_time = tk.LabelFrame(root, text=f"Jump time", padx=10, pady=10)
frame_indicator_jump_time.pack(pady=10, padx=10, fill="both")
label_indicator_jump_time_LX = tk.Label(frame_indicator_jump_time, text=f"Jump time LX: {0.0} s", font=("Arial", 12))
label_indicator_jump_time_LX.pack(side=tk.LEFT, padx=20)
label_indicator_jump_time_RX = tk.Label(frame_indicator_jump_time, text=f"Jump time RX: {0.0} s", font=("Arial", 12))
label_indicator_jump_time_RX.pack(side=tk.LEFT, padx=20)
label_indicator_jump_time_AVG = tk.Label(frame_indicator_jump_time, text=f"Jump time AVG: {0.0} s", font=("Arial", 12))
label_indicator_jump_time_AVG.pack(side=tk.LEFT, padx=20)

frame_indicator_first_touch = tk.LabelFrame(root, text=f"First touch", padx=10, pady=10)
frame_indicator_first_touch.pack(pady=10, padx=10, fill="both")
label_indicator_first_touch_LX = tk.Label(frame_indicator_first_touch, text=f"First touch LX: {0.0} s", font=("Arial", 12))
label_indicator_first_touch_LX.pack(side=tk.LEFT, padx=20)
label_indicator_first_touch_RX = tk.Label(frame_indicator_first_touch, text=f"First touch RX: {0.0} s", font=("Arial", 12))
label_indicator_first_touch_RX.pack(side=tk.LEFT, padx=20)

frame_indicator_jump_power = tk.LabelFrame(root, text=f"Jump power", padx=10, pady=10)
frame_indicator_jump_power.pack(pady=10, padx=10, fill="both")
label_indicator_jump_power_LX = tk.Label(frame_indicator_jump_power, text=f"Jump power LX: {0.0} N", font=("Arial", 12))
label_indicator_jump_power_LX.pack(side=tk.LEFT, padx=20)
label_indicator_jump_power_RX = tk.Label(frame_indicator_jump_power, text=f"Jump power RX: {0.0} N", font=("Arial", 12))
label_indicator_jump_power_RX.pack(side=tk.LEFT, padx=20)
label_indicator_jump_power_AVG = tk.Label(frame_indicator_jump_power, text=f"Jump power AVG: {0.0} N", font=("Arial", 12))
label_indicator_jump_power_AVG.pack(side=tk.LEFT, padx=20)

# Chart section
frame_chart = tk.Frame(root)
frame_chart.pack(fill=tk.BOTH, expand=True)

# Creating the figure and axes
fig, (ax_jpLX, ax_jpRX) = plt.subplots(2, 1, figsize=(5, 3))
fig.tight_layout()  # Improve the layout
fig.subplots_adjust(hspace=0.5)  # Increase vertical space between charts

# Creation of empty lines
line_jpLX, = ax_jpLX.plot([], [])
line_jpRX, = ax_jpRX.plot([], [])

# Axis configuration
ax_jpLX.set_title("Left Footrest")
ax_jpRX.set_title("Right Footrest")
ax_jpLX.set_xlabel("Time (s)")
ax_jpRX.set_xlabel("Time (s)")
ax_jpLX.set_ylabel("Newton (N)")
ax_jpRX.set_ylabel("Newton (N)")

# Tkinter Integration
canvas = FigureCanvasTkAgg(fig, master=frame_chart)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
canvas.draw()

# Adding the toolbar above the chart
toolbar = NavigationToolbar2Tk(canvas, frame_chart)
toolbar.pack(side=tk.TOP, fill=tk.X)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Starting data acquirer
data_acquirer.start()

# Starting root mainloop
root.mainloop()

