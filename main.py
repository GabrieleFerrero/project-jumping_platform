import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import serial
import serial.tools.list_ports
from time import sleep
from datetime import datetime
import json
from abc import ABC, abstractmethod
import threading
import time
import queue




class TimeoutException(Exception):
    """Eccezione personalizzata per il timeout"""
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

        self.lock_IO = threading.Lock()

    def function_execution_with_timeout(self, timeout, func, *args, **kwargs):
        """
        Wrapper che esegue una funzione con un limite di tempo.
        :param timeout: Tempo massimo di esecuzione in secondi
        :param func: La funzione da eseguire
        :param args: Argomenti posizionali della funzione
        :param kwargs: Argomenti keyword della funzione
        :return: Il valore di ritorno della funzione se eseguita in tempo
        :raises TimeoutException: Se la funzione impiega troppo tempo
        """

        #print(f"Function: {func}")

        # Funzione per eseguire la funzione target
        def target(q, *args, **kwargs):
            try:
                result = func(*args, **kwargs)
                q.put(result)  # Mette il risultato nella coda
            except Exception as e:
                q.put(e)

        # Creiamo una coda per ottenere il risultato dalla funzione
        q = queue.Queue()
        
        # Eseguiamo la funzione in un thread separato
        thread = threading.Thread(target=target, args=(q, *args), kwargs=kwargs)
        thread.start()

        # Aspettiamo che la funzione finisca o che il timeout scada
        thread.join(timeout)

        if thread.is_alive():
            thread.join()
            raise TimeoutException(f"Function execution exceeded {timeout} seconds.")

        if not q.empty():
            result = q.get()
            if isinstance(result, Exception):
                raise result
            return result
        return None  # Nel caso in cui la funzione non restituisca nulla

    

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
    

    def reset_buffer(self, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"reset_buffer [{self.info_device}]")

        try: 
            with self.lock_IO:
                self.function_execution_with_timeout(self.device_timeout, self._reset_buffer)
            message.set_code(Code("OK"))
            message.set_code_description("Buffer empty")
        except Exception as e:
            message.set_code(Code("ERROR"))
            message.set_code_description(str(e))

        return message


    def check_connection(self, automatic_error_resolution=False):
        with self.lock_IO:
            if self.device_connection==None: return False
            else: 
                try: return self.function_execution_with_timeout(self.device_timeout, self._check_connection)
                except Exception as e: 
                    #print(f"{e}")
                    return False

    
    def close_connection(self, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"close_connection [{self.info_device}]")

        with self.lock_IO:
            try: self.function_execution_with_timeout(self.device_timeout, self._close_connection)
            except: pass
            self.device_connection = None

        message.set_code(Code("OK"))
        message.set_code_description("Connection closed")

        return message
    
    def open_connection(self, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"open_connection [{self.info_device}]")

        message.add_recalled_actions(self.close_connection(automatic_error_resolution=automatic_error_resolution))

        if self.info_device:
            try:
                with self.lock_IO:
                    self.device_connection = self.function_execution_with_timeout(self.device_timeout, self._open_connection)
                
                message.add_recalled_actions(self.reset_buffer(automatic_error_resolution=automatic_error_resolution))

                if self.check_connection(automatic_error_resolution=automatic_error_resolution):
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
    

    def error_resolution(self, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"error_resolution [{self.info_device}]")
        message.set_code(Code("OK"))

        if automatic_error_resolution:
            if not self.check_connection(automatic_error_resolution=automatic_error_resolution):
                message.add_recalled_actions(self.open_connection(automatic_error_resolution=automatic_error_resolution))
        

        return message
    

    
    def read(self, locked=True, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"read [{self.info_device}]")

        try:
            if locked: self.lock_IO.acquire()
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
        finally:
            if locked: self.lock_IO.release()

        if message.get_code() == Code("ERROR"):
            message.add_recalled_actions(self.error_resolution(automatic_error_resolution=automatic_error_resolution))

        return message

    
    def write(self, data, locked=True, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"write [{self.info_device}]")

        try:
            if locked: self.lock_IO.acquire()
            result = self.function_execution_with_timeout(self.device_timeout, self._write, data)
            message.set_code(Code("OK"))
            message.set_response(result)
        except TimeoutException as e:
            message.set_code(Code("ERROR"))
            message.set_code_description("Timeout Error")
        except Exception as e:
            message.set_code(Code("ERROR"))
            message.set_code_description(f"{e}")
        finally:
            if locked: self.lock_IO.release()

        if message.get_code() == Code("ERROR"):
            message.add_recalled_actions(self.error_resolution(automatic_error_resolution=automatic_error_resolution))

        return message
    
    def write_and_read(self, data, automatic_error_resolution=False):
        message = Message()
        message.set_action(f"write_and_read [{self.info_device}]")

        with self.lock_IO:
            message_send = self.write(data, locked=False, automatic_error_resolution=False)
            message.add_recalled_actions(message_send)
            if message_send.get_code()==Code('OK'):
                message_read = self.read(locked=False, automatic_error_resolution=False)
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
        
        if message.get_code() == Code("ERROR"):
            message.add_recalled_actions(self.error_resolution(automatic_error_resolution=automatic_error_resolution))

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
            sleep(1)
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
    def __init__(self, device, data_queue, data_acquirer, root):

        self.device=device
        self.data_queue=data_queue
        self.data_acquirer=data_acquirer
        self.root=root

        self.after_id=None

        self.data_jpLX = []
        self.data_jpRX = []
        self.data_time = []

        self.fs_representation=True
        self.enable_representation=False
        self.representing=threading.Lock()

        self.number_of_samples_for_mass_calculation=200

        self.timeout_empty_queue=1
        self.time_sleep=1 # ms (milliseconds)

        self.force_stop_representation()


    
    def run(self):
        error=False
        with self.representing:
            if not self.fs_representation:
                data, empty = self.get_data()
                if self.enable_representation and not empty:
                    error = not self.add_data(data)
                    if not error:
                        #self.draw_graph()
                        if len(self.data_time)<=self.number_of_samples_for_mass_calculation:
                            label_acquisition_status.config(text=f"Stay still! Mass calculation {int((len(self.data_time)/self.number_of_samples_for_mass_calculation)*100)}%", fg="orange",  font=("Arial", 15))
                            root.update_idletasks()
                        else:
                            label_acquisition_status.config(text=f"Jump!", fg="green",  font=("Arial", 15))
                            root.update_idletasks() 
                elif not self.enable_representation and not empty:
                    error = not self.add_data(data)
                    label_acquisition_status.config(text="Analysis in progress...", font=("Arial", 15), fg="purple")
                    root.update_idletasks()
                elif self.enable_representation and empty:
                    label_acquisition_status.config(text="Waiting for data...", font=("Arial", 15), fg="blue")
                    root.update_idletasks()
                elif not self.enable_representation and empty:
                    label_acquisition_status.config(text="Acquisition status: Stop", font=("Arial", 15), fg="black")
                    root.update_idletasks()
                    self.extrapolate_data()
                    self.draw_graph()

        if error:
            label_acquisition_status.config(text="Error in analysis!", font=("Arial", 15), fg="red")
            root.update_idletasks()
            self.force_stop_representation()
            if not check_device_connection():
                disconnect_device()
                update_device_list(None)

        self.after_id = self.root.after(self.time_sleep, self.run)



    def set_graph(self, data_time, data_jpLX, data_jpRX):
        line_jpLX.set_data(data_time, data_jpLX)
        line_jpRX.set_data(data_time, data_jpRX)
        ax_jpLX.relim()
        ax_jpRX.relim()
        ax_jpLX.autoscale_view()
        ax_jpRX.autoscale_view()
        canvas.draw()

    def draw_default_graph(self):
        self.set_graph([], [], [])

    def draw_graph(self):
        self.set_graph(self.data_time, self.data_jpLX, self.data_jpRX)

    def set_extrapolate_data(self, e_data):
        label_indicator_mass.config(text=f"Mass: {round(e_data["mass"],2)} Kg", fg=e_data["status_color"])
        label_indicator_jump_time_LX.config(text=f"Jump time LX: {e_data["jump_time_LX_value"]} s", fg=e_data["status_color"])
        label_indicator_jump_time_RX.config(text=f"Jump time RX: {e_data["jump_time_RX_value"]} s", fg=e_data["status_color"])
        label_indicator_jump_time_AVG.config(text=f"Jump time AVG: {e_data["jump_time_AVG_value"]} s", fg=e_data["status_color"])
        label_indicator_first_touch_LX.config(text=f"First touch LX: {e_data["first_touch_LX_value"]} s", fg=e_data["status_color"])
        label_indicator_first_touch_RX.config(text=f"First touch RX: {e_data["first_touch_RX_value"]} s", fg=e_data["status_color"])
        label_indicator_expressed_strength_LX.config(text=f"Expressed strength LX: {e_data["expressed_strength_LX_value"]} N", fg=e_data["status_color"])
        label_indicator_expressed_strength_RX.config(text=f"Expressed strength RX: {e_data["expressed_strength_RX_value"]} N", fg=e_data["status_color"])
        label_indicator_expressed_strength_AVG.config(text=f"Expressed strength AVG: {e_data["expressed_strength_AVG_value"]} N", fg=e_data["status_color"])
        root.update_idletasks()

    def default_extrapolate_data(self):
        de_data = {
            "status_color": "black",
            "mass": 0.0,
            "jump_time_LX_value": 0.0,
            "jump_time_RX_value": 0.0,
            "jump_time_AVG_value": 0.0,
            "first_touch_LX_value": 0.0,
            "first_touch_RX_value": 0.0,
            "expressed_strength_LX_value": 0.0,
            "expressed_strength_RX_value": 0.0,
            "expressed_strength_AVG_value": 0.0
            
        }
        self.set_extrapolate_data(de_data)

    def extrapolate_data(self):
        e_data = {
            "status_color": "black",
            "mass": 0.0,
            "jump_time_LX_value": 0.0,
            "jump_time_RX_value": 0.0,
            "jump_time_AVG_value": 0.0,
            "first_touch_LX_value": 0.0,
            "first_touch_RX_value": 0.0,
            "expressed_strength_LX_value": 0.0,
            "expressed_strength_RX_value": 0.0,
            "expressed_strength_AVG_value": 0.0
        }

        if self.data_time:
            if len(self.data_time)<self.number_of_samples_for_mass_calculation:
                e_data["status_color"]="red"
                e_data["mass"]=-1
            else:

                mass_LX = (sum(self.data_jpLX[0:self.number_of_samples_for_mass_calculation])/self.number_of_samples_for_mass_calculation)/9.81
                mass_RX = (sum(self.data_jpRX[0:self.number_of_samples_for_mass_calculation])/self.number_of_samples_for_mass_calculation)/9.81
                e_data["mass"] = mass_LX+mass_RX


                first_touch_LX_index_start = next((i for i, strength_value in enumerate(self.data_jpLX) if strength_value!=0), None)
                first_touch_RX_index_start = next((i for i, strength_value in enumerate(self.data_jpRX) if strength_value!=0), None)
                first_touch_LX_index_end = next((i for i, strength_value in enumerate(self.data_jpLX[::-1]) if strength_value!=0), None)
                first_touch_RX_index_end = next((i for i, strength_value in enumerate(self.data_jpRX[::-1]) if strength_value!=0), None)

                if first_touch_RX_index_start is not None and first_touch_LX_index_start is not None:
                    first_leave_LX_value=self.data_time[first_touch_LX_index_start]
                    first_leave_RX_value=self.data_time[first_touch_RX_index_start]

                    if first_touch_RX_index_end is not None and first_touch_LX_index_end is not None:
                        e_data["first_touch_LX_value"]=self.data_time[first_touch_LX_index_end]
                        e_data["first_touch_RX_value"]=self.data_time[first_touch_RX_index_end]

                        e_data["jump_time_LX_value"]=e_data["first_touch_LX_value"]-first_leave_LX_value
                        e_data["jump_time_RX_value"]=e_data["first_touch_RX_value"]-first_leave_RX_value
                        e_data["jump_time_AVG_value"]=(e_data["jump_time_LX_value"]+e_data["jump_time_RX_value"])/2

                    
                    e_data["expressed_strength_AVG_value"]=(e_data["expressed_strength_LX_value"]+e_data["expressed_strength_RX_value"])/2

        else:
            e_data["status_color"]="red"

        self.set_extrapolate_data(e_data)

    
    def _start_representation(self):
        self.force_stop_representation()
        with self.representing:
            self.fs_representation=False
            self.data_acquirer.stop_acquisition()
            while not self.get_data()[1]: pass
            self.data_acquirer.start_acquisition()
            self.enable_representation=True
        self.after_id = self.root.after(self.time_sleep, self.run)

    def start_representation(self):
        self.data_jpLX = []
        self.data_jpRX = []
        self.data_time = []
        self.default_extrapolate_data()
        self.draw_default_graph()
        self._start_representation()

    def _stop_representation(self):
        self.data_acquirer.stop_acquisition()
        with self.representing: self.enable_representation=False

    def stop_representation(self):
        self._stop_representation()

    def force_stop_representation(self):
        self.fs_representation=True
        self._stop_representation()

    def stop_representation_activity(self):
        try: self.root.after_cancel(self.after_id)
        except: pass
        self.force_stop_representation()

    def get_data(self):
        try: return (self.data_queue.get(timeout=self.timeout_empty_queue), False)
        except: return (None, True)
    
    def add_data(self, data):
        if data is not None and data[0] is not None:
            if data[0]["code"]=="OK" and data[0]["jpLX"] is not None and data[0]["jpRX"] is not None:
                self.data_time.append(data[1])
                self.data_jpLX.append(data[0]["jpLX"])
                self.data_jpRX.append(data[0]["jpRX"])
            return True
        else: return False
            
    





class DataAcquirer(threading.Thread):
    def __init__(self, device, data_queue):
        super().__init__()

        self.device=device
        self.data_queue=data_queue

        self.acquisition_activity=True
        self.enable_acquisition=False
        self.acquiring=threading.Lock()

        self.acquisition_start_time=datetime.now()

        self.time_sleep=0.0001

        self._stop_acquisition()
    
    def run(self):
        while self.acquisition_activity:
            error=False
            with self.acquiring:
                if self.enable_acquisition:
                    data = self.get_data()

                    if data is None: error=True

                    self.data_queue.put((data, (datetime.now()-self.acquisition_start_time).total_seconds()))

            if error: self._stop_acquisition()
            time.sleep(self.time_sleep)

    def _start_acquisition(self):
        with self.acquiring: self.enable_acquisition=True

    def start_acquisition(self):
        self.acquisition_start_time = datetime.now()
        self._start_acquisition()

    def _stop_acquisition(self):
        with self.acquiring: self.enable_acquisition=False

    def stop_acquisition(self):
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
    devices_found = serial.tools.list_ports.comports()
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
        scale_tare()
    else:
        disconnect_device()
        update_device_list(None)

def disconnect_device():
    label_status_connection.config(text="Disconnecting...", fg="orange")
    label_scale_tare.config(text="Not calibrated!", fg="red")
    root.update_idletasks() 
    data_representer.force_stop_representation()
    data_representer.default_extrapolate_data()
    data_representer.draw_default_graph()
    arduino.close_connection()
    check_device_connection()

def update_device_list(event):
    data_device = get_usb_devices()
    if data_device: combobox_menu_device['values'] = [device["port"] for device in data_device]
    else:
        combobox_menu_device['values'] = []
        combobox_menu_device.set('')  # Removes selected text
    root.update_idletasks() 
    return len(data_device)
    
def on_device_selected(event):
    selected = device_var.get()
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

def start_acquisition():
    if check_device_connection():
        label_acquisition_status.config(text="Initialization...", font=("Arial", 15), fg="black")
        root.update_idletasks()
        data_representer.start_representation()
    else: 
        disconnect_device()
        update_device_list(None)

def stop_acquisition():
    data_representer.stop_representation()

def on_close():
    data_representer.stop_representation_activity()
    data_acquirer.stop_acquisition_activity()
    data_acquirer.join()
    disconnect_device()
    root.quit()
    root.destroy()

def scale_tare():
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
        update_device_list(None)
    


# ----------------------

root = tk.Tk()
arduino = Arduino(device_timeout=2, baudrate=460800)
data_queue=queue.Queue()
data_acquirer = DataAcquirer(arduino, data_queue)
data_representer = DataRepresenter(arduino, data_queue, data_acquirer, root)


# Creazione finestra principale
root.title("Jumping platform")
root.geometry("1000x1000")
root.protocol("WM_DELETE_WINDOW", on_close)

# Sezione superiore: Selezione dispositivo e stato
frame_top = tk.Frame(root)
frame_top.pack(pady=10)

tk.Label(frame_top, text="Select Device:").pack(side=tk.LEFT)
device_var = tk.StringVar()
combobox_menu_device = ttk.Combobox(frame_top, textvariable=device_var, state="readonly")
combobox_menu_device.pack(side=tk.LEFT, padx=10)
combobox_menu_device.bind("<<ComboboxSelected>>", on_device_selected)
combobox_menu_device.bind("<FocusIn>", update_device_list)
combobox_menu_device.bind("<FocusOut>", update_device_list)

label_status_connection = tk.Label(frame_top, text="Not Connected", fg="red")
label_status_connection.pack(side=tk.LEFT, padx=10)

update_device_list(None)

# Scale tare
frame_scale_tare = tk.Frame(root)
frame_scale_tare.pack(pady=10)
btn_scale_tare = tk.Button(frame_scale_tare, text="Tare", command=scale_tare)
btn_scale_tare.pack(side=tk.LEFT, padx=10)
label_scale_tare = tk.Label(frame_scale_tare, text=f"Not calibrated!", font=("Arial", 12), fg="red")
label_scale_tare.pack(side=tk.LEFT, padx=20)

# Sezione pulsanti
frame_acquisition = tk.Frame(root)
frame_acquisition.pack(pady=10)

btn_start = tk.Button(frame_acquisition, text="Start", command=start_acquisition)
btn_start.pack(side=tk.LEFT, padx=10)
btn_stop = tk.Button(frame_acquisition, text="Stop", command=stop_acquisition)
btn_stop.pack(side=tk.LEFT, padx=10)

# Sezione indicatori
frame_acquisition_status = tk.LabelFrame(root)
frame_acquisition_status.pack(pady=10)
label_acquisition_status = tk.Label(frame_acquisition_status, text=f"Acquisition status: Stop", font=("Arial", 15))
label_acquisition_status.pack(side=tk.LEFT, padx=20)

frame_indicator_mass = tk.LabelFrame(root, text=f"Mass", padx=10, pady=10)
frame_indicator_mass.pack(pady=10, padx=10, fill="both")
label_indicator_mass = tk.Label(frame_indicator_mass, text=f"Mass: {0.0} Kg", font=("Arial", 12))
label_indicator_mass.pack(side=tk.LEFT, padx=20)

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

frame_indicator_expressed_strength = tk.LabelFrame(root, text=f"Expressed strength", padx=10, pady=10)
frame_indicator_expressed_strength.pack(pady=10, padx=10, fill="both")
label_indicator_expressed_strength_LX = tk.Label(frame_indicator_expressed_strength, text=f"Expressed strength LX: {0.0} N", font=("Arial", 12))
label_indicator_expressed_strength_LX.pack(side=tk.LEFT, padx=20)
label_indicator_expressed_strength_RX = tk.Label(frame_indicator_expressed_strength, text=f"Expressed strength RX: {0.0} N", font=("Arial", 12))
label_indicator_expressed_strength_RX.pack(side=tk.LEFT, padx=20)
label_indicator_expressed_strength_AVG = tk.Label(frame_indicator_expressed_strength, text=f"Expressed strength AVG: {0.0} N", font=("Arial", 12))
label_indicator_expressed_strength_AVG.pack(side=tk.LEFT, padx=20)

# Sezione grafici
frame_bottom = tk.Frame(root)
frame_bottom.pack(fill=tk.BOTH, expand=True)

# Creazione della figura e degli assi
fig, (ax_jpLX, ax_jpRX) = plt.subplots(2, 1, figsize=(5, 3))
fig.tight_layout()  # Migliora la disposizione
fig.subplots_adjust(hspace=0.5)  # Aumenta lo spazio verticale tra i grafici

# Creazione delle linee vuote
line_jpLX, = ax_jpLX.plot([], [])
line_jpRX, = ax_jpRX.plot([], [])

# Configurazione degli assi
ax_jpLX.set_title("Left Footrest")
ax_jpRX.set_title("Right Footrest")
ax_jpLX.set_xlabel("Time (s)")
ax_jpRX.set_xlabel("Time (s)")
ax_jpLX.set_ylabel("Newton (N)")
ax_jpRX.set_ylabel("Newton (N)")

# Integrazione con Tkinter
canvas = FigureCanvasTkAgg(fig, master=frame_bottom)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
canvas.draw()

# Aggiunta della barra degli strumenti sopra il grafico
toolbar = NavigationToolbar2Tk(canvas, frame_bottom)
toolbar.pack(side=tk.TOP, fill=tk.X)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Avvio aggiornamento grafici
data_acquirer.start()

root.mainloop()
