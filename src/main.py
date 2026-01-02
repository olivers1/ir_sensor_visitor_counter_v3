import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import pwmio
from enum import Enum
import numpy as np


# IO-pin setup for IR-leds with pwm output signal
ir_led0_pwm = pwmio.PWMOut(board.D14, frequency=38000, duty_cycle=32768)   # 50% duty cycle = 65535 / 2 = 32768
ir_led1_pwm = pwmio.PWMOut(board.D15, frequency=38000, duty_cycle=32768)

# SPI setup
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)  # create the spi bus
cs = digitalio.DigitalInOut(board.D8)   # create the cs (chip select)
mcp = MCP.MCP3008(spi, cs)  # create the mcp object


class SensorTrigState(Enum):
    NO_TRIG = 0
    TRIG = 1
    UNKNOWN = 2


class SensorSample:
    def __init__(self):
        self.value: int = 0
        self.timestamp: int = 0
        self.trig_state = SensorTrigState.UNKNOWN
        
    def set_sample(self, value, timestamp, trig_state):
        self.value = value
        self.timestamp = timestamp
        self.trig_state = trig_state
    
    def get_sample(self):
        return self.value, self.timestamp, self.trig_state


class IrSensor:
    def __init__(self, mcp_channel :int, sensor_trig_threshold: int):
        self.mcp_channel = mcp_channel
        self.sensor_trig_threshold = sensor_trig_threshold
        
    def get_sensor_data(self):
        # read sensor value and timestamp
        sensor_read = AnalogIn(mcp, self.mcp_channel)
        value = sensor_read.value
        timestamp = round(time.time()*1000)
        
        # evaluate readout value to determine if sensor was trigged (blocked)
        trig_state = SensorTrigState.NO_TRIG
        if(value < self.sensor_trig_threshold):   # detect sensor trig. below threshold == trig, above threshold = no trig
            trig_state = SensorTrigState.TRIG    # trig detected
        else:
            trig_state = SensorTrigState.NO_TRIG  # no trig detected
        return value, timestamp, trig_state
    

class SensorHandler:
    def __init__(self, number_of_sensors: int, num_sample_columns: int, num_consecutive_trigs):
        self.number_of_sensors = number_of_sensors
        self.num_sample_columns = num_sample_columns
        self.num_consecutive_trigs = num_consecutive_trigs
        
        self.index_counter = 0
        self.create_log_arrays()  # create log arrays to store log samples

    def create_log_arrays(self):
        self.sensor_log_sample_array = self.create_log_sample_array(self.number_of_sensors, self.num_sample_columns)
        self.consecutive_num_trigs_array = self.create_log_sample_array(self.number_of_sensors, self.num_consecutive_trigs)
        
    def register_log_sample(self, sensor_id, value: int, timestamp: int, trig_state: SensorTrigState):
        # check if there are any empty columns to store sample in, otherwise create more columns
        # check for emty columns only when all sensors have stored their data samples (1st sensor_id is 'sensor0'
        if(sensor_id == self.number_of_sensors - 1):
            #print(np.shape(self.sensor_log_sample_array)[1])
            if(np.shape(self.sensor_log_sample_array)[1] <= self.index_counter + 1):    # check if all array columns are occupied
                new_columns = self.create_log_sample_array(self.number_of_sensors, 1)   # create an extra column
                self.sensor_log_sample_array = np.append(self.sensor_log_sample_array, new_columns, 1)
        
        # store log sample
        self.sensor_log_sample_array[sensor_id][self.index_counter].set_sample(value, timestamp, trig_state)

        # increase index_counter when all sensors have stored their data samples, first sensor_id is 'sensor0'
        if(sensor_id == self.number_of_sensors - 1):
            self.index_counter += 1
            return self.index_counter - 1   # value adjusted to return the index of last stored sample
        return self.index_counter

    def create_log_sample_array(self, number_of_sensors: int, num_of_columns):
        return np.array([[SensorSample() for _ in range(num_of_columns)] for _ in range(number_of_sensors)], dtype=object)    # create an number_of_sensors dimensional array
                                
    def get_log_sample(self, sensor_id, sample_index):
        return self.sensor_log_sample_array[sensor_id][sample_index]
    
    def get_sensor_log_sample_array(self):
        return self.sensor_log_sample_array
    
    def get_element_consecutive_num_trigs_array(self, sensor_id, sample_index):
        return self.consecutive_num_trigs_array[sensor_id][sample_index]
    
    def get_consecutive_num_trigs_array(self):
        return self.consecutive_num_trigs_array


class AppLoggingState(Enum):
    INIT = 0
    IDLE = 1
    LOG_START = 2
    ALL_SENSORS_TRIGGED = 3
    LOG_STOP = 4
    LOG_EVALUATION = 5


class CountdownTimer:
    def __init__(self, delay):
        self.delay = delay
        self.fire_time = time.monotonic() + delay
        self.fired = False
    
    def start(self):
        # start or restart the timer
        self.fire_time = time.monotonic() + self.delay
        self.fired = False

    def cancel(self):
        self.fire_time = None
        self.fired = False
    
    def ready(self):
        if self.fired == True:
            return False
        
        if time.monotonic() >= self.fire_time:
            self.fired = True
            return True
        
        return False


class TrigEvaluationManager:
    def __init__(self):
        self.sensor_trig_threshold = 1000   # sensor digital value (0 - 65535) to represent IR-sensor detection, a value below threshold means sensor is trigged/blocked
        self.number_of_sensors = 2
        self.sensors = []   # list containing all sensors
        self.initial_num_sample_columns = 1     # specifies number of columns for the initial log array
        self.readout_frequency = 1  # Hz [12 Hz = real run mode] 
        self.index_counter = 0      # current index of sensor_log_sample_array
        self.num_consecutive_trigs = 5     # [5 - run] number of sensor trigs in a consecutive order to count it as a trig
        self.sensor_handler = SensorHandler(self.number_of_sensors, self.initial_num_sample_columns, self.num_consecutive_trigs)
        self.verified_sensor_trig_state = [SensorTrigState.UNKNOWN, SensorTrigState.UNKNOWN]
        self.current_app_logging_state = AppLoggingState.INIT   # keeps track of current app logging state
        self.previous_app_logging_state = AppLoggingState.INIT  # keeps track of the previous app logging state
        self.index_start_sample = 0     # index at sensor_log_sample_array when logging is started
        self.index_stop_sample = 0      # index at sensor_log_sample_array when logging is stopped
        self.countdown_timer = CountdownTimer(10)     # stores an CountdownTimer object

    def run(self):
        for sensor_id in range(self.number_of_sensors):
            self.sensors.append(IrSensor(sensor_id, self.sensor_trig_threshold))
    
        while(True):    
            print("current_app_logging_state:", self.current_app_logging_state.name)
            for sensor_id, sensor in enumerate(self.sensors):
                self.index_counter = self.sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the tuple returned from the function call
                #===
                print(f"(sensor_id, index_counter: {sensor_id}, {self.index_counter})") 
                print(self.sensor_handler.get_log_sample(sensor_id, self.index_counter).value, self.sensor_handler.get_log_sample(sensor_id, self.index_counter).timestamp, self.sensor_handler.get_log_sample(sensor_id, self.index_counter).trig_state.name)
                
            time.sleep(1/self.readout_frequency) # setting periodic time for the sensor read
            
            # start adding samples to the consecutive_trigs array and analyse it when number of samples exceeds size of the consecutive_trigs array
            if(self.index_counter >= self.num_consecutive_trigs - 1):
                self.verify_sensor_trig_states()
            
            self.update_app_logging_state()
 


    def verify_sensor_trig_states(self):
        # add samples to consecutive_num_trigs_array
        for sensor_id in range(self.number_of_sensors):
            for list_index in range(self.num_consecutive_trigs):
                self.sensor_handler.consecutive_num_trigs_array[sensor_id][list_index] = self.sensor_handler.get_log_sample(sensor_id, self.index_counter - ((self.num_consecutive_trigs - 1) - list_index))
            
        for list_index in range(self.num_consecutive_trigs):
            for sensor_id in range(self.number_of_sensors):
                print(f"(sensor_id, list_index: {sensor_id}, {list_index})") 
                print(self.sensor_handler.get_element_consecutive_num_trigs_array(sensor_id, list_index).timestamp, self.sensor_handler.get_element_consecutive_num_trigs_array(sensor_id, list_index).trig_state.name)
            
        # check if trig state is stable by verifying that all elements in a row have the same trig state independently of the trig states in the other row/rows
        trig_states = np.array([[sample.trig_state.name for sample in row] for row in self.sensor_handler.consecutive_num_trigs_array])
        row_check = np.all(trig_states == trig_states[:, [0]], axis=1)
        print(row_check)

        # store current verified trig state for the sensors
        self.verified_sensor_trig_state = []    # clear variable each iteration to only store current trig state for the sensors
        for sensor_id, is_stable in enumerate(row_check):
            if is_stable == True:
                self.verified_sensor_trig_state.append(self.sensor_handler.consecutive_num_trigs_array[sensor_id][0].trig_state)
            elif is_stable == False:
                self.verified_sensor_trig_state.append(SensorTrigState.UNKNOWN) 
        print("verified_sensor_trig_state:", [sensor_id.name for sensor_id in self.verified_sensor_trig_state])

    def update_app_logging_state(self):
        # verify that both sensors are unblocked (NO_TRIG) and update app logging state to IDLE
        if self.current_app_logging_state == AppLoggingState.INIT and all(sensor_id == SensorTrigState.NO_TRIG for sensor_id in self.verified_sensor_trig_state):
            self.current_app_logging_state = AppLoggingState.IDLE
        # set app logging state to LOG_START if any of the sensors are confirmed to be in TRIG state
        elif self.current_app_logging_state == AppLoggingState.IDLE and any(sensor_id == SensorTrigState.TRIG for sensor_id in self.verified_sensor_trig_state):
            self.current_app_logging_state = AppLoggingState.LOG_START
            self.capture_log_start_stop_index(AppLoggingState.LOG_START)    # store log start index from the log_sample_array
            # start timer to timeout logging if only one of the sensors is trigged
            self.countdown_timer.start()
        # stop logging if timer times out or update logging state to ALL_SENSORS_TRIGGED and cancel timer if both sensors are trigged
        elif self.current_app_logging_state == AppLoggingState.LOG_START:
            if self.countdown_timer.ready() == True and sum(sensor_id == SensorTrigState.TRIG for sensor_id in self.verified_sensor_trig_state) == 1:
                self.current_app_logging_state = AppLoggingState.LOG_STOP
                self.capture_log_start_stop_index(AppLoggingState.LOG_STOP)     # store log stop index from the log_sample_array
            # if countdown timer has not fired yet and both sensors are being trigged, cancel the timer and update logging state accordingly
            elif self.countdown_timer.ready() == False and all(sensor_id == SensorTrigState.TRIG for sensor_id in self.verified_sensor_trig_state):
                self.countdown_timer.cancel()   # stop timeout timer
                self.current_app_logging_state = AppLoggingState.ALL_SENSORS_TRIGGED
        # ADD CODE HERE TO SET TO RUN A TIMER AND AFTER IT FINISHED ADD A NEW APP LOGGING STATE SAYING FIRST MEAN VALUE SESSION COMPLETED
     
    def capture_log_start_stop_index(self, log_action: AppLoggingState):
        # capture sample index when app logging state is set to LOG_START
        if log_action == AppLoggingState.LOG_START:
            self.index_start_sample = self.index_counter - self.num_consecutive_trigs   # adjust index to the first sample to detect movement
            print("index_start_sample:", self.index_start_sample)
        elif log_action == AppLoggingState.LOG_STOP:
            self.index_stop_sample = self.index_counter
            print("index_stop_sample:", self.index_stop_sample)

def main():
    app = TrigEvaluationManager()
    app.run()

if __name__ == "__main__":
   main()
