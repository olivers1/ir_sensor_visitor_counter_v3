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
import logging
from logging.handlers import RotatingFileHandler


# IO-pin setup for IR-leds with pwm output signal
ir_led0_pwm = pwmio.PWMOut(board.D14, frequency=38000, duty_cycle=32768)   # 50% duty cycle = 65535 / 2 = 32768
ir_led1_pwm = pwmio.PWMOut(board.D15, frequency=38000, duty_cycle=32768)

# SPI setup
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)  # create the spi bus
cs = digitalio.DigitalInOut(board.D8)   # create the cs (chip select)
mcp = MCP.MCP3008(spi, cs)  # create the mcp object

# setup logging to files
filename_debug = "debug.log"
filename_info = "info.log"
# logging handlers
console = logging.StreamHandler()
console.setLevel(logging.INFO)     # set the log level which is printed to terminal output

#file_handler_debug = logging.FileHandler(filename_debug, mode='w', encoding='utf-8')
file_handler_debug = RotatingFileHandler(
    "debug.log",
    mode="w",              # overwrite after rotation
    maxBytes= 5 * 1024 * 1024,  # 5 MB
    backupCount=5,         # keep 5 old file
    encoding="utf-8"
)
file_handler_debug.setLevel(logging.DEBUG)

file_handler_info = RotatingFileHandler(
    filename_info,
    mode='w',                  # overwrite after rotation
    maxBytes=5 * 1024 * 1024,  # 5 MB max
    backupCount=1,             # keep 1 old files
    encoding='utf-8'
)
file_handler_info.setLevel(logging.INFO)
# logging config
logging.basicConfig(
    level=logging.DEBUG, handlers=[console, file_handler_debug, file_handler_info],
    style="{",
    format="{asctime} - {funcName} - {levelname}: {message}",
    )

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
        self.sensor_log_sample_array = self.create_log_sample_array(self.number_of_sensors, self.num_sample_columns)
        self.consecutive_trigs_array = self.create_log_sample_array(self.number_of_sensors, self.num_consecutive_trigs)

    def reset(self):
        """Clear all logged samples and counters."""
        logging.info("All logged samples and counters cleared")
        self.index_counter = 0
        self.sensor_log_sample_array = self.create_log_sample_array(self.number_of_sensors, self.num_sample_columns)
        self.consecutive_trigs_array = self.create_log_sample_array(self.number_of_sensors, self.num_consecutive_trigs)

    def register_log_sample(self, sensor_id, value: int, timestamp: int, trig_state: SensorTrigState):
        # check if there are any empty columns to store sample in, otherwise create more columns
        # check for emty columns only when all sensors have stored their data samples (1st sensor_id is 'sensor0'
        if(sensor_id == self.number_of_sensors - 1):
            #print(np.shape(self.sensor_log_sample_array)[1])
            if(np.shape(self.sensor_log_sample_array)[1] <= self.index_counter + 1):    # check if all array columns are occupied
                new_columns = self.create_log_sample_array(self.number_of_sensors, 200)     # create 200 extra columns
                self.sensor_log_sample_array = np.append(self.sensor_log_sample_array, new_columns, 1)
        
        # store log sample
        self.sensor_log_sample_array[sensor_id][self.index_counter].set_sample(value, timestamp, trig_state)

        # increase index_counter when all sensors have stored their data samples, first sensor_id is 'sensor0'
        if(sensor_id == self.number_of_sensors - 1):
            self.index_counter += 1
        return self.index_counter

    def create_log_sample_array(self, number_of_sensors: int, num_of_columns):
        return np.array([[SensorSample() for _ in range(num_of_columns)] for _ in range(number_of_sensors)], dtype=object)    # create an number_of_sensors dimensional array
                                
    def get_log_sample(self, sensor_id, sample_index):
        return self.sensor_log_sample_array[sensor_id][sample_index]
    
    def get_sensor_log_sample_array(self):
        return self.sensor_log_sample_array
    
    def get_element_consecutive_trigs_array(self, sensor_id, sample_index):
        return self.consecutive_trigs_array[sensor_id][sample_index]
    
    def get_consecutive_trigs_array(self):
        return self.consecutive_trigs_array


class CountdownTimer:
    def __init__(self, delay):
        self.delay = delay
        self.fire_time = time.monotonic() + delay
        self.started = False
    
    def start(self):
        # start or restart the timer
        self.fire_time = time.monotonic() + self.delay
        self.started = True

    def cancel(self):
        self.fire_time = None
        self.started = False

    def is_started(self):
        return self.started
    
    def ready(self):
        if not self.started or self.fire_time is None:
            return False
        
        if time.monotonic() >= self.fire_time:
            self.started = False
            return True
        
        return False


class SensorsState(Enum):
    NO_TRIG = 0
    EXACTLY_ONE_TRIG = 1
    ALL_TRIG = 2
    UNKNOWN = 3


class AppLoggingState(Enum):
    INIT = 0
    IDLE = 1
    LOG_START = 2
    LOGGING = 3
    LOG_STOP = 4
    LOG_EVALUATION = 5


class SetLogIndex(Enum):
    STOP = 0
    START = 1


class MovementDirection(Enum):
    ENTRY = "ENTRY"
    EXIT = "EXIT"
    INVALID = "INVALID"


class TrigEvaluationManager:
    def __init__(self):
        self.sensor_trig_threshold = 1000   # sensor digital value (0 - 65535) to represent IR-sensor detection, a value below threshold means sensor is trigged/blocked
        self.number_of_sensors = 2
        self.sensors = []   # list containing all sensors
        self.initial_num_sample_columns = 1000  # specifies number of columns for the initial log array
        self.readout_frequency = 8  # Hz (8 Hz in run mode] 
        self.max_index_counter = 10000  # maximum number of colums (length) om sample array
        self.current_index_counter = 0  # current index of sensor_log_sample_array
        self.next_index_counter = 0     # next index of sensor_log_sample_array 
        self.num_consecutive_trigs = 5     # 5 (run mode) number of sensor trigs in a consecutive order to count it as a trig
        self.sensor_handler = SensorHandler(self.number_of_sensors, self.initial_num_sample_columns, self.num_consecutive_trigs)
        self.verified_sensor_trig_state = [SensorTrigState.UNKNOWN, SensorTrigState.UNKNOWN]
        self.prev_verified_sensor_trig_state = [SensorTrigState.UNKNOWN, SensorTrigState.UNKNOWN]
        self.current_state = AppLoggingState.INIT  # keeps track of current app logging state
        self.previous_state = AppLoggingState.INIT  # keeps track of the previous app logging state
        self.log_start_index = 0     # index at sensor_log_sample_array when logging is started
        self.log_stop_index = 0      # index at sensor_log_sample_array when logging is stopped
        self.log_started_timeout = 5    
        self.log_finished_timeout = 5   # timeout in seconds (before stopping logging and evaluate trig result) after logging has started and sensor state is NO_TRIG
        self.countdown_timers = {
            SensorsState.EXACTLY_ONE_TRIG: CountdownTimer(self.log_started_timeout),  # timeout if only one sensor triggers
            SensorsState.NO_TRIG: CountdownTimer(self.log_finished_timeout), # timeout before evaluate logs after sensors all sensors are unblocked (NO_TRIG)
        }
        self.sensor_trig_arrays = []
        self.sensor_mean_values = []
        self.first_trig_sensor_id = None    # stores and in value referring to sensor_id of the sensor that trigged first
        self.latest_trig_timestamp = 0      # the latest/highest timestamp of any of the sensors that trigged
        self.identified_movement_direction = []
        self.all_sensors_trigged_simultaniously = False     # keeps track of if both sensors were trigged at the same any time during logging. Used for validating the _detect_movement_direction() 

    def run(self):
        for sensor_id in range(self.number_of_sensors):
            self.sensors.append(IrSensor(sensor_id, self.sensor_trig_threshold))
    
        while(True):    
            for sensor_id, sensor in enumerate(self.sensors):
                self.next_index_counter = self.sensor_handler.register_log_sample(sensor_id, *sensor.get_sensor_data())    # '*' unpacks the tuple returned from the function call
            
                self.current_index_counter = self.next_index_counter - 1

            for sensor_id, sensor in enumerate(self.sensors):
                logging.debug("sensor_id, list_index: (%d, %d)", sensor_id, self.current_index_counter)
                logging.debug("%s | %d | %d", self.sensor_handler.get_log_sample(sensor_id, self.current_index_counter).trig_state.name, self.sensor_handler.get_log_sample(sensor_id, self.current_index_counter).timestamp, self.sensor_handler.get_log_sample(sensor_id, self.current_index_counter).value)
                    
            time.sleep(1/self.readout_frequency) # setting periodic time for the sensor read
            
            # start adding samples to the consecutive_trigs array and analyse it when number of samples exceeds size of the consecutive_trigs array
            if(self.current_index_counter >= self.num_consecutive_trigs - 1):
                self.verify_sensor_trig_states()
            
            self.update_logging_state()
            #print("current_state:", self.current_state.name)
            logging.debug("current_state: %s", self.current_state.name)

    def verify_sensor_trig_states(self):
        # add samples to consecutive_trigs_array
        for sensor_id in range(self.number_of_sensors):
            for list_index in range(self.num_consecutive_trigs):
                self.sensor_handler.consecutive_trigs_array[sensor_id][list_index] = self.sensor_handler.get_log_sample(sensor_id, self.current_index_counter - ((self.num_consecutive_trigs - 1) - list_index))
  
        for list_index in range(self.num_consecutive_trigs):
            for sensor_id in range(self.number_of_sensors):
                logging.debug(f"sensor_id, list_index: ({sensor_id}, {list_index})")
                logging.debug(f"{self.sensor_handler.get_element_consecutive_trigs_array(sensor_id, list_index).trig_state.name} | {self.sensor_handler.get_element_consecutive_trigs_array(sensor_id, list_index).timestamp} | {self.sensor_handler.get_element_consecutive_trigs_array(sensor_id, list_index).value}") 
                
        # check if trig state is stable by verifying that all elements in a row only have the same trig state. Independtly of what trig state the other row have
        trig_states = np.array([[sample.trig_state.name for sample in row] for row in self.sensor_handler.consecutive_trigs_array])
        row_check = np.all(trig_states == trig_states[:, [0]], axis=1)
        #print(row_check)

        # store current verified trig state for the sensors
        self.verified_sensor_trig_state = []    # clear array at each iteration to only store current trig state for the sensors
        for sensor_id, is_stable in enumerate(row_check):
            if is_stable == True:
                self.verified_sensor_trig_state.append(self.sensor_handler.consecutive_trigs_array[sensor_id][0].trig_state)
            elif is_stable == False:
                self.verified_sensor_trig_state.append(SensorTrigState.UNKNOWN) 
        logging.debug("verified_sensor_trig_state: %s", [sensor_id.name for sensor_id in self.prev_verified_sensor_trig_state])
        new_state = self.verified_sensor_trig_state

        if new_state != self.prev_verified_sensor_trig_state:
            logging.info("verified_sensor_trig_state Transition: %s → %s", [sensor_id.name for sensor_id in self.prev_verified_sensor_trig_state] , [sensor_id.name for sensor_id in new_state])
            self.prev_verified_sensor_trig_state = new_state.copy()

    def update_sensors_state(self):
        sensors_state = SensorsState.UNKNOWN
        if all(s == SensorTrigState.NO_TRIG for s in self.verified_sensor_trig_state):
            sensors_state = SensorsState.NO_TRIG
        elif sum(s == SensorTrigState.TRIG for s in self.verified_sensor_trig_state) == 1:
            sensors_state = SensorsState.EXACTLY_ONE_TRIG
        elif all(s == SensorTrigState.TRIG for s in self.verified_sensor_trig_state):
            sensors_state = SensorsState.ALL_TRIG
        return sensors_state
    
    def update_logging_state(self):
        sensors_state = self.update_sensors_state()

        if self.current_state == AppLoggingState.INIT:
            if self.current_index_counter < self.num_consecutive_trigs - 1:
                return
            
            if sensors_state == SensorsState.NO_TRIG:
                #self.current_state = AppLoggingState.IDLE
                self.enter_state(AppLoggingState.IDLE)
        
        elif self.current_state == AppLoggingState.IDLE:
            if sensors_state == SensorsState.EXACTLY_ONE_TRIG:
                # Detect which sensor triggered first
                self.first_trig_sensor_id = self.verified_sensor_trig_state.index(SensorTrigState.TRIG)
                self.enter_state(AppLoggingState.LOG_START)

            elif sensors_state == SensorsState.ALL_TRIG:
                self.first_trig_sensor_id = None    # both sensors trigged at the same time → INVALID
                self.enter_state(AppLoggingState.LOG_START)

            elif sensors_state == SensorsState.NO_TRIG:
                if self.current_index_counter > self.max_index_counter:   # prevent sample_log_array from growing to infinate size
                    self.clear_log_memory()
        
        elif self.current_state == AppLoggingState.LOG_START:
            self.enter_state(AppLoggingState.LOGGING)

        
        # elif self.current_state == AppLoggingState.LOG_START:
        #     if sensors_state == SensorsState.ALL_TRIG:
        #         #self.current_state = AppLoggingState.LOGGING
        #         self.enter_state(AppLoggingState.LOGGING)

        #     elif sensors_state == SensorsState.EXACTLY_ONE_TRIG:
        #         if self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].is_started() == False:
        #             self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].start()    # start timer to timeout if only one sensor is trigged

        #         if self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].ready():    # if timer finishes, set log stop index
        #             self.enter_state(AppLoggingState.LOG_STOP)
            
        #     else:
        #         self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].cancel()    # start timer to timeout if only one sensor is trigged
        
        
        elif self.current_state == AppLoggingState.LOGGING:
            if sensors_state == SensorsState.NO_TRIG:
                if self.countdown_timers[SensorsState.NO_TRIG].is_started() == False:
                    self.countdown_timers[SensorsState.NO_TRIG].start()

                if self.countdown_timers[SensorsState.NO_TRIG].ready():
                    self.enter_state(AppLoggingState.LOG_STOP)
            
            elif sensors_state == SensorsState.EXACTLY_ONE_TRIG:
                self.countdown_timers[SensorsState.NO_TRIG].cancel()
            
            elif sensors_state == SensorsState.ALL_TRIG:
                self.all_sensors_trigged_simultaniously = True  # both sensors were trigged at same time any time during the logging
                self.countdown_timers[SensorsState.NO_TRIG].cancel()
        
        elif self.current_state == AppLoggingState.LOG_STOP:
            logging.info("first_trig_sensor_id: %s", self.first_trig_sensor_id)
            self.enter_state(AppLoggingState.LOG_EVALUATION)

        elif self.current_state == AppLoggingState.LOG_EVALUATION:
            self.enter_state(AppLoggingState.INIT)

    def enter_state(self, new_state):
        if new_state == self.current_state:     # no state change since current state is same as new state
           return
        
        logging.info("AppLoggingState Transition: %s → %s",
            self.current_state.name,
            new_state.name)
        
        self.previous_state = self.current_state
        self.current_state = new_state

        if new_state == AppLoggingState.INIT:
            self.clear_log_memory()     # clear memory logs
        elif new_state == AppLoggingState.IDLE:
            pass
        elif new_state == AppLoggingState.LOG_START:
            self.capture_start_stop_index(SetLogIndex.START)    # set log start index
            
            #if self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].ready():    # if timer finishes, set log stop index
            #    self.capture_start_stop_index(SetLogIndex.STOP)

        elif new_state == AppLoggingState.LOGGING:
            pass
            #self.countdown_timers[SensorsState.EXACTLY_ONE_TRIG].cancel()   # cancel timer if all sensor have been trigged
            
        elif new_state == AppLoggingState.LOG_STOP:
            self.capture_start_stop_index(SetLogIndex.STOP)     # set log stop index

        elif new_state == AppLoggingState.LOG_EVALUATION:
            self.evaluate_logs(self.log_start_index, self.log_stop_index)
            
    def capture_start_stop_index(self, action):
        # capture sample index when app logging is set to start and stop respectively
        if action == SetLogIndex.START:
            self.log_start_index = self.next_index_counter - self.num_consecutive_trigs   # adjust index to the first sample when sensor was firstly trigged
            logging.info("log_start_index: %d", self.log_start_index)
        elif action == SetLogIndex.STOP:
            self.log_stop_index = self.next_index_counter
            logging.info("log_stop_index: %d", self.log_stop_index)

    def evaluate_logs(self, start_index, stop_index):
        self.sensor_trig_arrays = []    # clear list to avoid list growth
        self.sensor_mean_values = []    # clear list to avoid list growth
        
        # extract sensor trigs and store in a list containing the trigs as sublists for each sensor
        self._extract_sensor_trigs(start_index, stop_index)
        self._compute_sensor_means()
        self._detect_movement_direction()

    def _compute_sensor_means(self):
        for sensor_id, sensor_samples in enumerate(self.sensor_trig_arrays):
            sum = 0
            counter = 0
            mean_value = 0
            for sample in sensor_samples:
                sum += sample.timestamp 
                counter += 1
                # extract the latest trig timestamp
                if sample.timestamp > self.latest_trig_timestamp:
                    self.latest_trig_timestamp = sample.timestamp
            # calculate timestamp mean value for each sensor
            if counter > 0:
                mean_value = sum / counter
            else:
                None
            # store sensor trig mean value for each sensor, sensor_id is represented of index position in list
            self.sensor_mean_values.append(mean_value)
    
        print("sensor_trig_arrays:")
        for sensor_id, sensor_samples in enumerate(self.sensor_trig_arrays):
            logging.debug("sensor %d trig timestamps:", sensor_id)
            #print(f"sensor_id {sensor_id}")
            for sample in sensor_samples:
                logging.debug("%s | %d", sample.trig_state.name, sample.timestamp)

        for sensor_id, mean in enumerate(self.sensor_mean_values):
            logging.info("sensor %d mean timestamp: %.3f", sensor_id, mean)

    def _extract_sensor_trigs(self, start_index, stop_index):
        for sensor_id, sensor in enumerate(self.sensors):
            sensor_trigs = []
            for index in range(start_index, stop_index):
                sample = self.sensor_handler.get_log_sample(sensor_id, index)
                if sample.trig_state == SensorTrigState.TRIG:
                    sensor_trigs.append(sample)

            self.sensor_trig_arrays.append(sensor_trigs)    # store trigs for each sensor_id 


    def _detect_movement_direction(self):
        """
        Determines ENTRY / EXIT / INVALID based on sensor mean timestamps and if both sensors were trigged during logging state
        Assumes exactly 2 sensors.
        """
        self.identified_movement_direction.append(self.latest_trig_timestamp)   # add latest verified_sensor_trig timestamp
        # sanity checks
        if len(self.sensor_mean_values) != 2:
            self.identified_movement_direction.append(MovementDirection.INVALID)
            logging.warning("%s: invalid sensor count", self.identified_movement_direction)
            return self.identified_movement_direction

        t0, t1 = self.sensor_mean_values    # fetch mean value for each of the sensors
        # If one or both sensors never trigged (mean value == 0)
        if t0 == 0 or t1 == 0:
            self.identified_movement_direction.append(MovementDirection.INVALID)
            logging.info("%s: one or both sensors never trigged", self.identified_movement_direction)
            return self.identified_movement_direction
        
        # Same timestamp → ambiguous
        if abs(t0 - t1) < 1e-6:
            self.identified_movement_direction.append(MovementDirection.INVALID)
            logging.info("%s: sensor mean values indicates sensors were trigged at same time", self.identified_movement_direction)
            return self.identified_movement_direction
        
        # verify that all sensors were trigged at the same time, anytime during logging (overlapping sensor trig)
        if self.all_sensors_trigged_simultaniously == False:
            self.identified_movement_direction.append(MovementDirection.INVALID)
            logging.info("%s: sensors were never trigged at same time, anytime during logging", self.identified_movement_direction)
            return self.identified_movement_direction


        # direction determination
        if t0 < t1:
            expected_first = 0
            direction = MovementDirection.EXIT
        else:
            expected_first = 1
            direction = MovementDirection.ENTRY
        
        # check which sensor that trigged first during logging state
        if self.first_trig_sensor_id != expected_first:
            self.identified_movement_direction.append(MovementDirection.INVALID)
            logging.info("%s: direction mismatch, order of senors mean values does not match the first trigged sensor", self.identified_movement_direction)
        else:
            self.identified_movement_direction.append(direction)
            logging.info("%s: detected movement direction", self.identified_movement_direction)
        
        return self.identified_movement_direction

    def clear_log_memory(self):
        # reset sensor handler buffers
        self.sensor_handler.reset()
        
        # reset variables
        self.first_trig_sensor_id = None
        self.latest_trig_timestamp = 0
        self.all_sensors_trigged_simultaniously = False

        # reset counters
        self.current_index_counter = 0
        self.next_index_counter = 0

        # reset indices
        self.log_start_index = 0
        self.log_stop_index = 0

        # clear evaluation results
        self.sensor_trig_arrays = []
        self.sensor_mean_values = []
        self.identified_movement_direction = []

        # reset verified trig states
        self.verified_sensor_trig_state = [SensorTrigState.UNKNOWN for _ in range(self.number_of_sensors)]

        # cancel timers
        for timer in self.countdown_timers.values():
            timer.cancel()


def main():
    app = TrigEvaluationManager()
    app.run()

if __name__ == "__main__":
   main()
