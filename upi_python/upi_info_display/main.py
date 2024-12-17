import time
import subprocess
import logging
from gpiozero import Button
from e_paper_display import display_on_epaper, display_on_epaper_inverse
from constants import SYSTEM_STATES, ACTION_STATES

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class RecordingSystem:
    def __init__(self):
        self.cur_system_state = SYSTEM_STATES[0]
        self.cur_action_state = ACTION_STATES[0]
        self.button = Button(15, pull_up=True)
        self.pressed_time = None
        self.long_press_duration = 3
        self.system_started = False
        self.stop_system()
        self.start_system()
        self.start_action_server()

    def run_script_with_display(self, script_path, start_message, end_message, sleep_time):
        logging.info(start_message)
        display_on_epaper(start_message, update_time=True)
        try:
            self.run_script(script_path)
            time.sleep(sleep_time)
            logging.info(end_message)
            display_on_epaper(end_message, update_time=True)
        except Exception as e:
            logging.error(f"Error during {start_message.lower()}: {e}")
            display_on_epaper(f"Error: {e}", update_time=True)


    def run_script_with_display_inverse(self, script_path, start_message, end_message, sleep_time):
        logging.info(start_message)
        display_on_epaper_inverse(start_message, update_time=True)
        try:
            self.run_script(script_path)
            time.sleep(sleep_time)
            logging.info(end_message)
            display_on_epaper_inverse(end_message, update_time=True)
        except Exception as e:
            logging.error(f"Error during {start_message.lower()}: {e}")
            display_on_epaper_inverse(f"Error: {e}", update_time=True)


    def start_system(self):
        self.run_script_with_display("./scripts/start_system.sh", "System \n Starting ...", "System \n Started", 10)
        self.system_started = True

    def stop_system(self):
        self.run_script_with_display("./scripts/stop_system.sh", "System \n Stopping ...", "System \n Stopped", 3)

    def start_action_server(self):
        self.run_script_with_display("./scripts/start_action_server.sh", "Action Server \n Starting ...", "Action Server \n Started", 1)

    def start_recording(self):
        self.run_script_with_display("./scripts/record_rosbag.sh", "Recording \n Starting ...", "Recording \n Started", 3)

    def stop_recording(self):
        self.run_script_with_display("./scripts/stop_record_rosbag.sh", "Recording \n Stopping ...", "Recording \n Stopped", 3)

    def run_script(self, script_path):
        try:
            subprocess.run([script_path], check=True)
            logging.info("Script executed successfully")
        except subprocess.CalledProcessError as e:
            logging.error(f"Failed to execute script {script_path}: {e}")
            raise
        except Exception as e:
            logging.error(f"Unexpected error while executing script {script_path}: {e}")
            raise

    def handle_long_press(self):
        logging.info(f"Current system state: {self.cur_system_state}")
        if self.cur_system_state == SYSTEM_STATES[0]:
            logging.info("Long press detected. Starting recording ...")
            self.start_recording()
            self.cur_system_state = SYSTEM_STATES[1]
        else:
            logging.info("Long press detected. Stopping recording ...")
            self.stop_recording()
            self.cur_system_state = SYSTEM_STATES[0]

    def handle_short_press(self):
        if self.system_started:
            action_state_str = "true" if not self.cur_action_state else "false"
            logging.info(f"Action: {action_state_str}")
            try:
                subprocess.Popen(["docker", "exec", "-it", "upi_system", "bash", "-c", f"source /ros_entrypoint.sh && rosservice call /upi/status/toggle_srv {action_state_str}"])
                self.cur_action_state = not self.cur_action_state
                if action_state_str == "true":
                    display_on_epaper_inverse(f"Recording \n Action: {self.cur_action_state}", update_time=True)
                else:
                    display_on_epaper(f"Recording \n Action: {self.cur_action_state}", update_time=True)
            except Exception as e:
                logging.error(f"Failed to toggle action state: {e}")
                display_on_epaper("Error: Action toggle failed", update_time=True)
        else:
            logging.warning("Docker container is not running or container_id is not available.")
            display_on_epaper("Error: Docker not running", update_time=True)

    def run(self):
        try:
            while True:
                if self.button.is_pressed:
                    if self.pressed_time is None:
                        self.pressed_time = time.time()
                else:
                    if self.pressed_time is not None:
                        press_duration = time.time() - self.pressed_time
                        if press_duration < self.long_press_duration:
                            self.handle_short_press()
                        else:
                            self.handle_long_press()
                        self.pressed_time = None
                time.sleep(0.1)
        except KeyboardInterrupt:
            logging.info("Shutting down system...")
            self.stop_system()

if __name__ == "__main__":
    record_system = RecordingSystem()
    record_system.run()