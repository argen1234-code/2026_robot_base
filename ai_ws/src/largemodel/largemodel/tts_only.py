import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import pygame
from utils import large_model_interface
from utils.paths import package_root
from ament_index_python.packages import get_package_share_directory

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_only_node')
        self.get_logger().info("TTS Only Node is starting...")

        # Initialize parameters. / 初始化参数。
        self.init_param_config()

        # Initialize model interface. / 初始化模型接口。
        self.model_client = large_model_interface.model_interface()
        self.model_client.init_language(self.language)

        # Initialize TTS. / 初始化TTS。
        self.system_sound_init()

        # Initialize Pygame audio player. / 初始化Pygame音频播放器。
        pygame.mixer.init()
        self.get_logger().info("Pygame mixer initialized.")

        # Create a subscriber to receive text for synthesis. / 创建一个订阅者来接收要合成的文本。
        self.text_subscriber = self.create_subscription(
            String,
            'tts_text_input',
            self.tts_callback,
            10)
        
        self.get_logger().info("TTS Only Node started. Waiting for text on topic '/tts_text_input'.")

    def init_param_config(self):
        self.pkg_path = get_package_share_directory('largemodel')
        self.declare_parameter('language', 'zh')
        self.declare_parameter('useolinetts', False)
        self.declare_parameter("regional_setting", "China")
        
        self.language = self.get_parameter('language').get_parameter_value().string_value
        self.useolinetts = self.get_parameter('useolinetts').get_parameter_value().bool_value
        self.regional_setting = (self.get_parameter("regional_setting").get_parameter_value().string_value)


        if self.useolinetts:
            self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.mp3")
        else:
            self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.wav")
        self.get_logger().info(f"Language set to: {self.language}")
        self.get_logger().info(f"Using online TTS: {self.useolinetts}")
        self.get_logger().info(f"TTS output path: {self.tts_out_path}")

    def system_sound_init(self):
        """Initialize TTS system"""
        if self.regional_setting == "China":  # 如果是中国地区
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.mp3")
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.wav")
        elif self.regional_setting == "international":  # 如果是国际地区
            model_type = "XUNFEI_FOR_INTERNATIONAL"
            pkg_path = package_root()
            self.tts_out_path = os.path.join(pkg_path, "resources_file", "XUNFEI_TTS.mp3")
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)
        self.model_client.tts_model_init(model_type, self.language)
        self.get_logger().info(f'TTS initialized with {model_type} model')

    def tts_callback(self, msg):
        """Callback function to handle incoming text messages for TTS."""
        user_input = msg.data
        self.get_logger().info(f'Received text for TTS: "{user_input}"')
        
        # Voice synthesis. / 语音合成。
        try:
            self.model_client.voice_synthesis(user_input, self.tts_out_path)
            # Play audio. / 播放音频。
            self.play_audio(self.tts_out_path)
        except Exception as e:
            self.get_logger().error(f"An error occurred during TTS processing: {e}")

    def play_audio(self, file_path):
        """Play audio synchronously. / 同步播放音频。"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"Audio file not found: {file_path}")
            return
            
        try:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.delay(100)
            self.get_logger().info("Playback finished.")
        except Exception as e:
            self.get_logger().error(f"Error playing audio: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 