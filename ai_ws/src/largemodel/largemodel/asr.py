import rclpy
import os
import time
from rclpy.node import Node
import pyaudio
from playsound import playsound
import wave
import threading
import webrtcvad
import queue
from std_msgs.msg import String,UInt16,Bool
from utils.mic_serial import kws_mic
from utils import large_model_interface
from utils.large_model_interface import rec_wav_music_en
from ament_index_python.packages import get_package_share_directory
class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        # 初始化参数、变量 / Initialize parameters and variables
        self.init_param_config()
        # 初始化语音唤醒 / Initialize keyword spotting (KWS)
        self.kws_init()
        # 初始化ASR模型 / Initialize ASR model
        self.asr_mdoel_init()
        # 初始化语言设置 / Initialize language settings
        self.language_init()
        # 初始化系统声音 / Initialize system sound functionality
        self.system_sound_init()
        # 初始化ROS通信 / Initialize ROS communication
        self.init_ros_comunication()
        # 打印初始化信息 / Log initialization completion
        self.get_logger().info("asr_node Initialization completed")

    def init_ros_comunication(self):
        # 创建蜂鸣器发布者 / Create a publisher for the buzzer
        self.pub_beep = self.create_publisher(UInt16, "beep", 10)
        # 创建ASR发布者，发布转换完成的消息 / Create an ASR publisher to publish conversion results
        self.asr_pub = self.create_publisher(String, "asr", 5)
        # 创建唤醒信息发布者 / Create a publisher for wake-up signals
        self.wakeup_pub = self.create_publisher(Bool, "wakeup", 5)

    def init_param_config(self):
        self.user_speechdir = os.path.join(get_package_share_directory('largemodel'), "resources_file", "user_speech.wav")
        # 参数声明 / Declare parameters
        self.declare_parameter('VAD_MODE', 1)
        self.declare_parameter('sample_rate', 16000) 
        self.declare_parameter('frame_duration_ms', 30)        
        self.declare_parameter('language', 'en')
        self.declare_parameter('use_oline_asr', False) 
        self.declare_parameter('mic_serial_port', "/dev/ttyUSB1")
        self.declare_parameter('mic_index', 0)
        self.declare_parameter("regional_setting", "China")

        # 获取服务器参数 / Get server parameters
        self.VAD_MODE = self.get_parameter('VAD_MODE').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.frame_duration_ms = self.get_parameter('frame_duration_ms').get_parameter_value().integer_value
        self.language = self.get_parameter('language').get_parameter_value().string_value  
        self.use_oline_asr = self.get_parameter('use_oline_asr').get_parameter_value().bool_value
        self.mic_serial_port=self.get_parameter('mic_serial_port').get_parameter_value().string_value
        self.mic_index=self.get_parameter('mic_index').get_parameter_value().integer_value
        self.regional_setting = (self.get_parameter("regional_setting").get_parameter_value().string_value)

        self.frame_bytes = int(self.sample_rate * self.frame_duration_ms / 1000) # 音频帧大小 / Audio frame size

        # 大模型接口实例端 / Instance of the large model interface
        self.modelinterface=large_model_interface.model_interface()
        # 初始化 WebRTC VAD / Initialize WebRTC VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(self.VAD_MODE)
        self.current_thread = None  # 唤醒处理线程 / Thread for handling wake-up events
        self.stop_event=threading.Event()

    def main_loop(self):
        while rclpy.ok():
            while self.audio_request_queue.qsize() > 1:  # 只处理最近的一次唤醒请求，防止重复唤醒 / Process only the most recent wake-up request to prevent duplicates
                self.audio_request_queue.get()
            if not self.audio_request_queue.empty():
                self.audio_request_queue.get()
                self.wakeup_pub.publish(Bool(data=True))  # 发布唤醒信号 / Publish wake-up signal
                self.get_logger().info("I'm here")
                playsound(self.audio_dict[self.first_response])  # 应答用户 / Respond to the user

                if self.current_thread and self.current_thread.is_alive():  # 打断上次的唤醒处理线程 / Interrupt the previous wake-up handling thread
                    self.stop_event.set()
                    self.current_thread.join()  # 等待当前线程结束 / Wait for the current thread to finish
                    self.stop_event.clear()  # 清除事件 / Clear the event
                self.current_thread = threading.Thread(target=self.kws_handler)
                self.current_thread.daemon = True
                self.current_thread.start()            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

    def kws_handler(self)->None:
        if self.stop_event.is_set():
            return      

        if self.listen_for_speech(self.mic_index):
            asr_text = self.ASR_conversion(self.user_speechdir)  # 进行 ASR 转换 / Perform ASR conversion
            if asr_text =='error':  # 检查 ASR 结果长度是否小于4个字符 / Check if ASR result length is less than 4 characters
                self.get_logger().warn("I still don't understand what you mean. Please try again")
                playsound(self.audio_dict[self.error_response])  # 错误响应 / Error response
            else:
                
                self.get_logger().info(asr_text)
                self.get_logger().info("okay😀, let me think for a moment...")
                self.asr_pub_result(asr_text)  # 发布 ASR结果 / Publish ASR result            
        else:
            return

    def system_sound_init(self):  # 初始化系统声音相关的功能 / Initialize system sound functionality
        pkg_path=get_package_share_directory('largemodel')
        self.audio_dict={}  # 系统声音字典 / Dictionary of system sounds
        self.audio_dict['longwan-women-1']=os.path.join(pkg_path, "resources_file", "longwan-women-1.mp3")
        self.audio_dict['longwan-women-2']=os.path.join(pkg_path, "resources_file", "longwan-women-2.mp3")
        self.audio_dict['longxiaochun-women-1']=os.path.join(pkg_path, "resources_file", "longxiaochun-women-1.mp3")
        self.audio_dict['longxiaochun-women-2']=os.path.join(pkg_path, "resources_file", "longxiaochun-women-2.mp3")

    def asr_mdoel_init(self):  # 初始化asr模型 / Initialize ASR model
        if self.regional_setting == "international": 
            if self.use_oline_asr:
                self.get_logger().info(f"The online asr model :XUN-FEI ASR is loaded")   
            else:
                self.modelinterface.init_local_asr_model()
                self.get_logger().info(f"The local asr model :XUN-FEI ASR is loaded")
            
        elif self.regional_setting == "China":
            if self.use_oline_asr:
                # -------- SenseVoiceSmall 语音识别  --模型加载----- / Load SenseVoiceSmall online ASR model
                self.get_logger().info(f'The online asr model :{self.modelinterface.init_oline_asr(self.language)} is loaded')
            else:
                self.modelinterface.init_local_asr_model()
                self.get_logger().info('The asr model :SenseVoiceSmall is loaded')
        else:
            while True:
                self.get_logger().info('Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".')
                time.sleep(1)

    def language_init(self):
        if self.language=='zh':
            self.first_response='longwan-women-1'
            self.error_response='longwan-women-2'        
        elif self.language=='en':
            self.first_response='longxiaochun-women-1'
            self.error_response='longxiaochun-women-2'
        else:
            while True:  
                self.get_logger().error("Language setting error, please check your language setting")
                time.sleep(3)

    def kws_init(self):  # 初始化关键词唤醒相关的内容 / Initialize keyword spotting (KWS) related content
        self.port_name =self.mic_serial_port
        self.audio_request_queue = queue.Queue()  # 用于传递音频请求 / Queue for passing audio requests
        self.serial_port = kws_mic(port=self.port_name, kwsquence=self.audio_request_queue,baudrate=115200)
        self.serial_port.open()
        if not self.serial_port.ser or not self.serial_port.ser.is_open:
            self.get_logger().error("Failed to open KWS serial port.")
        receive_thread = threading.Thread(target=self.serial_port.receive_data)
        receive_thread.daemon = True
        receive_thread.start()                 

    def asr_pub_result(self,asr_result:str)->None:
        msg=String(data=asr_result)
        self.asr_pub.publish(msg)

    def ASR_conversion(self, input_file:str)->str:
        if self.regional_setting == "international":  
            if self.use_oline_asr:
                res=rec_wav_music_en()
                if res is not None:
                    return res
                else:
                    return "error"
            else:
                result=self.modelinterface.SenseVoiceSmall_ASR(input_file)
                if result[0] == 'ok' and len(result[1]) > 4:
                    return result[1]
                else:
                    self.get_logger().error(f'ASR Error:{result[1]}')  # ASR error.
                    return 'error'
        else:
            if self.use_oline_asr:
                result=self.modelinterface.oline_asr(input_file)
                if result[0] == 'ok' and len(result[1]) > 4:
                    return result[1]
                else:
                    self.get_logger().error(f'ASR Error:{result[1]}')  # ASR error.
                    return 'error'
            else:
                result=self.modelinterface.SenseVoiceSmall_ASR(input_file)
                if result[0] == 'ok' and len(result[1]) > 4:
                    return result[1]
                else:
                    self.get_logger().error(f'ASR Error:{result[1]}')  # ASR error.
                    return 'error'            

    def listen_for_speech(self,mic_index=0):
        p = pyaudio.PyAudio()   # Create PyAudio instance. / 创建PyAudio实例。
        audio_buffer = []       # Store audio data. / 存储音频数据。
        silence_counter = 0     # Silence counter. / 静音计数器。
        MAX_SILENCE_FRAMES = 90  # 30帧*30ms=900ms静音后停止 / Stop after 900ms of silence (30 frames * 30ms)
        speaking = False  # Flag indicating speech activity. / 语音活动标志。
        frame_counter = 0  # Frame counter. / 计数器。
        stream_kwargs = {
            'format': pyaudio.paInt16,
            'channels': 1,
            'rate': self.sample_rate,
            'input': True,
            'frames_per_buffer': self.frame_bytes,
        }
        if mic_index != 0:
            stream_kwargs['input_device_index'] = mic_index

        # Prompt the user to speak via the buzzer. / 通过蜂鸣器提示用户讲话。
        self.pub_beep.publish(UInt16(data = 1))
        time.sleep(0.5)
        self.pub_beep.publish(UInt16(data = 0))

        try:
            # Open audio stream. / 打开音频流。
            stream = p.open(**stream_kwargs)
            while True:
                if self.stop_event.is_set():
                    return False 
        
                frame = stream.read(self.frame_bytes, exception_on_overflow=False)  # Read audio data. / 读取音频数据。
                is_speech = self.vad.is_speech(frame, self.sample_rate)  # VAD detection. / VAD检测。
  
                if is_speech:
                    # Detected speech activity. / 检测到语音活动。
                    speaking = True
                    audio_buffer.append(frame)
                    silence_counter = 0
                else:
                    if speaking:
                        # Detect silence after speech activity. / 在语音活动后检测静音。
                        silence_counter += 1
                        audio_buffer.append(frame)  # Continue recording buffer. / 持续记录缓冲。
                        
                        # End recording when silence duration meets the threshold. / 静音持续时间达标时结束录音。
                        if silence_counter >= MAX_SILENCE_FRAMES:
                            break
                frame_counter += 1
                if frame_counter % 2 == 0:
                    self.get_logger().info('1' if is_speech else '-')
                    # Real-time status display.
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

        # Save valid recording (remove trailing silence). / 保存有效录音（去除尾部静音）。
        if speaking and len(audio_buffer) > 0:
            # Trim the last silent part. / 裁剪最后静音部分。
            clean_buffer = audio_buffer[:-MAX_SILENCE_FRAMES] if len(audio_buffer) > MAX_SILENCE_FRAMES else audio_buffer
            
            with wave.open(self.user_speechdir, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(clean_buffer))
                return True



def main(args=None):
    rclpy.init(args=args)
    sense_voice_node = ASRNode()
    try:
        sense_voice_node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        sense_voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


