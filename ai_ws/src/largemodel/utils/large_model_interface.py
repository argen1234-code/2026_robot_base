# 🔧 Cleaned import statements - only keep used imports / 清理后的导入语句 - 只保留实际使用的导入
import dashscope
from openai import OpenAI
import os
import wave
from http import HTTPStatus
from dashscope.audio.asr import Recognition, TranslationRecognizerRealtime
# 云端 ASR 模式下不导入 funasr/torch，避免白白占用 ~0.5GB；
# 只有切换到本地 SenseVoice 时才在 init_local_asr_model() 里延迟导入。
from dashscope.audio.tts_v2 import SpeechSynthesizer
from ament_index_python.packages import get_package_share_directory
import yaml
import base64
import json
import cv2
from urllib.parse import quote_plus, urlencode
from urllib.request import Request, urlopen
from urllib.error import URLError
import datetime
import time
from datetime import datetime
from wsgiref.handlers import format_date_time
from time import mktime
import hmac
import hashlib
import websocket
import _thread as thread
import ssl

xufei = ""
Ws_Param = ""
STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识

record_speech_file = os.path.join(
    get_package_share_directory("largemodel"), "resources_file", "user_speech.wav"
)


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret, AudioFile):

        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.AudioFile = AudioFile

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {
            "domain": "iat",
            "language": "en_us",
            "accent": "mandarin",
            "vinfo": 1,
            "vad_eos": 10000,
        }

    # 生成url
    def create_url(self):
        url = "wss://ws-api.xfyun.cn/v2/iat"
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/iat " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        # 将请求的鉴权参数组合为字典
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        # 拼接鉴权参数，生成url
        url = url + "?" + urlencode(v)
        return url
    
# 收到websocket消息的处理
def on_message(ws, message):

    try:
        code = json.loads(message)["code"]
        sid = json.loads(message)["sid"]
        if code != 0:
            errMsg = json.loads(message)["message"]
            # print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            data = json.loads(message)["data"]["result"]["ws"]

            result = ""
            for i in data:
                for w in i["cw"]:
                    result += w["w"]

            global xufei
            xufei += result

    except Exception as e:
        print("receive msg,but parse exception:", e)

# 收到websocket错误的处理
def on_error(ws, error):
    print("### error:", error)


# 收到websocket关闭的处理
def on_close(ws, a, b):
    # print("###speak iat closed ###")
    return

# 收到websocket连接建立的处理
def on_open(ws):
    def run(*args):
        frameSize = 8000  # 每一帧的音频大小
        intervel = 0.04  # 发送音频间隔(单位:s)
        status = (
            STATUS_FIRST_FRAME  # 音频的状态信息，标识音频是第一帧，还是中间帧、最后一帧
        )

        with open(wsParam.AudioFile, "rb") as fp:
            while True:
                buf = fp.read(frameSize)
                # 文件结束
                if not buf:
                    status = STATUS_LAST_FRAME
                # 第一帧处理
                # 发送第一帧音频，带business 参数
                # appid 必须带上，只需第一帧发送
                if status == STATUS_FIRST_FRAME:

                    d = {
                        "common": wsParam.CommonArgs,
                        "business": wsParam.BusinessArgs,
                        "data": {
                            "status": 0,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        },
                    }
                    d = json.dumps(d)
                    ws.send(d)
                    status = STATUS_CONTINUE_FRAME
                # 中间帧处理
                elif status == STATUS_CONTINUE_FRAME:
                    d = {
                        "data": {
                            "status": 1,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                # 最后一帧处理
                elif status == STATUS_LAST_FRAME:
                    d = {
                        "data": {
                            "status": 2,
                            "format": "audio/L16;rate=16000",
                            "audio": str(base64.b64encode(buf), "utf-8"),
                            "encoding": "raw",
                        }
                    }
                    ws.send(json.dumps(d))
                    time.sleep(1)
                    break
                # 模拟音频采样间隔
                time.sleep(intervel)
        ws.close()

    thread.start_new_thread(run, ())

wsParam = ""


from utils.paths import package_root

pkg_path = package_root()
XUNFEI_TTS_FILE = os.path.join(pkg_path, "resources_file", "XUNFEI_TTS.mp3")

# XUNFEI_TTS_FILE = os.path.join(
#     get_package_share_directory("largemodel"), "resources_file", "XUNFEI_TTS.mp3"
# )

class Ws_Param_1(object):
    # 初始化 initialization
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.Text = Text

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {
            "aue": "lame",
            "sfl": 1,
            "auf": "audio/L16;rate=16000",
            "vcn": "x4_xiaoyan",
            "tte": "utf8",
            "speed": 50,
            "pitch": 50,
        }
        self.Data = {
            "status": 2,
            "text": str(base64.b64encode(self.Text.encode("utf-8")), "UTF8"),
        }
        # 使用小语种须使用以下方式，此处的unicode指的是 utf16小端的编码方式，即"UTF-16LE"”
        # self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-16')), "UTF8")}

    # 生成url Generate URL
    def create_url_1(self):
        url = "wss://tts-api.xfyun.cn/v2/tts"
        # 生成RFC1123格式的时间戳 Generate timestamp in RFC1123 format
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串 Splicing strings
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        # 进行hmac-sha256进行加密 Encrypt hmac-sha256
        signature_sha = hmac.new(
            self.APISecret.encode("utf-8"),
            signature_origin.encode("utf-8"),
            digestmod=hashlib.sha256,
        ).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding="utf-8")

        authorization_origin = (
            'api_key="%s", algorithm="%s", headers="%s", signature="%s"'
            % (self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        )
        authorization = base64.b64encode(authorization_origin.encode("utf-8")).decode(
            encoding="utf-8"
        )
        # 将请求的鉴权参数组合为字典 Combine the requested authentication parameters into a dictionary
        v = {"authorization": authorization, "date": date, "host": "ws-api.xfyun.cn"}
        # 拼接鉴权参数，生成url Splicing authentication parameters and generating URLs
        url = url + "?" + urlencode(v)
        return url


def on_message_1(ws, message):
    try:
        message = json.loads(message)
        code = message["code"]
        sid = message["sid"]
        audio = message["data"]["audio"]
        audio = base64.b64decode(audio)
        status = message["data"]["status"]
        # print(message)
        if status == 2:
            # print("ws is closed")
            ws.close()
        if code != 0:
            errMsg = message["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:
            with open(XUNFEI_TTS_FILE, "ab") as f:
                f.write(audio)
    except Exception as e:
        print("receive msg,but parse exception:", e)


# 收到websocket错误的处理 Handling of websocket errors received
def on_error_1(ws, error):
    print("### error:", error)


def on_close_1(ws, close_status_code, close_msg):
    return


# 收到websocket连接建立的处理 Received processing for establishing websocket connection
def on_open_1(ws):
    def run(*args):
        d = {
            "common": wsParam.CommonArgs,
            "business": wsParam.BusinessArgs,
            "data": wsParam.Data,
        }
        d = json.dumps(d)
        # print("------>开始发送文本数据")
        ws.send(d)
        if os.path.exists(XUNFEI_TTS_FILE):
            os.remove(XUNFEI_TTS_FILE)

    thread.start_new_thread(run, ())

class model_interface:
    def __init__(self, llm_platform='ollama', logger=None, mcp_server=None):
        self.llm_platform = llm_platform
        self.client = None
        # Optional separate client for image/video inference.  This lets the
        # text LLM (e.g. DeepSeek) and the vision model (e.g. a local
        # qwen2.5vl:3b via Ollama) run on different platforms.
        self.vision_client = None
        self.vision_model_name = None
        self.vision_platform = ''
        self.logger = logger  # Save logger instance / 保存logger实例
        self.mcp_server = mcp_server  # Save mcp_server instance / 保存mcp_server实例
        self.init_config_param()
        dashscope.api_key = self.tongyi_api_key

    def init_config_param(self):
        self.pkg_path=get_package_share_directory('largemodel')      
        config_param_file = os.path.join(self.pkg_path, "config", "large_model_interface.yaml")
        with open(config_param_file, 'r') as file:
            config_param = yaml.safe_load(file)
        
        def _env(name, key):
            """Environment variables override YAML so secrets never live in git."""
            return os.environ.get(name) or config_param.get(key)

        self.ANYTHINGLLM_BASE_URL = config_param.get("ANYTHINGLLM_BASE_URL")
        self.API_KEY = _env("ANYTHINGLLM_API_KEY", "API_KEY")


        # Tongyi Qianwen configuration / 通义千问配置
        self.tongyi_api_key = _env("TONGYI_API_KEY", 'tongyi_api_key')
        self.tongyi_base_url=config_param.get('tongyi_base_url')
        self.tongyi_model = config_param.get('tongyi_model')
        self.tongyi_media_model = config_param.get('tongyi_media_model', 'wanx-v1')

        # iFlytek Spark configuration / 讯飞星火配置
        self.spark_app_id = _env("SPARK_APP_ID", 'spark_app_id')
        self.spark_api_key = _env("SPARK_API_KEY", 'spark_api_key')
        self.spark_api_secret = _env("SPARK_API_SECRET", 'spark_api_secret')
        self.spark_model = config_param.get('spark_model')
        self.spark_model_url = config_param.get('spark_model_url')
        self.spark_media_model = config_param.get('spark_media_model', 'image_understanding')

        # Baidu Qianfan configuration / 百度千帆配置
        self.qianfan_api_key = _env("QIANFAN_API_KEY", 'qianfan_api_key')
        self.qianfan_base_url = config_param.get('qianfan_base_url')
        self.qianfan_model = config_param.get('qianfan_model')
        self.qianfan_media_model = config_param.get('qianfan_media_model', 'ernie-vilg-v2')

        # OpenRouter configuration / OpenRouter配置
        self.openrouter_api_key = _env("OPENROUTER_API_KEY", 'openrouter_api_key')
        self.openrouter_model = config_param.get('openrouter_model')
        # DeepSeek configuration / DeepSeek配置
        self.deepseek_api_key = _env("DEEPSEEK_API_KEY", 'deepseek_api_key')
        self.deepseek_base_url = config_param.get('deepseek_base_url', 'https://api.deepseek.com')
        self.deepseek_model = config_param.get('deepseek_model', 'deepseek-flash')

        # Ollama configuration / Ollama配置
        self.ollama_host = config_param.get('ollama_host', 'http://localhost:11434')
        self.ollama_model = config_param.get('ollama_model', 'llava')

        # Separate vision model.  Set vision_platform to "ollama" to run the
        # image/video model locally while the text LLM stays on another
        # platform.  Leave it empty to use the text platform for vision too.
        self.vision_platform = str(config_param.get('vision_platform', '') or '').strip().lower()
        self.ollama_vision_model = config_param.get('ollama_vision_model', 'qwen2.5vl:3b')
        self.ollama_vision_num_ctx = int(config_param.get('ollama_vision_num_ctx', 2048))
        self.ollama_vision_keep_alive = config_param.get('ollama_vision_keep_alive', 0)

        # ASR & TTS configuration / ASR & TTS 配置
        self.oline_asr_model=config_param.get('oline_asr_model')
        self.zh_tts_model=config_param.get('zh_tts_model')
        self.zh_tts_json=config_param.get('zh_tts_json')
        self.en_tts_model=config_param.get('en_tts_model')
        self.en_tts_json=config_param.get('en_tts_json')
        self.oline_asr_sample_rate=config_param.get('oline_asr_sample_rate')
        self.oline_tts_model=config_param.get('oline_tts_model')
        self.voice_tone=config_param.get('voice_tone')
        self.local_asr_model=config_param.get('local_asr_model')
        self.tts_supplier=config_param.get('tts_supplier')
        self.baidu_API_KEY=_env("BAIDU_API_KEY", 'baidu_API_KEY')
        self.baidu_SECRET_KEY=_env("BAIDU_SECRET_KEY", 'baidu_SECRET_KEY')
        self.CUID=config_param.get('CUID')
        self.PER=config_param.get('PER')
        self.SPD=config_param.get('SPD')
        self.PIT=config_param.get('PIT')
        self.VOL=config_param.get('VOL')

        # 允许用 MODELS_DIR 覆盖模型目录前缀（主机原生运行 / 容器路径兼容）。
        # YAML 里保留容器路径 /root/yahboom_ws/...，现有部署不受影响。
        models_dir = os.environ.get("MODELS_DIR")
        if models_dir:
            prefix = "/root/yahboom_ws/src/largemodel/MODELS"
            models_dir = models_dir.rstrip("/")
            for attr in ("local_asr_model", "zh_tts_model", "zh_tts_json",
                         "en_tts_model", "en_tts_json"):
                value = getattr(self, attr, None)
                if isinstance(value, str) and value.startswith(prefix):
                    setattr(self, attr, value.replace(prefix, models_dir, 1))

    def init_llm(self):
        """Initialize the corresponding model client based on the platform name. / 根据平台名称初始化对应的模型客户端。"""
        if self.llm_platform == 'ollama':
            self.init_ollama()
        elif self.llm_platform == 'tongyi':
            self.init_tongyi()
        elif self.llm_platform == 'spark':
            self.init_spark()
        elif self.llm_platform == 'qianfan':
            self.init_qianfan()
        elif self.llm_platform == 'openrouter':
            self.init_openrouter()
        elif self.llm_platform == 'deepseek':
            self.init_deepseek()
        else:
            print(f"Unsupported LLM platform: {self.llm_platform}, defaulting to Ollama.")
            self.init_ollama()

        self.init_vision()

    def init_vision(self):
        """
        Initialize a separate vision client when vision_platform is set.

        The text model and the vision model are deliberately independent:
        DeepSeek can handle text while qwen2.5vl:3b runs locally through
        Ollama.  If vision_platform is empty, image/video inference falls
        back to the text platform's client.
        """
        self.vision_client = None
        self.vision_model_name = None

        if not self.vision_platform:
            print("No separate vision platform configured; using the text platform for vision.")
            return

        try:
            if self.vision_platform == 'ollama':
                from ollama import Client
                self.vision_client = Client(host=self.ollama_host)
                self.vision_model_name = self.ollama_vision_model
            elif self.vision_platform == 'tongyi':
                self.vision_client = OpenAI(
                    api_key=self.tongyi_api_key, base_url=self.tongyi_base_url)
                self.vision_model_name = self.tongyi_model
            elif self.vision_platform == 'qianfan':
                self.vision_client = OpenAI(
                    api_key=self.qianfan_api_key, base_url=self.qianfan_base_url)
                self.vision_model_name = self.qianfan_model
            elif self.vision_platform == 'openrouter':
                self.vision_client = OpenAI(
                    base_url="https://openrouter.ai/api/v1",
                    api_key=self.openrouter_api_key)
                self.vision_model_name = self.openrouter_model
            elif self.vision_platform == 'deepseek':
                import httpx
                self.vision_client = OpenAI(
                    api_key=self.deepseek_api_key,
                    base_url=self.deepseek_base_url,
                    timeout=20.0,
                    max_retries=2,
                    http_client=httpx.Client(
                        timeout=httpx.Timeout(20.0, connect=10.0),
                        limits=httpx.Limits(max_keepalive_connections=0, max_connections=10),
                    ),
                )
                self.vision_model_name = self.deepseek_model
            else:
                print(f"Unsupported vision platform: {self.vision_platform}; "
                      "falling back to the text platform for vision.")
                self.vision_platform = ''
                return

            print(f"Vision client initialized successfully. "
                  f"Using {self.vision_platform} model {self.vision_model_name}")
        except Exception as e:
            print(f"Failed to initialize vision client: {e}")
            self.vision_client = None
            self.vision_model_name = None
            self.vision_platform = ''

    def init_language(self,language):
        self.system_text={}
        if language=='zh':
            self.system_text['text1']="请分析这个图像或视频"
            self.system_text['text2']="我已经准备好，请开始您的指令吧"        
        elif language=='en':
            self.system_text['text1']="Please analyze this image or video"        
            self.system_text['text2']="I am ready. Please start your instructions" 
        
    def init_messages(self):
        """General message history initialization. / 通用消息历史初始化。"""
        # Directly use the internally saved mcp_server instance / 直接使用内部保存的 mcp_server 实例
        self.messages = [
            {"role": "system", "content": self._generate_system_prompt(self.mcp_server)},
            {"role": "assistant", "content": self.system_text.get('text2', "I am ready.")}
        ]
    def _generate_system_prompt(self, mcp_server):
        """
        Dynamically generate the System Prompt.
        /
        动态生成系统级指令 (System Prompt)。
        """
        if getattr(self, 'minimal_prompt', False):
            return '你是机器人助手。请用简洁的中文回答用户。只输出JSON：{"response":"回答内容","tools":[]}'
        if not mcp_server:
            # Return a minimal system prompt without any tool descriptions.
            # /
            # 返回一个最简单的、不包含任何工具描述的系统提示。
            return "You are a helpful AI assistant."

        tools_description = mcp_server.get_tools_json_schema()

        # Check if language is set to English
        # 检查语言是否设置为英文
        is_english = hasattr(self, 'system_text') and 'text2' in self.system_text and 'Please start your instructions' in self.system_text['text2']
        
        # Fallback check using the node's language setting
        # 回退检查使用节点的语言设置
        if not is_english and hasattr(self, 'node') and hasattr(self.node, 'language'):
            is_english = self.node.language == 'en'

        # Return a more flexible Prompt that supports conversation, with bilingual comments.
        # /
        # 返回一个更灵活、支持对话的Prompt，并附带双语注释。
        if is_english:
            # English version of the system prompt
            # 英文版本的系统提示
            # Generate tool definitions separately to avoid f-string nesting issues
            
            return f'''You are the control hub for a robot, an AI capable of accurately converting natural language commands into JSON format, or engaging in natural conversation when no tools are available.

# Primary Rule
Your sole output must be a well-formed JSON object. Absolutely no text, explanations, or Markdown tags are allowed outside of the JSON.

# Tool Definition
The tools you can use are defined below. You must strictly adhere to their parameter schema:```json
{json.dumps(tools_description, indent=2, ensure_ascii=False)}
```

# Output Formats
Based on the user's intent, choose the most appropriate of the following three JSON structures for your response:

## Format 1: Direct Tool Call (For simple, explicit instructions)
```json
{{
  "response": "A confirmation or a brief reply to the user's command.",
  "tools": [
    {{
      "name": "Tool Name",
      "arguments": {{
        "parameter_name_1": "parameter_value_1"
      }}
    }}
  ]
}}
```

## Format 2: AI Agent Call (For complex, ambiguous, or multi-step instructions)
```json
{{
  "response": "Okay, this task requires some planning. Please wait a moment.",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "This must be populated with the user's original, complete, and unmodified instruction."
      }}
    }}
  ]
}}```

## Format 3: General Conversation (When no tools are applicable)
```json
{{
  "response": "This is the model's direct answer, e.g., for weather conditions, general knowledge questions, etc.",
  "tools": []
}}
```

# Core Instructions & Logic

1.  **Parameter Extraction**:
    *   **Mandatory**: You **must** find a value for every required argument of a tool.
    *   **Intelligent Filling**: If the user's instruction is vague (e.g., "draw me a picture"), you must use the user's descriptive text ("draw a picture") as the value for the core parameter (e.g., the `prompt` parameter for the `generate_image` tool). **Never leave the parameter value empty.**
    *   **Default Values**: For path parameters like `image_path` or `video_path`, if the user does not explicitly provide one, you can leave it as an empty string (`""`). The system will automatically use a default file.

2.  **Decision Logic**:
    *   **First Priority**: If the user's instruction is clear, the intent is explicit, and a single tool can complete the task -> Use **Format 1**.
    *   **Second Priority**: If the user's instruction is ambiguous, broad, or clearly requires multiple steps (e.g., "look around and then draw what you see") -> Use **Format 2** by calling `agent_call`.
    *   **Third Priority (Fallback)**: If the user is just making small talk or asking questions (like about weather, news, or general knowledge) and **no tool can fulfill the request** -> Use **Format 3**, providing a direct answer in the `response` field and keeping the `tools` field empty.

3.  **Response Content (`response` field)**:
    *   This field is used for natural language interaction with the user and should be concise and friendly.

# Examples

```json
{{
  "response": "Okay, generating an image of a cyberpunk city at sunset for you.",
  "tools": [
    {{
      "name": "generate_image",
      "arguments": {{
        "prompt": "A cyberpunk city at sunset"
      }}
    }}
  ]
}}
```

**User Instruction**: "First, look at the surrounding environment, then write a document summarizing what you see"
**Your Output**:
```json
{{
  "response": "Okay, this task requires some planning. Please wait a moment.",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "First, look at the surrounding environment, then write a document summarizing what you see"
      }}
    }}
  ]
}}
```

**User Instruction**: "Tell me a joke"
**Your Output**:
```json
{{
  "response": "Why are math books always so melancholic? Because it has too many problems.",
  "tools": [
    {{
      
    }}
  ]
}}
```

'''
        else:
            # Chinese version of the system prompt (original)
            # 中文版本的系统提示（原始版本）
            return f'''你是机器人的控制中枢，一个能将自然语言指令精确转换为JSON格式，或在无工具可用时进行自然对话的AI。

# 首要规则
你的唯一输出必须是一个结构完整的JSON对象。绝对禁止输出任何JSON之外的文本、解释或Markdown标记。

# 工具定义
你可使用的工具如下，请严格遵守其参数schema：
```json
{json.dumps(tools_description, indent=2, ensure_ascii=False)}
```

# 输出格式 / Output Formats
根据用户意图，从以下三种JSON结构中选择最合适的一种进行回复：

## 格式一：直接工具调用 (适用于简单、明确的指令)
```json
{{
  "response": "对用户指令的确认或简短回复。",
  "tools": [
    {{
      "name": "工具名称",
      "arguments": {{
        "参数名1": "参数值1"
      }}
    }}
  ]
}}
```

## 格式二：调用AI Agent (适用于复杂、模糊或多步指令)
```json
{{
  "response": "好的，这个任务需要我规划一下，请稍候。",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "这里必须填充用户原始的、完整的、未经修改的指令。"
      }}
    }}
  ]
}}
```

## 格式三：常规对话 (当没有工具适用时)
```json
{{
  "response": "这里是模型的直接回答，例如天气情况、常识问答等。",
  "tools": []
}}
```

# 核心指令与逻辑

1.  **参数提取:
    *   **强制性**: **必须**为工具的每一个必需参数（required arugments）找到一个值。
    *   **智能填充**: 如果用户指令很模糊（例如“给我画张画”），你必须将用户的描述性文本（“画张画”）作为核心参数的值（例如 `generate_image` 工具的 `prompt` 参数）。**绝对不能将参数值留空。**
    *   **默认值**: 对于 `image_path` 或 `video_path`这类路径参数，如果用户没有明确提供，可以留空（`""`），系统会自动使用默认文件。

2.  **决策逻辑**:
    *   **第一顺位**: 如果用户指令清晰，意图明确，且单个工具就能完成 -> 使用**格式一**。
    *   **第二顺位**: 如果用户指令模糊、宽泛，或明显需要多个步骤（例如“看看周围有什么，然后画出来”） -> 使用**格式二**，调用`agent_call`。
    *   **第三顺位 (Fallback)**: 如果用户只是在进行日常对话、提问（如天气、新闻、常识），并且**没有任何工具能满足需求** -> 使用**格式三**，在 `response` 字段中直接回答，并保持 `tools` 字段为空。

3.  **回复内容 (`response`字段)**:
    *   此字段是用于与用户进行自然语言交互的，应简洁、友好。

# 示例

```json
{{
  "response": "好的，正在为您生成一张关于日落时分赛博朋克城市的图片。",
  "tools": [
    {{
      "name": "generate_image",
      "arguments": {{
        "prompt": "日落时分的赛博朋克城市"
      }}
    }}
  ]
}}
```

**用户指令**: "先看看周围环境，然后写一份文档总结你看到了什么"
**你的输出**:
```json
{{
  "response": "好的，这个任务需要我规划一下，请稍候。",
  "tools": [
    {{
      "name": "agent_call",
      "arguments": {{
        "task": "先看看周围环境，然后写一份文档总结你看到了什么"
      }}
    }}
  ]
}}
```
'''


    def init_ollama(self):
        """Initialize Ollama client"""
        try:
            from ollama import Client
            self.client = Client(host=self.ollama_host)
            print(f"Ollama client initialized successfully. Using model {self.ollama_model}")
        except Exception as e:
            print(f"Failed to initialize Ollama client: {e}")
            self.client = None

    def init_tongyi(self):
        """Initialize Tongyi client"""
        try:
            dashscope.api_key = self.tongyi_api_key
            self.client = OpenAI(api_key=self.tongyi_api_key, base_url=self.tongyi_base_url)
            print(f"Tongyi (Dashscope) client initialized successfully. Using model {self.tongyi_model}")
        except Exception as e:
            print(f"Failed to initialize Tongyi client: {e}")
            self.client = None

    def init_spark(self):
        """Initialize Spark client"""
        try:
            from sparkai.llm.llm import ChatSparkLLM
            self.client = ChatSparkLLM(
                spark_api_url=self.spark_model_url,
                spark_app_id=self.spark_app_id,
                spark_api_key=self.spark_api_key,
                spark_api_secret=self.spark_api_secret,
                spark_llm_domain=self.spark_model,
                streaming=False,
            )
            print(f"Spark client initialized successfully. Using domain {self.spark_model}")
        except Exception as e:
            print(f"Failed to initialize Spark client: {e}")
            self.client = None

    def init_qianfan(self):
        """Initialize Qianfan client using OpenAI compatible mode"""
        try:
            self.client = OpenAI(
                api_key=self.qianfan_api_key,
                base_url=self.qianfan_base_url
            )
            print(f"Qianfan client (OpenAI compatible) initialized successfully. Using model {self.qianfan_model}")
        except Exception as e:
            print(f"Failed to initialize Qianfan client: {e}")
            self.client = None

    def init_openrouter(self):
        """Initialize OpenRouter client"""
        try:
            self.client = OpenAI(
                base_url="https://openrouter.ai/api/v1",
                api_key=self.openrouter_api_key,
            )
            print(f"OpenRouter client initialized successfully. Using model {self.openrouter_model}")
        except Exception as e:
            print(f"Failed to initialize OpenRouter client: {e}")
            self.client = None

    def init_deepseek(self):
        """Initialize DeepSeek client (OpenAI compatible)"""
        try:
            import httpx
            self.client = OpenAI(
                api_key=self.deepseek_api_key,
                base_url=self.deepseek_base_url,
                timeout=20.0,
                max_retries=2,
                http_client=httpx.Client(
                    timeout=httpx.Timeout(20.0, connect=10.0),
                    limits=httpx.Limits(max_keepalive_connections=0, max_connections=10),
                ),
            )
            print(f"DeepSeek client initialized successfully. Using model {self.deepseek_model}")
        except Exception as e:
            print(f"Failed to initialize DeepSeek client: {e}")
            self.client = None

    def infer_with_text(self, prompt, message=None):
        """Unified text inference interface. / 统一的文本推理接口。

        When ``message`` is supplied (an isolated context such as task
        planning or a vision prompt), it is copied locally and
        ``self.messages`` is left untouched.  This prevents background tasks
        from clobbering the user's conversation history.
        """
        if message is None:
            messages = self.messages
        else:
            messages = list(message)
        messages.append({"role": "user", "content": prompt})

        if not self.client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': messages}

        try:
            if self.llm_platform == 'ollama':
                response_content = self.ollama_infer(messages)
            elif self.llm_platform in ['tongyi', 'qianfan', 'openrouter', 'deepseek']:
                response_content = self.openai_compatible_infer(messages)
            elif self.llm_platform == 'spark':
                response_content = self.spark_infer(messages)
            else:
                response_content = f"Unsupported LLM platform: {self.llm_platform}"
        except Exception as e:
            response_content = f"Inference error on platform {self.llm_platform}: {e}"
        messages.append({"role": "assistant", "content": response_content})
        if message is None:
            self.messages = messages
        return {'response': response_content, 'messages': messages.copy()}

    def infer_with_image(self, image_path, text=None, message=None):
        """Unified image inference interface. / 统一的图像推理接口。"""
        if message is None:
            messages = self.messages
        else:
            messages = list(message)
        prompt = text if text else self.system_text.get('text1', "Describe this image.")
        messages.append({"role": "user", "content": prompt})

        if not self.client and not self.vision_client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': messages}

        try:
            if self.vision_platform == 'ollama' and self.vision_client is not None:
                response_content = self.ollama_infer(
                    messages, image_path=image_path,
                    client=self.vision_client, model=self.vision_model_name)
            elif self.vision_client is not None:
                response_content = self.openai_compatible_infer(
                    messages, image_path=image_path,
                    client=self.vision_client, model=self.vision_model_name)
            elif self.llm_platform == 'ollama':
                response_content = self.ollama_infer(messages, image_path=image_path)
            elif self.llm_platform in ['tongyi', 'qianfan', 'openrouter', 'deepseek']:
                response_content = self.openai_compatible_infer(messages, image_path=image_path)
            elif self.llm_platform == 'spark':
                response_content = self.spark_infer(messages, image_path=image_path)
            else:
                response_content = f"Image inference not supported for platform: {self.llm_platform}"
        except Exception as e:
            platform = self.vision_platform or self.llm_platform
            response_content = f"Image inference error on platform {platform}: {e}"

        messages.append({"role": "assistant", "content": response_content})
        if message is None:
            self.messages = messages
        return {'response': response_content, 'messages': messages.copy()}


    def generate_image(self, prompt, width=1024, height=1024, n=1):
        """Text-to-image function interface, supporting multiple platforms. / 文生图功能接口，支持多平台。"""
        if self.llm_platform == 'tongyi':
            return self._tongyi_generate_image(prompt, width, height, n)
        elif self.llm_platform == 'ollama':
            return self._ollama_generate_image_fallback(prompt, width, height, n)
        else:
            return {'status': 'failed', 'error': f'The current platform ({self.llm_platform}) does not support text-to-image generation. Currently only Tongyi and Ollama platforms are supported.'}

    def _tongyi_generate_image(self, prompt, width=1024, height=1024, n=1):
        """Tongyi Qianwen text-to-image implementation. / 通义千问文生图实现。"""
        
        if not self.client:
            return "Tongyi client is not initialized."
        
        try:
            # Use dashscope SDK for text-to-image call / 使用dashscope SDK进行文生图调用
            import dashscope
            dashscope.api_key = self.tongyi_api_key
            
            # Call text-to-image API / 调用文生图API
            response = dashscope.ImageSynthesis.call(
                model=self.tongyi_media_model,
                prompt=prompt,
                n=n,
                size=f'{width}*{height}'
            )
            
            if response.status_code == 200:
                # Return the generated image URLs / 返回生成的图片URL
                image_urls = [item['url'] for item in response.output['results']]
                return {'image_urls': image_urls, 'status': 'success'}
            else:
                return {'error': response.message, 'status': 'failed'}
        except Exception as e:
            return {'error': str(e), 'status': 'failed'}
    
    def _ollama_generate_image_fallback(self, prompt, width=1024, height=1024, n=1):
        """Fallback solution for text-to-image generation on the Ollama platform. / Ollama平台的文生图回退方案。"""
        # Since Ollama does not support text-to-image, provide fallback information / 由于Ollama不支持文生图，提供回退信息
        return {
            'status': 'failed',
            'error': 'Ollama does not support text-to-image generation.'
        }


    def text_to_image(self, prompt, width=1024, height=1024, n=1):
        """Independent text-to-image interface, specifically for generating images. / 独立的文生图接口，专门用于生成图像。"""
        # Directly call the generate_image method / 直接调用generate_image方法
        return self.generate_image(prompt, width, height, n)

    def infer_with_video(self, video_path, text=None, message=None):
        """Unified video inference interface. / 统一的视频推理接口。"""
        if message is None:
            messages = self.messages
        else:
            messages = list(message)
        prompt = text if text else "Describe the content of this video in detail."
        messages.append({"role": "user", "content": prompt})

        if not self.client and not self.vision_client:
            return {'response': f"Client for platform {self.llm_platform} is not initialized.", 'messages': messages}

        try:
            if self.vision_platform == 'ollama' and self.vision_client is not None:
                response_content = self.ollama_infer(
                    messages, video_path=video_path,
                    client=self.vision_client, model=self.vision_model_name)
            elif self.vision_client is not None:
                response_content = self.openai_compatible_infer(
                    messages, video_path=video_path,
                    client=self.vision_client, model=self.vision_model_name)
            elif self.llm_platform in ['tongyi', 'qianfan', 'openrouter']:
                response_content = self.openai_compatible_infer(messages, video_path=video_path)
            elif self.llm_platform == 'ollama':
                response_content = self.ollama_infer(messages, video_path=video_path)
            else:
                response_content = f"Video inference not supported for platform: {self.llm_platform}"
        except Exception as e:
            platform = self.vision_platform or self.llm_platform
            response_content = f"Video inference error on platform {platform}: {e}"

        messages.append({"role": "assistant", "content": response_content})
        if message is None:
            self.messages = messages
        return {'response': response_content, 'messages': messages.copy()}

    def ollama_infer(self, messages, image_path=None, video_path=None, client=None, model=None):
        """Infer using Ollama, supporting tool calls and video analysis. / 使用Ollama推理，支持工具调用和视频分析。"""
        client = client or self.client
        model = model or self.ollama_model
        if not client:
            return "Error: Ollama client not initialized"

        if image_path:
            image_data = self.encode_file_to_base64(image_path)
            messages[-1]['images'] = [image_data]
        elif video_path:
            # For videos, extract keyframes for analysis / 对于视频，提取关键帧进行分析
            print(f"Starting to extract video frames: {video_path}")
            frame_images = self._extract_video_frames(video_path)
            if frame_images:
                print(f"Successfully extracted {len(frame_images)} video frames")
                messages[-1]['images'] = frame_images
            else:
                print("Failed to extract video frames")
                return "Error: Failed to extract frames from video"

        # Check if tool call support is needed / 检查是否需要工具调用支持
        # If it's video or image analysis, use normal mode to get a natural language description / 如果是视频或图像分析，使用普通模式获取自然语言描述
        if image_path or video_path:
            try:
                vision_options = {'num_predict': 256, 'temperature': 0.6}
                chat_kwargs = {'model': model, 'messages': messages}
                if client is self.vision_client and self.vision_model_name:
                    # Vision models are the main memory consumers on a Pi 5.
                    # Keep the context small and unload right after the
                    # request so the desktop and Nav2 do not get starved.
                    vision_options['num_ctx'] = self.ollama_vision_num_ctx
                    chat_kwargs['keep_alive'] = self.ollama_vision_keep_alive
                chat_kwargs['options'] = vision_options
                response = client.chat(**chat_kwargs)
                return response['message']['content']
            except Exception as e:
                return f"Ollama multimedia analysis failed: {e}"
        else:
            # For text dialogues, try to use the tool call feature / 文本对话时尝试使用工具调用功能
            try:
                response = client.chat(
                    model=model,
                    messages=messages,
                    format='json',  # Request JSON format output to parse tool calls / 要求JSON格式输出以便解析工具调用
                    options={'num_predict': 256, 'temperature': 0.6},
                )
                return response['message']['content']
            except Exception as e:
                print(f"Ollama tool call failed, falling back to normal mode: {e}")
                # Fallback to normal mode / 回退到普通模式
                try:
                    response = client.chat(model=model, messages=messages, options={'num_predict': 256, 'temperature': 0.6})
                    return response['message']['content']
                except Exception as e2:
                    return f"Ollama inference failed: {e2}"

    def spark_infer(self, messages, image_path=None):
        """Infer using iFlytek Spark. / 使用讯飞星火推理。"""
        from sparkai.core.messages import ChatMessage
        spark_messages = [ChatMessage(role=msg["role"], content=msg["content"]) for msg in messages]
        
        if image_path:
            image_data = self.encode_file_to_base64(image_path)
            last_user_prompt = spark_messages[-1].content
            new_content = [
                {"type": "image", "content": image_data},
                {"type": "text", "content": last_user_prompt}
            ]
            spark_messages[-1].content = new_content
        
        response = self.client.generate([spark_messages])
        if response and response.generations and len(response.generations) > 0 and len(response.generations) > 0:
            return response.generations[0][0].text
        return "Error: Received empty or malformed response from Spark."

    def openai_compatible_infer(self, messages, image_path=None, video_path=None, client=None, model=None):
        """Handle multimedia inference for all OpenAI-compatible platforms. / 处理所有OpenAI兼容平台的多媒体推理。"""
        client = client or self.client
        last_user_message = messages[-1]['content']
        if isinstance(last_user_message, list):
            last_user_prompt = " ".join([item['text'] for item in last_user_message if item['type'] == 'text'])
        else:
            last_user_prompt = last_user_message

        new_content = []

        if image_path:
            base64_media = self.encode_file_to_base64(image_path)
            new_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_media}"}})
        elif video_path:
            base64_media = self.encode_file_to_base64(video_path)
            new_content.append({"type": "video_url", "video_url": {"url": f"data:video/mp4;base64,{base64_media}"}})
        
        new_content.append({"type": "text", "text": last_user_prompt})
        
        # Create a new message list for this request to avoid modifying the original list / 创建一个新的消息列表用于本次请求，以避免修改原始列表
        request_messages = messages[:-1] + [{"role": "user", "content": new_content}]
        
        model_map = {
            'tongyi': self.tongyi_model,
            'qianfan': self.qianfan_model,
            'openrouter': self.openrouter_model,
            'deepseek': self.deepseek_model
        }
        model_to_use = model or model_map.get(self.llm_platform, self.tongyi_model) # Default to Tongyi model / 默认为通义模型

        call_kwargs = {}
        if client is self.client and self.llm_platform == 'deepseek':
            call_kwargs = {'max_tokens': 1024, 'reasoning_effort': 'low'}
        completion = client.chat.completions.create(model=model_to_use, messages=request_messages, **call_kwargs)
        return completion.choices[0].message.content

    def init_oline_asr(self,language):
        self.language=language
        return self.oline_asr_model
        
    def oline_asr(self,input_file):
        if self.oline_asr_model in ['paraformer-realtime-v2','paraformer-realtime-v1','paraformer-realtime-8k-v2','paraformer-realtime-8k-v1']:
            output=self.paraformer_asr_inferce(input_file)
            return output
        elif self.oline_asr_model in ['gummy-realtime-v1','gummy-chat-v1']:
            output=self.gummy_asr_inferce(input_file)
            return output

    def paraformer_asr_inferce(self,input_file):
        recognition = Recognition(model=self.oline_asr_model,
                            format='wav',
                            sample_rate=self.oline_asr_sample_rate,
                            callback=None)
        result = recognition.call(input_file)
        if result.status_code == HTTPStatus.OK:
            sentences = result.get_sentence()
            if sentences and isinstance(sentences, list):
                return ['ok', sentences[0].get('text', '')]
            else:
                return ['error', 'ASR Error: The large model returned an empty result. Please check your account balance or parameter configuration.']
        else: 
            return ['error', 'ASR Error:'+result.message] 

    def gummy_asr_inferce(self,input_file):
        translator = TranslationRecognizerRealtime(
            model=self.oline_asr_model,
            format="wav",
            sample_rate=self.oline_asr_sample_rate,
            translation_target_languages=[self.language],
            translation_enabled=True,
            callback=None,
        )
        result = translator.call(input_file)
        if not result.error_message:
            output=''
            for transcription_result in result.transcription_result_list:
                output+=transcription_result.text
            return ['ok', output]
        else:
            return ['error', result.error_message]

    def init_local_asr_model(self):
        from funasr import AutoModel
        self.model_senceVoice = AutoModel(model=self.local_asr_model, trust_remote_code=False,disable_update=True)

    def tts_model_init(self,model_type='oline',language='zh'):
        if model_type=='oline':
            if self.tts_supplier=='baidu':
                self.token=self.fetch_token()
    
            self.model_type='oline'      
        elif model_type=='local':
            import piper
            self.model_type='local'
            if language=='zh':
                tts_model=self.zh_tts_model
                tts_json=self.zh_tts_json
            elif language=='en':
                tts_model=self.en_tts_model
                tts_json=self.en_tts_json
            self.synthesizer = piper.PiperVoice.load(tts_model, config_path=tts_json, use_cuda=False)      

        elif model_type == "XUNFEI_FOR_INTERNATIONAL":
            self.model_type = "XUNFEI_FOR_INTERNATIONAL"

    def SenseVoiceSmall_ASR(self, input_file,language='zn'):
        res = self.model_senceVoice.generate(
            input=input_file,
            cache={},
            language=language,
            use_itn=False,
        )
        prompt = res[0]['text'].split(">")[-1]
        return ['ok', prompt]

    def voice_synthesis(self,text,path):
        if self.model_type=='oline':
            if self.tts_supplier=='baidu':
                TTS_URL = 'http://tsn.baidu.com/text2audio'
                tex = quote_plus(text)  
                params = {'tok': self.token, 'tex': tex, 'per': self.PER, 'spd': self.SPD, 'pit': self.PIT, 'vol': self.VOL, 'aue': 3, 'cuid': self.CUID,
                            'lan': 'zh', 'ctp': 1}

                data = urlencode(params)
                req = Request(TTS_URL, data.encode('utf-8'))
                try:
                    f = urlopen(req)
                    result_str = f.read()
                except  URLError as err:
                    print('asr http response http code : ' + str(err.code))
                    result_str = err.read()
                    return  1
                with open(path, 'wb') as of:
                    of.write(result_str)
                    return 0
            
            elif self.tts_supplier=='aliyun':
                self.synthesizer = SpeechSynthesizer(model= self.oline_tts_model, voice=self.voice_tone,volume=100)
                audio = self.synthesizer.call(text)
                if audio is None:
                    return 1
                else:
                    with open(path, 'wb') as f:
                        f.write(audio)  
                    return 0                                 
        elif self.model_type=='local':
            with wave.open(path, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(self.synthesizer.config.sample_rate)
                self.synthesizer.synthesize(text, wav_file)    

        elif self.model_type == "XUNFEI_FOR_INTERNATIONAL":
            Xinghou_speaktts(text)       

    def fetch_token(self):
        TOKEN_URL = 'http://aip.baidubce.com/oauth/2.0/token'
        SCOPE = 'audio_tts_post'
        params = {'grant_type': 'client_credentials',
                'client_id': self.baidu_API_KEY,
                'client_secret': self.baidu_SECRET_KEY}
        post_data = urlencode(params)
        post_data = post_data.encode('utf-8')
        req = Request(TOKEN_URL, post_data)
        try:
            f = urlopen(req, timeout=5)
            result_str = f.read()
        except URLError as err:
            print('token http response http code : ' + str(err.code))
            result_str = err.read()
        result_str = result_str.decode()
        result = json.loads(result_str)
        if ('access_token' in result.keys() and 'scope' in result.keys()):
            return result['access_token']

    def _extract_video_frames(self, video_path, max_frames=5):
        """Extract keyframes from a video for analysis. / 从视频中提取关键帧用于分析。"""
        try:
            import cv2
            import tempfile

            cap = cv2.VideoCapture(video_path)
            if not cap.isOpened():
                print(f"Error: Cannot open video file {video_path}")
                return None

            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            if total_frames == 0:
                print(f"Error: Video file {video_path} has no frames")
                cap.release()
                return None

            # Calculate the interval for frame extraction / 计算要提取的帧的间隔
            frame_interval = max(1, total_frames // max_frames)
            frame_images = []

            frame_count = 0
            extracted_count = 0

            while extracted_count < max_frames:
                ret, frame = cap.read()
                if not ret:
                    break

                # Extract one frame every frame_interval frames / 每隔frame_interval帧提取一帧
                if frame_count % frame_interval == 0:
                    # Save the frame to a temporary file / 保存帧到临时文件
                    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
                        temp_path = temp_file.name
                        cv2.imwrite(temp_path, frame)

                        # Encode to base64 / 编码为base64
                        frame_base64 = self.encode_file_to_base64(temp_path)
                        frame_images.append(frame_base64)

                        # Clean up the temporary file / 清理临时文件
                        os.unlink(temp_path)

                        extracted_count += 1

                frame_count += 1

            cap.release()
            print(f"Successfully extracted {len(frame_images)} frames from video")
            return frame_images

        except Exception as e:
            print(f"Error extracting frames from video {video_path}: {e}")
            return None

    @staticmethod
    def encode_file_to_base64(file_path):
        with open(file_path, "rb") as file:
            return base64.b64encode(file.read()).decode("utf-8")

# 录完音，可以直接调用去识别 After recording the audio, it can be directly called for recognition
def rec_wav_music_en():
    global xufei, wsParam
    xufei = ""
    # time1 = datetime.now()
    wsParam = Ws_Param(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        AudioFile=record_speech_file,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message, on_error=on_error, on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

    return xufei


def Xinghou_speaktts(context):
    global wsParam
    # 测试时候在此处正确填写相关信息即可运行 Fill in the relevant information correctly here during testing to run
    wsParam = Ws_Param_1(
        APPID="f12672f1",
        APISecret="NmUyYTRmNTM2MjE3OWJkMDczYzlhZDgz",
        APIKey="8c7b9858dc5e11e8490ce0d09879ad1e",
        Text=context,
    )
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url_1()
    ws = websocket.WebSocketApp(
        wsUrl, on_message=on_message_1, on_error=on_error_1, on_close=on_close_1
    )
    ws.on_open = on_open_1
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})