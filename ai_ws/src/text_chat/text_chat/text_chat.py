import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import readline
import os
import atexit
import termios
import tty

class TextChatNode(Node):
    def __init__(self):
        super().__init__('text_chat_node')
        # Publisher to send user input to the model_service (simulating ASR)
        self.asr_publisher = self.create_publisher(String, 'asr', 10)
        
        # Subscriber to receive text responses from the model_service
        self.response_subscriber = self.create_subscription(
            String,
            'text_response',
            self.response_callback,
            10)
            
        self.get_logger().info('Text Chat Node has been started.')
        self.get_logger().info("Enter your message below (type 'exit' or 'quit' to stop).")
        
        # [最终修复] 恢复锁，以支持线程安全的打印
        self.print_lock = threading.Lock()

    def response_callback(self, msg):
        """Callback for handling messages from the model_service."""
        # [最终修复] 恢复锁
        with self.print_lock:
            # Platform-agnostic way to handle readline buffer
            buffer = ""
            if hasattr(readline, "get_line_buffer"):
                # This works on GNU/Linux
                buffer = readline.get_line_buffer()
            
            # Clear the current line and move cursor to the beginning
            # \r: carriage return, \x1b[K: clear line from cursor to end
            sys.stdout.write('\r\x1b[K')
            
            # Print the model's response
            print(f"Model: {msg.data}")
            
            # Restore the input prompt and the user's typed buffer
            sys.stdout.write("> " + buffer)
            sys.stdout.flush()

    # [最终修复] 恢复 input_loop 线程，以利用 readline 的全部功能
    def input_loop(self):
        """Handles user input in a dedicated thread."""
        while rclpy.ok():
            try:
                # 使用 input() 来获得完整的 readline 功能
                user_input = input() # prompt is handled by the callback
                
                # 在接收到输入后，立即打印新的提示符，以获得更好的交互感
                with self.print_lock:
                    sys.stdout.write("> ")
                    sys.stdout.flush()

                if user_input.lower() in ['exit', 'quit']:
                    self.get_logger().info("Exit command received. Shutting down.")
                    # [核心修改] 输入线程只负责发出关闭信号
                    if rclpy.ok():
                        rclpy.shutdown()
                    break

                if user_input:
                    msg = String()
                    msg.data = user_input
                    self.asr_publisher.publish(msg)

            except (KeyboardInterrupt, EOFError):
                self.get_logger().info("Input interrupted. Shutting down.")
                if rclpy.ok():
                    rclpy.shutdown()
                break

def main(args=None):
    # [最终的、决定性的修复]
    # 保存原始终端设置
    original_terminal_settings = termios.tcgetattr(sys.stdin)

    # 注册一个atexit函数，确保无论如何都能恢复终端
    def restore_terminal():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
        print("\nTerminal settings restored.")

    atexit.register(restore_terminal)

    rclpy.init(args=args)
    text_chat_node = TextChatNode()
    
    # 打印初始提示
    sys.stdout.write("> ")
    sys.stdout.flush()

    # 启动输入线程
    input_thread = threading.Thread(target=text_chat_node.input_loop)
    input_thread.daemon = True
    input_thread.start()
    
    try:
        # 主线程进入 spin()，它会一直运行直到 rclpy.shutdown() 被调用
        rclpy.spin(text_chat_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass # 预期内的干净退出
    finally:
        # 在spin结束后，节点实际上已经失效，我们只需要确保rclpy完全关闭
        if rclpy.ok():
             rclpy.shutdown()
        text_chat_node.get_logger().info("RCLPY shutdown complete.")

if __name__ == '__main__':
    main()