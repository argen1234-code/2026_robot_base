import os
import math
import cv2
import re
import threading
from urllib.request import urlretrieve
from datetime import datetime
from utils import person_locator
from utils.tool_chain_manager import ToolChainManager
from utils.tool_adapters import (
    SummonUserToolAdapter, SeeWhatToolAdapter, AnalyzeVideoToolAdapter,
    WriteDocumentToolAdapter, GenerateImageToolAdapter, ScanTableToolAdapter,
    VisualPositioningToolAdapter
)
# Data passing is now handled by a generic mechanism. / 数据传递现已由通用机制处理。

class ToolsManager:
    """
    Manages all tool operations and integrations.
    管理所有工具操作和集成。
    """
    
    def __init__(self, node):
        """
        Initialize ToolsManager with ROS2 node reference.
        使用ROS2节点引用初始化工具管理器。
        
        :param node: LargeModelService node instance.
        """
        self.node = node

        # Serializes access to the shared OpenCV capture.  The summon task
        # runs in a worker thread, so a plain seewhat() call must not read the
        # same VideoCapture concurrently.
        self.camera_lock = threading.Lock()

        # Initialize the new tool chain manager. / 初始化新的工具链管理器。
        self.tool_chain_manager = ToolChainManager(logger=node.get_logger())
        self._setup_tool_chain()

    def _setup_tool_chain(self):
        """
        Register all tool adapters with tool chain manager.
        注册所有工具适配器到工具链管理器。
        """
        # Register tool adapters. / 注册工具适配器。
        self.tool_chain_manager.register_tool(SummonUserToolAdapter(self))
        self.tool_chain_manager.register_tool(SeeWhatToolAdapter(self))
        self.tool_chain_manager.register_tool(AnalyzeVideoToolAdapter(self))
        self.tool_chain_manager.register_tool(WriteDocumentToolAdapter(self))
        self.tool_chain_manager.register_tool(GenerateImageToolAdapter(self))
        self.tool_chain_manager.register_tool(ScanTableToolAdapter(self))
        self.tool_chain_manager.register_tool(VisualPositioningToolAdapter(self))


    def execute_tool_chain(self, tool_calls):
        """
        Execute multiple tool calls through tool chain manager.
        通过工具链管理器执行多个工具调用。
        
        :param tool_calls: List of tool calls to execute.
        :return: List of tool outputs.
        """
        return self.tool_chain_manager.execute_tool_chain(tool_calls)

    def seewhat(self):
        """
        Capture camera frame and analyze environment with AI model.
        捕获摄像头画面并使用AI模型分析环境。
        
        :return: Dictionary with scene description and image path, or None if failed.
        """
        self.node.get_logger().info("Executing seewhat() tool")
        image_path = self.capture_frame()
        if image_path:
            # Use isolated context for image analysis. / 使用隔离的上下文进行图像分析。
            analysis_text = self._get_actual_scene_description(image_path)

            # Return structured data for the tool chain. / 为工具链返回结构化数据。
            return {
                "description": analysis_text,
                "image_path": image_path
            }
        else:
            if self.node.language == 'zh':
                error_msg = "捕获摄像头画面失败"
            else:
                error_msg = "Failed to capture camera frame"
            self.node.get_logger().error(error_msg)
            print(f"\n{error_msg}\n")
            return None

    def _get_actual_scene_description(self, image_path, message_context=None):
        """
        Get AI-generated scene description for captured image.
        获取捕获图像的AI生成场景描述。
        
        :param image_path: Path to captured image file.
        :return: Plain text description of scene.
        """
        try:
            # Use a mandatory plain text description prompt. / 使用强制的纯文本描述提示。
            if self.node.language == 'zh':
                scene_prompt = "请用纯文本详细描述这张图片。包括：环境、人物、物体、颜色、光线。不要使用JSON、不要调用工具、不要其他格式，只要文字描述。请将你的回复内容限制在100个字以内。"
            else:
                scene_prompt = "Please describe this image in plain text only. Include: environment, people, objects, colors, lighting. Do not use JSON, do not call tools, do not use other formats, just text description. Keep your response under 100 words."

            # 移除重复的角色设定，使用系统级别的角色设定
            # 直接使用模型客户端的默认消息上下文
            result = self.node.model_client.infer_with_image(image_path, scene_prompt)

            if isinstance(result, dict):
                description = result.get('response', '')
                # If JSON format is still returned, try to extract descriptive content. / 如果仍然返回JSON格式，则尝试提取描述性内容。
                if description.startswith('{') or '```json' in description:
                    self.node.get_logger().warning("Large model still returns JSON format, attempting to extract description content")
                    # Retry with a simpler prompt. / 使用更简单的提示重试。
                    simple_prompt = "描述这张图片中你看到的内容。"
                    retry_result = self.node.model_client.infer_with_image(image_path, simple_prompt)
                    if isinstance(retry_result, dict):
                        description = retry_result.get('response', 'Failed to get scene description')
                    else:
                        description = str(retry_result)

                return description
            else:
                return str(result)

        except Exception as e:
            self.node.get_logger().error(f"Failed to get scene description: {e}")
            return "Failed to get scene description"

    def capture_frame(self):
        """Thread-safe wrapper around the actual capture."""
        with self.camera_lock:
            return self._capture_frame_unlocked()

    def _capture_frame_unlocked(self):
        """
        Capture frame from camera and save to file.
        从摄像头捕获画面并保存到文件。
        
        :return: Path to saved image file, or None if failed.
        """
        if not self.node.camera_initialized or self.node.cap is None:
            if self.node.language == 'zh':
                error_msg = "摄像头未初始化或未就绪，无法捕获画面。"
            else:
                error_msg = "Camera not initialized or not ready, unable to capture frame."
            self.node.get_logger().error(error_msg)
            return None

        # To get the latest frame, we must clear the internal buffer of VideoCapture by reading out old frames.
        # We read frames for a short, fixed duration (e.g., 0.5 seconds) to discard them.
        self.node.get_logger().info("Clearing camera buffer to get the latest frame...")
        import time
        start_time = time.time()
        
        # Keep reading for a short period to flush the buffer
        # The last successfully read frame will be the latest one.
        ret, frame = False, None
        while (time.time() - start_time) < 0.5: # Clear buffer for 0.5 seconds
            read_ok, current_frame = self.node.cap.read()
            if read_ok:
                ret, frame = True, current_frame
            else:
                # Stop if we can't read from the stream
                break

        if not ret:
            self.node.get_logger().error("Failed to capture any frame from camera.")
            return None

        self.node.get_logger().debug("Successfully captured fresh frame from camera")

        self.node.last_frame = frame.copy()

        # No camera preview window: cv2.imshow() can take >10s to
        # initialize on Wayland/container sessions and is not needed
        # for voice interaction.

        resources_dir = os.path.join(self.node.pkg_path, "resources_file")
        os.makedirs(resources_dir, exist_ok=True)

        # Use a timestamp in the filename to avoid caching issues. / 在文件名中使用时间戳以避免缓存问题。
        import time
        timestamp = int(time.time() * 1000)  # Millisecond timestamp. / 毫秒级时间戳。
        temp_image_path = os.path.join(resources_dir, f"captured_frame_{timestamp}.jpg")

        try:
            success = cv2.imwrite(temp_image_path, self.node.last_frame)
            if not success:
                self.node.get_logger().error(f"Failed to save frame to {temp_image_path}")
                return None

            # Validate image content to ensure a new frame was captured. / 验证图像内容以确保捕获了新画面。
            frame_hash = self._calculate_frame_hash(self.node.last_frame)
            self.node.get_logger().info(f"Frame captured and saved to {temp_image_path}, hash: {frame_hash[:8]}")

            # Clean up old image files to save disk space. / 清理旧图像文件以节省磁盘空间。
            self._cleanup_old_frames(resources_dir)

        except Exception as e:
            self.node.get_logger().error(f"Exception while saving frame: {e}")
            return None

        return temp_image_path

    def _cleanup_old_frames(self, resources_dir):
        """
        Clean up old image files, keep only recent 5 files.
        清理旧的图像文件，只保留最近的5个文件。
        """
        try:
            import glob
            import os

            # Find all captured_frame_*.jpg files. / 查找所有captured_frame_*.jpg文件。
            pattern = os.path.join(resources_dir, "captured_frame_*.jpg")
            frame_files = glob.glob(pattern)

            # Sort by modification time and keep the 5 newest files. / 按修改时间排序，保留最新的5个文件。
            if len(frame_files) > 5:
                frame_files.sort(key=os.path.getmtime)
                files_to_delete = frame_files[:-5]  # Delete all except the 5 newest ones. / 除了最新的5个，其他都删除。

                for file_path in files_to_delete:
                    try:
                        os.remove(file_path)
                        self.node.get_logger().debug(f"Cleaned up old frame: {file_path}")
                    except Exception as e:
                        self.node.get_logger().warning(f"Failed to delete old frame {file_path}: {e}")

        except Exception as e:
            self.node.get_logger().warning(f"Failed to cleanup old frames: {e}")

    def _calculate_frame_hash(self, frame):
        """
        Calculate frame hash to verify image content changes.
        计算图像帧的哈希值，用于验证图像内容是否发生变化。
        
        :param frame: Image frame data.
        :return: MD5 hash of frame data.
        """
        try:
            import hashlib
            # Convert the image to a byte string and calculate the MD5 hash. / 将图像转换为字节串并计算MD5哈希。
            frame_bytes = frame.tobytes()
            hash_md5 = hashlib.md5(frame_bytes).hexdigest()
            return hash_md5
        except Exception as e:
            self.node.get_logger().warning(f"Failed to calculate frame hash: {e}")
            return "unknown"

    # ------------------------------------------------------------------
    # Summon / person localization
    # ------------------------------------------------------------------
    def _lookup_tf(self, target_frame, source_frame, stamp=None):
        """
        Return (translation_xyz, quaternion_xyzw) of source in target.
        The caller decides which timestamp to use; using the image timestamp
        is important because VLM inference can take several seconds.
        """
        from rclpy.duration import Duration
        from rclpy.time import Time

        if stamp is None:
            when = Time()
        else:
            when = Time.from_msg(stamp)

        transform = self.node.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            when,
            timeout=Duration(seconds=0.5),
        )
        t = transform.transform.translation
        q = transform.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def _detect_single_person(self, image_path):
        """Ask the multimodal model for exactly one person box."""
        image = cv2.imread(image_path)
        if image is None:
            return None, "cannot_read_image"

        height, width = image.shape[:2]
        prompt = (
            f"图像尺寸为{width}x{height}像素。找出图中的所有人，"
            '仅返回JSON：{"boxes":[[x1,y1,x2,y2]]}。'
            "坐标使用原图像素；没有人时返回空数组。"
            "不要输出JSON以外的任何文字。"
        )

        result = self.node.model_client.infer_with_image(
            image_path,
            prompt,
            message=[{"role": "system", "content": "只输出所要求的JSON。"}],
        )
        if isinstance(result, dict):
            raw = result.get("response", "")
        else:
            raw = str(result)

        box = person_locator.parse_single_box(raw, width, height)
        if box is None:
            self.node.get_logger().warning(
                f"Person detection rejected or ambiguous: {str(raw)[:300]}"
            )
            return None, "no_single_person"

        return box, None

    def locate_user(self):
        """
        Detect the user, turn the detection into a map-frame standoff goal,
        and return structured data for the navigation task node.

        Range priority:
          1. 2D LiDAR at the visual bearing (most reliable indoors).
          2. Monocular ground-plane projection (flat floor assumption).

        Nothing is returned as successful unless TF, image, and detection are
        all valid; the caller must refuse to move otherwise.
        """
        node = self.node
        self.node.get_logger().info("Locating user for a summon task")

        image_path = self.capture_frame()
        if not image_path:
            return {"success": False, "reason": "capture_failed"}

        # Use the capture time for every TF lookup below.  The VLM may take
        # several seconds, but the scene (and the user's position) correspond
        # to this instant.
        image_stamp = node.get_clock().now().to_msg()

        box, error = self._detect_single_person(image_path)
        if box is None:
            return {
                "success": False,
                "reason": error or "no_single_person",
                "image_path": image_path,
            }

        bearing = person_locator.bearing_from_box(
            box, node.camera_fx, node.camera_cx
        )
        self.node.get_logger().info(
            f"Single person box={box}, bearing={math.degrees(bearing):.1f} deg"
        )

        try:
            robot_xyz, _ = self._lookup_tf(
                node.map_frame, node.base_frame, image_stamp
            )
        except Exception as exc:
            return {
                "success": False,
                "reason": f"tf_base_unavailable: {exc}",
                "image_path": image_path,
            }

        person_xyz = None
        range_source = None

        want_lidar = node.range_source in ("auto", "lidar") and node.enable_lidar_fusion
        want_ground = node.range_source in ("auto", "ground")

        if want_lidar:
            scan = node.get_latest_scan(max_age_s=1.0)
            if scan is not None:
                try:
                    lidar_result = person_locator.lidar_point_for_bearing(
                        scan=scan,
                        bearing_camera=bearing,
                        tf_lookup=self._lookup_tf,
                        camera_frame=node.camera_frame,
                        laser_frame=node.laser_frame,
                        stamp=scan.header.stamp,
                        window_deg=node.lidar_range_window_deg,
                        min_range_m=node.lidar_min_range_m,
                        max_range_m=node.max_user_distance_m,
                    )
                except Exception as exc:
                    self.node.get_logger().warning(
                        f"LiDAR range estimation failed: {exc}"
                    )
                    lidar_result = None

                if lidar_result is not None:
                    point_laser, measured_range = lidar_result
                    try:
                        laser_xyz, laser_quat = self._lookup_tf(
                            node.map_frame, node.laser_frame, scan.header.stamp
                        )
                        rotated = person_locator.quat_rotate(laser_quat, point_laser)
                        person_xyz = (
                            laser_xyz[0] + rotated[0],
                            laser_xyz[1] + rotated[1],
                            0.0,
                        )
                        range_source = f"lidar ({measured_range:.2f} m)"
                        self.node.get_logger().info(
                            f"LiDAR range at person bearing: {measured_range:.2f} m"
                        )
                    except Exception as exc:
                        self.node.get_logger().warning(
                            f"LiDAR point transform failed: {exc}"
                        )
                        person_xyz = None

        if person_xyz is None and want_ground:
            try:
                camera_xyz, camera_quat = self._lookup_tf(
                    node.map_frame, node.camera_frame, image_stamp
                )
                person_xyz = person_locator.ground_point_from_box(
                    box=box,
                    fx=node.camera_fx,
                    fy=node.camera_fy,
                    cx=node.camera_cx,
                    cy=node.camera_cy,
                    camera_xyz=camera_xyz,
                    camera_quat=camera_quat,
                    max_range_m=node.max_user_distance_m,
                )
                if person_xyz is not None:
                    range_source = "ground_plane"
                    self.node.get_logger().info(
                        "Ground-plane range estimate: "
                        f"person=({person_xyz[0]:.2f}, {person_xyz[1]:.2f})"
                    )
            except Exception as exc:
                self.node.get_logger().warning(
                    f"Ground-plane projection failed: {exc}"
                )

        if person_xyz is None:
            return {
                "success": False,
                "reason": "range_unavailable",
                "image_path": image_path,
                "box": box,
                "bearing": bearing,
            }

        distance = math.hypot(person_xyz[0] - robot_xyz[0],
                              person_xyz[1] - robot_xyz[1])
        if distance > node.max_user_distance_m:
            return {
                "success": False,
                "reason": "user_too_far",
                "image_path": image_path,
                "distance": distance,
            }

        goal = person_locator.make_standoff_goal(
            person_xyz, robot_xyz, node.standoff_distance_m
        )

        data = {
            "success": True,
            "image_path": image_path,
            "box": box,
            "bearing": bearing,
            "range_source": range_source,
            "person_xyz": person_xyz,
            "robot_xyz": robot_xyz,
            "distance": distance,
        }

        if goal is None:
            data.update({"success": False, "reason": "already_close"})
            return data

        data["goal"] = goal
        self.node.get_logger().info(
            f"Summon goal in {node.map_frame}: "
            f"x={goal['x']:.2f}, y={goal['y']:.2f}, "
            f"yaw={math.degrees(goal['yaw']):.1f} deg, "
            f"distance={distance:.2f} m, range_source={range_source}"
        )
        return data

    def generate_image(self, args):
        """
        Generate image from text prompt and save to local file.
        根据文本提示生成图像并保存到本地文件。
        
        :param args: Arguments containing prompt and other parameters.
        :return: Dictionary with status and image paths.
        """
        self.node.get_logger().info(f"Executing generate_image() tool with args: {args}")
        try:
            prompt = args.get("prompt") if isinstance(args, dict) else args
            if not prompt:
                self.node.get_logger().error("Missing 'prompt' argument for generate_image.")
                return {'status': 'failed', 'error': 'Missing prompt argument'}

            # 1. Call the text-to-image service to get the result. / 1. 调用文生图服务获取结果。
            api_result = self.node.model_client.text_to_image(prompt)

            # 2. Check if the API call was successful and get the URL. / 2. 检查API调用是否成功，并获取URL。
            if not isinstance(api_result, dict) or api_result.get('status') != 'success':
                error_msg = api_result.get('error', 'Failed to generate image from API')
                self.node.get_logger().error(f"Image generation API failed: {error_msg}")
                return {'status': 'failed', 'error': error_msg}

            image_urls = api_result.get("image_urls", [])
            if not image_urls:
                self.node.get_logger().error("API returned success but no image URLs found.")
                return {'status': 'failed', 'error': 'No image URLs in API response'}

            # 3. Download and save the image. / 3. 下载并保存图片。
            saved_paths = []
            image_dir = os.path.join(self.node.pkg_path, "resources_file", "generated_images")
            os.makedirs(image_dir, exist_ok=True)

            for i, url in enumerate(image_urls):
                try:
                    # Create a filename. / 创建一个文件名。
                    safe_prompt = re.sub(r'[\\/*?:"<>|]', "", prompt)[:50]
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"{safe_prompt}_{timestamp}_{i+1}.png"
                    save_path = os.path.join(image_dir, filename)

                    # Download image from URL. / 从URL下载图片。
                    urlretrieve(url, save_path)
                    self.node.get_logger().info(f"Image successfully downloaded and saved to: {save_path}")
                    saved_paths.append(save_path)

                    # Immediately display the first successfully downloaded image. / 成功下载第一张图片后立即显示它。
                    if len(saved_paths) == 1:
                        try:
                            generated_image = cv2.imread(save_path)
                            if generated_image is not None:
                                cv2.imshow("Generated Image", generated_image)
                                cv2.waitKey(1)
                        except Exception as display_error:
                            self.node.get_logger().error(f"Failed to display image {save_path}: {display_error}")

                except Exception as download_error:
                    self.node.get_logger().error(f"Failed to download image from {url}: {download_error}")
                    continue # If one image fails to download, continue with the next one. / 如果一张图片下载失败，继续尝试下一张。

            if not saved_paths:
                return {'status': 'failed', 'error': 'Image generation succeeded, but all downloads failed.'}

            # 4. Return a success result containing the local path. / 4. 返回包含本地路径的成功结果。
            if self.node.language == 'zh':
                message = f"成功生成并保存了 {len(saved_paths)} 张图片。"
            else:
                message = f"Successfully generated and saved {len(saved_paths)} image(s)."
                
            return {
                'status': 'success',
                'image_urls': image_urls,
                'saved_paths': saved_paths,
                'message': message
            }

        except Exception as e:
            self.node.get_logger().error(f"Failed to execute generate_image tool: {e}")
            if self.node.language == 'zh':
                error_msg = f"执行图片生成工具失败: {str(e)}"
            else:
                error_msg = f'generate_image tool execution failed: {str(e)}'
            return {'status': 'failed', 'error': error_msg}

    def analyze_video(self, args):
        """
        Analyze video file and provide content description.
        分析视频文件并提供内容描述。
        
        :param args: Arguments containing video path.
        :return: Dictionary with video description and path.
        """
        self.node.get_logger().info(f"Executing analyze_video() tool with args: {args}")
        try:
            video_path = args.get("video_path") if isinstance(args, dict) else args

            # Smart path fallback mechanism. / 智能路径回退机制。
            if not video_path or not os.path.exists(os.path.expanduser(video_path)):
                self.node.get_logger().warn(f"Video path not specified or invalid ('{video_path}').")
                default_path = os.path.join(self.node.pkg_path, "resources_file", "analyze_video", "test_video.mp4")
                if os.path.exists(default_path):
                    video_path = default_path
                    self.node.get_logger().info(f"Fallback to default test video for analysis: {video_path}")
                else:
                    error_msg = f"未指定或无效的视频路径，且默认测试视频也不存在于: {default_path}"
                    self.node.get_logger().error(f"{error_msg}")
                    return {"description": None, "video_path": video_path, "error": error_msg}
            else:
                video_path = os.path.expanduser(video_path)

            # After the fallback check, the path should be valid. / 经过回退检查后，路径应该是有效的。
            if video_path and os.path.exists(video_path):
                # Set different prompts based on language to enforce plain text output. / 根据语言设置不同的提示以强制纯文本输出。
                if self.node.language == 'zh':
                    prompt = "请用纯文本详细描述这个视频的内容。不要使用JSON、不要调用工具、不要其他格式，只要文字描述。请将你的回复内容限制在100个字以内。"
                else:
                    prompt = "Please describe the video content in plain text. Do not use JSON, do not call tools, and do not use other formats—just text. Keep your response under 100 words."

                # Use a fully isolated, one-time context for video analysis to ensure a plain text description. / 使用完全隔离的一次性上下文进行视频分析，以确保获得纯文本描述。
                simple_context = [{
                    "role": "system",
                    "content": "You are a video description assistant. You MUST respond with plain text descriptions only. Never use JSON format, never call tools. Only provide natural language descriptions of what you see in the video."
                }]
                
                result = self.node.model_client.infer_with_video(video_path, prompt, message=simple_context)
                
                if isinstance(result, dict):
                    description = result.get('response', '')
                else:
                    description = str(result)
                
                return {
                    "description": description,
                    "video_path": video_path
                }
            elif video_path:
                error_msg = f"Video file not found: {video_path}"
                self.node.get_logger().error(f" {error_msg}")
                return {"description": None, "video_path": video_path, "error": error_msg}
            else:
                error_msg = "Missing video_path argument"
                self.node.get_logger().error(f" {error_msg}")
                return {"description": None, "video_path": None, "error": error_msg}
        except Exception as e:
            self.node.get_logger().error(f"Failed to execute analyze_video tool: {e}")
            if self.node.language == 'zh':
                error_msg = f"执行视频分析工具失败: {str(e)}"
            else:
                error_msg = f"Failed to execute analyze_video tool: {str(e)}"
            return {"description": None, "video_path": None, "error": error_msg}

    def visual_positioning(self, args):
        """
        Locate object coordinates in image and save results to MD file.
        定位图像中物体坐标并将结果保存为MD文件。
        
        :param args: Arguments containing image path and object name.
        :return: Dictionary with file path and coordinate data.
        """
        self.node.get_logger().info(f"Executing visual_positioning() tool with args: {args}")
        try:
            # Get parameters. / 获取参数。
            image_path = args.get("image_path") if isinstance(args, dict) else None
            object_name = args.get("object_name") if isinstance(args, dict) else None

            # Smart path fallback mechanism. / 智能路径回退机制。
            if not image_path or not os.path.exists(os.path.expanduser(image_path)):
                self.node.get_logger().warn(f" Image path not specified or invalid ('{image_path}').")
                default_path = os.path.join(self.node.pkg_path, "resources_file", "visual_positioning", "test_image.jpg")
                if os.path.exists(default_path):
                    image_path = default_path
                    self.node.get_logger().info(f" Fallback to default test image for visual positioning: {image_path}")
                else:
                    error_msg = f"未指定或无效的图片路径，且默认测试图片也不存在于: {default_path}"
                    self.node.get_logger().error(f" {error_msg}")
                    return {"file_path": None, "coordinates_content": None, "explanation_content": None, "error": error_msg}
            else:
                # Resolve the ~ symbol to the actual path. / 解析~符号为实际路径。
                image_path = os.path.expanduser(image_path)
            
            if not object_name:
                self.node.get_logger().error("Missing 'object_name' argument for visual positioning.")
                return
            
            # Construct a prompt asking the large model to identify the coordinates of the specified object. / 构造提示，要求大模型识别指定物品的坐标。
            if self.node.language == 'zh':
                prompt = f"请仔细分析这张图片，用一个个框定位图像每一个{object_name}的位置并描述其各自的特征。请为每一个{object_name}单独一行输出坐标，严格按照[中心点x,中心点y,宽度,高度]的格式，不要包含任何其他文字。例如：\n[100,200,50,80]\n[150,250,60,90]。请在坐标前添加对每个{object_name}的简要描述。"
            else:
                prompt = f"Please carefully analyze this image and find the position of all {object_name}. Output coordinates for each person on a separate line, strictly in the format [center_x,center_y,width,height]. Do not include any other text. For example:\n[100,200,50,80]\n[150,250,60,90]. Please add a brief description of each {object_name} before the coordinates."
            
            # Call the large model for visual positioning. / 调用大模型进行视觉定位。
            # Use an independent message context to avoid influence from historical image analysis. / 使用独立的消息上下文，以避免历史图像分析的影响。
            message_to_use = [{
                "role": "system",
                "content": "You are a visual positioning assistant. Your task is to analyze images and provide precise coordinates for specified objects. Always respond with coordinates in the format [center_x,center_y,width,height] and provide clear descriptions."
            }]
            result = self.node.model_client.infer_with_image(image_path, prompt, message=message_to_use)
            
            # Process the result - ensure it is a string type. / 处理结果 - 确保是字符串类型。
            if isinstance(result, dict):
                response_text = result.get('response', '')
            else:
                response_text = str(result)  # Ensure conversion to string. / 确保转换为字符串。
            
            # Ensure response_text is a string. / 确保response_text是字符串。
            if not isinstance(response_text, str):
                response_text = str(response_text)
            
            # Separate coordinates and explanation information. / 分离坐标和解释信息。
            lines = response_text.split('\n')
            coordinates_lines = []
            explanation_lines = []
            
            # Extract coordinate lines. / 提取坐标行。
            for line in lines:
                line_str = str(line)  # Ensure it is a string. / 确保是字符串。
                if re.search(r'\[\s*\d+\s*,\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\]', line_str):
                    coordinates_lines.append(line_str)
                else:
                    explanation_lines.append(line_str)
            
            # Organize content. / 组织内容。
            if self.node.language == 'zh':
                no_coordinates_msg = "未检测到有效坐标信息"
                no_explanation_msg = "无额外解释信息"
            else:
                no_coordinates_msg = "No valid coordinate information detected"
                no_explanation_msg = "No additional explanation information"
                
            coordinates_content = "\n".join(coordinates_lines) if coordinates_lines else no_coordinates_msg
            explanation_content = "\n".join(explanation_lines) if explanation_lines else no_explanation_msg
            
            # Save the result as an MD file. / 将结果保存为MD文件。
            # Save in the same directory as the image. / 与图片保存在同一目录下。
            image_dir = os.path.dirname(image_path)
            image_filename = os.path.basename(image_path)
            image_name_without_ext = os.path.splitext(image_filename)[0]
            md_filename = f"{image_name_without_ext}_position.md"
            md_file_path = os.path.join(image_dir, md_filename)
            
            # Ensure all content is a string. / 确保所有内容都是字符串。
            coordinates_content = str(coordinates_content)
            explanation_content = str(explanation_content)
            
            # Modify the part that writes to the MD file to save coordinates and explanations. / 修改写入MD文件的部分，以保存坐标和解释。
            if self.node.language == 'zh':
                title = "# 视觉定位结果"
                target_label = "目标"
                position_label = "位置"
                explanation_label = "## 解释"
                footer = "[中心点x,中心点y,宽度,高度] \n[center_x,center_y,width,height]。"
            else:
                title = "# Visual Positioning Result"
                target_label = "Target"
                position_label = "Position"
                explanation_label = "## Explanation"
                footer = "[center_x,center_y,width,height]"

            with open(md_file_path, 'w', encoding='utf-8') as f:
                f.write(f"{title}\n\n")
                f.write(f"{target_label}: {object_name}\n")
                f.write(f"{position_label}: {coordinates_content}\n")
                if explanation_content.strip():
                    f.write(f"\n{explanation_label}\n\n")
                    f.write(f"{explanation_content}\n")
                f.write(f"\n{footer}\n")
            
            self.node.get_logger().info(f"Visual positioning result saved to: {md_file_path}")

            # Return structured data containing file path and coordinate content. / 返回包含文件路径和坐标内容的结构化数据。
            return {
                "file_path": md_file_path,
                "coordinates_content": coordinates_content,
                "explanation_content": explanation_content
            }
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to execute visual_positioning tool: {e}")
            if self.node.language == 'zh':
                error_msg = f"执行视觉定位工具失败: {str(e)}"
            else:
                error_msg = f"Failed to execute visual_positioning tool: {str(e)}"
            return {"file_path": None, "coordinates_content": None, "explanation_content": None, "error": error_msg}

    def scan_table(self, args):
        """
        Scan table content from image and save to MD file.
        从图像中扫描表格内容并保存到MD文件。
        
        :param args: Arguments containing image path.
        :return: Dictionary with file path and table content.
        """
        self.node.get_logger().info(f"Executing scan_table() tool with args: {args}")
        try:
            # Get parameters. / 获取参数。
            image_path = args.get("image_path") if isinstance(args, dict) else None

            # Smart path fallback mechanism. / 智能路径回退机制。
            if not image_path or not os.path.exists(os.path.expanduser(image_path)):
                self.node.get_logger().warn(f" Image path not specified or invalid ('{image_path}').")
                default_path = os.path.join(self.node.pkg_path, "resources_file", "scan_table", "test_table.jpg")
                if os.path.exists(default_path):
                    image_path = default_path
                    self.node.get_logger().info(f" Fallback to default test table image for scanning: {image_path}")
                else:
                    error_msg = f"未指定或无效的图片路径，且默认测试表格图片也不存在于: {default_path}"
                    self.node.get_logger().error(f" {error_msg}")
                    return {"file_path": None, "table_content": None, "error": error_msg}
            else:
                # Resolve the ~ symbol to the actual path. / 解析~符号为实际路径。
                image_path = os.path.expanduser(image_path)
            
            # Construct a prompt asking the large model to convert the table in the image to Markdown format. / 构造提示，要求大模型将图片中的表格转换为Markdown格式。
            if self.node.language == 'zh':
                prompt = "请仔细分析这张图片，将图片中的表格内容完整地转换为Markdown格式。请确保格式正确，不要包含任何其他无关文字。"
            else:
                prompt = "Please carefully analyze this image and convert the table content into Markdown format. Ensure the format is correct and do not include any other irrelevant text."
            
            # Use the current platform's multimodal capabilities for table recognition. / 使用当前平台的多模态能力进行表格识别。
            self.node.get_logger().info(f"Using {self.node.model_client.llm_platform} platform for table scanning")
            
            # Use the generic multimodal inference interface, supporting all platforms. / 使用通用的多模态推理接口，支持所有平台。
            try:
                # Call the generic image inference interface. / 调用通用的图像推理接口。
                # Use an independent message context to avoid influence from historical image analysis. / 使用独立的消息上下文，以避免历史图像分析的影响。
                message_to_use = [{
                    "role": "system",
                    "content": "You are a table scanning assistant. Your task is to analyze images and convert table content into Markdown format. Provide only the Markdown table content without any additional text."
                }]
                result = self.node.model_client.infer_with_image(image_path, prompt, message=message_to_use)

                # Process the result - ensure it is a string type. / 处理结果 - 确保是字符串类型。
                if isinstance(result, dict):
                    response_text = result.get('response', '')
                else:
                    response_text = str(result)

                # Ensure response_text is a string. / 确保response_text是字符串。
                if not isinstance(response_text, str):
                    response_text = str(response_text)

            except Exception as e:
                self.node.get_logger().error(f"Failed to analyze table with {self.node.model_client.llm_platform}: {e}")
                return
            
            # Save the result as an MD file. / 将结果保存为MD文件。
            image_dir = os.path.dirname(image_path)
            image_filename = os.path.basename(image_path)
            image_name_without_ext = os.path.splitext(image_filename)[0]
            md_filename = f"{image_name_without_ext}_table.md"
            md_file_path = os.path.join(image_dir, md_filename)
            
            if self.node.language == 'zh':
                title = "# 扫描表格结果"
            else:
                title = "# Scanned Table Result"
                
            with open(md_file_path, 'w', encoding='utf-8') as f:
                f.write(f"{title}\n\n")
                f.write(response_text)
            
            self.node.get_logger().info(f"Table scan result saved to: {md_file_path}")

            # Return structured data containing the file path and table content. / 返回包含文件路径和表格内容的结构化数据。
            return {
                "file_path": md_file_path,
                "table_content": response_text
            }
            
        except Exception as e:
            self.node.get_logger().error(f"Failed to execute scan_table tool: {e}")
            if self.node.language == 'zh':
                error_msg = f"执行表格扫描工具失败: {str(e)}"
            else:
                error_msg = f"Failed to execute scan_table tool: {str(e)}"
            return {"file_path": None, "table_content": None, "error": error_msg}

    def write_document(self, arguments):
        """
        Generate and save document in specified format.
        生成并保存指定格式的文档。
        
        :param arguments: Dictionary with filename, content, format, title.
        :return: Dictionary with file path and status.
        """
        try:
            filename = arguments.get("filename", "")
            content = arguments.get("content", "")
            format_type = arguments.get("format", "txt").lower()
            title = arguments.get("title", "文档")

            self.node.get_logger().info(f"Start generating {format_type.upper()} document: {filename}")

            # Generate different document content based on the format - only basic formatting. / 根据格式生成不同的文档内容 - 只做最基本的格式包装。
            if format_type == "md":
                # MD format: only add title and timestamp. / MD格式：只添加标题和时间戳。
                document_content = f"# {title}\n\n{content}\n\n---\n*生成时间: {self._get_current_time()}*"
                if not filename:
                    filename = f"{title.replace(' ', '_')}_{self._get_timestamp()}.md"
            elif format_type == "html":
                # HTML format: only add basic HTML structure. / HTML格式：只添加基本的HTML结构。
                lang_attr = "zh-CN" if self.node.language == 'zh' else "en"
                time_label = "生成时间" if self.node.language == 'zh' else "Generated Time"
                document_content = f"""<!DOCTYPE html>
<html lang="{lang_attr}">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title}</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }}
        .footer {{ margin-top: 40px; font-size: 12px; color: #666; }}
    </style>
</head>
<body>
    {content}
    <div class="footer">{time_label}: {self._get_current_time()}</div>
</body>
</html>"""
                if not filename:
                    filename = f"{title.replace(' ', '_')}_{self._get_timestamp()}.html"
            elif format_type == "json":
                # JSON format: wrap into a JSON object. / JSON格式：包装成JSON对象。
                import json
                data = {
                    "title": title,
                    "content": content,
                    "generated_time": self._get_current_time(),
                    "timestamp": self._get_timestamp()
                }
                document_content = json.dumps(data, ensure_ascii=False, indent=2)
                if not filename:
                    filename = f"{title.replace(' ', '_')}_{self._get_timestamp()}.json"
            else:
                # Default TXT format: plain text. / 默认TXT格式：纯文本。
                document_content = f"{title}\n{'=' * len(title)}\n\n{content}\n\n生成时间: {self._get_current_time()}"
                if not filename:
                    filename = f"{title.replace(' ', '_')}_{self._get_timestamp()}.txt"


            doc_file_path = os.path.join(self.node.pkg_path, "resources_file", "documents",filename)

            with open(doc_file_path, 'w', encoding='utf-8') as f:
                f.write(document_content)

            self.node.get_logger().info(f"{format_type.upper()} document saved to: {doc_file_path}")
            
            # Return structured data including the file path and original content. / 返回包含文件路径和原始内容的结构化数据。
            if self.node.language == 'zh':
                status_message = f"{format_type.upper()}格式的文档已生成并保存到: {doc_file_path}"
            else:
                status_message = f"{format_type.upper()} document generated and saved to: {doc_file_path}"
            
            return {
                "file_path": doc_file_path,
                "status_message": status_message,
                "content": document_content
            }

        except Exception as e:
            self.node.get_logger().error(f"Failed to generate document: {e}")
            if self.node.language == 'zh':
                error_message = f"生成文档失败: {str(e)}"
            else:
                error_message = f"Failed to generate document: {str(e)}"
            return {
                "file_path": None,
                "status_message": error_message
            }

    def _get_current_time(self):
        """Get the current time string. / 获取当前时间字符串。"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _get_timestamp(self):
        """Get the timestamp. / 获取时间戳。"""
        import time
        return str(int(time.time()))
