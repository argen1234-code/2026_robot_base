"""
Tool Adapters
Wraps existing tools into a standardized interface.
/
工具适配器
将现有工具包装成标准化接口。
"""

from typing import Dict, Any, List
from utils.tool_chain_manager import ToolInterface, ToolInput, ToolOutput
import os
import json

class SummonUserToolAdapter(ToolInterface):
    """
    Adapter for the high-level summon task.

    The actual work (VLM detection, TF, Nav2 goal) is performed by
    ``LargeModelService._start_summon_thread`` so that it never blocks the
    ASR callback.  This adapter exists mainly to publish the tool schema to
    the LLM/MCP layer; if it is ever executed directly it refuses rather than
    blocking the executor.
    """

    def __init__(self, tools_manager):
        self.tools_manager = tools_manager

    @property
    def tool_name(self) -> str:
        return "summon_user"

    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": (
                "Call the user to the robot: capture an image, locate the user, "
                "and navigate the mobile base to a safe standoff distance from "
                "the user. Use this when the user asks the robot or trash bin "
                "to come over. Input: none. "
                "Trigger words: 过来, 来一下, 到我这里, 垃圾桶过来, come here."
            ),
            "properties": {},
            "required": []
        }

    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "goal": {"type": "object"},
                "distance": {"type": "number"},
                "range_source": {"type": "string"}
            }
        }

    def execute(self, tool_input: ToolInput) -> ToolOutput:
        return ToolOutput(
            tool_name=self.tool_name,
            success=False,
            data=None,
            metadata={},
            error_message=(
                "summon_user is executed asynchronously by the task layer; "
                "it must not be called through the blocking tool chain"
            )
        )


class MoveRelativeToolAdapter(ToolInterface):
    """Move the base by a distance/direction relative to its current pose."""

    def __init__(self, tools_manager):
        self.tools_manager = tools_manager

    @property
    def tool_name(self) -> str:
        return "move_relative"

    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": (
                "Move the mobile base relative to its CURRENT pose. Use this "
                "for commands such as '往前走一米', '向后退半米', "
                "'向左移动1米', 'turn right 45 degrees and go forward 0.5m'. "
                "The goal is published in the robot base_link frame; do NOT "
                "invent map coordinates. distance_m is in metres, direction "
                "is forward/backward/left/right. Use summon_user only when "
                "the user asks the robot to come to them."
            ),
            "properties": {
                "direction": {
                    "type": "string",
                    "enum": ["forward", "backward", "left", "right"],
                    "description": "Direction relative to the robot heading"
                },
                "distance_m": {
                    "type": "number",
                    "description": "Distance in metres (0.1 to 5.0)"
                },
                "yaw_deg": {
                    "type": "number",
                    "description": "Optional relative heading change in degrees"
                }
            },
            "required": ["direction", "distance_m"]
        }

    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "status_message": {"type": "string"},
                "goal": {"type": "object"}
            }
        }

    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            result = self.tools_manager.move_relative(tool_input.arguments)
            if result and isinstance(result, dict) and result.get("ok"):
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("status_message"),
                    metadata={"goal": result.get("goal")}
                )
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=(result or {}).get("error", "move_relative failed")
            )
        except Exception as e:  # noqa: BLE001
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )


class SeeWhatToolAdapter(ToolInterface):
    """seewhat tool adapter. / seewhat工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "seewhat"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Captures an image from the camera to analyze the current environment. Input: None. Output: The scene description can be found in the 'data' field. The image path can be found in the 'metadata.image_path' field.",
            "properties": {},
            "required": []
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "description": {"type": "string"},
                "image_path": {"type": "string"}
            }
        }
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            # Call the seewhat method which now returns a structured dictionary. / 调用已返回结构化字典的seewhat方法。
            result = self.tools_manager.seewhat()

            if result and isinstance(result, dict):
                # Correctly wrap the structured data into ToolOutput. / 将结构化数据正确地包装到ToolOutput中。
                output = ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("description"),
                    metadata={
                        "image_path": result.get("image_path"),
                        "analysis_type": "environment"
                    }
                )
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                return output
            else:
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=False,
                    data=None,
                    metadata={},
                    error_message="Failed to get structured data from seewhat"
                )
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )

class AnalyzeVideoToolAdapter(ToolInterface):
    """analyze_video tool adapter. / analyze_video工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "analyze_video"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Analyzes the content of a video file from a given path. Input: 'video_path' (string). Output: The video description can be found in the 'data' field.",
            "properties": {
                "video_path": {
                    "type": "string",
                    "description": "Full path to the video file to be analyzed"
                }
            },
            "required": []
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "description": {"type": "string"},
                "video_path": {"type": "string"}
            }
        }
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            result = self.tools_manager.analyze_video(tool_input.arguments)

            if result and isinstance(result, dict) and result.get("description"):
                output = ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("description"),
                    metadata={
                        "video_path": result.get("video_path"),
                        "analysis_type": "video_content"
                    }
                )
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                return output
            else:
                error_msg = result.get("error", "Failed to get structured data from analyze_video")
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=False,
                    data=None,
                    metadata={"video_path": result.get("video_path")},
                    error_message=error_msg
                )
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )

class WriteDocumentToolAdapter(ToolInterface):
    """write_document tool adapter. / write_document工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "write_document"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Creates and saves a text document. Input: 'content' (string), 'format' (string), 'title' (string, optional), 'filename' (string, optional). Output: The original content written to the file can be found in the 'data' field. The saved file path can be found in the 'metadata.file_path' field.",
            "properties": {
                "filename": {
                    "type": "string",
                    "description": "Filename with extension (optional, auto-generated if not provided)"
                },
                "content": {
                    "type": "string",
                    "description": "Text content for creating documents, articles, poems, stories, reports, or any written content. ⚠️ IMPORTANT: This tool ONLY creates text/document content. It CANNOT and should NOT be used for image creation, artwork, or visual content. For any visual content, use generate_image tool instead."
                },
                "format": {
                    "type": "string",
                    "description": "Document format: md, txt, html, json",
                    "enum": ["md", "txt", "html", "json"]
                },
                "title": {
                    "type": "string",
                    "description": "Title of the document"
                }
            },
            "required": []
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "file_path": {"type": "string"},
                "content": {"type": "string"}
            }
        }
    
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            # Call the write_document method which now returns a structured dictionary. / 调用已返回结构化字典的write_document方法。
            result = self.tools_manager.write_document(tool_input.arguments)

            if result and isinstance(result, dict) and result.get("file_path"):
                # Correctly wrap the structured data into ToolOutput. / 将结构化数据正确地包装到ToolOutput中。
                output = ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("content"),
                    metadata={
                        "file_path": result.get("file_path"),
                        "format": tool_input.arguments.get("format", "txt"),
                        "title": tool_input.arguments.get("title", "文档")
                    }
                )
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                return output
            else:
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=False,
                    data=result.get("status_message", "Failed to write document"),
                    metadata={}
                )
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )





class GenerateImageToolAdapter(ToolInterface):
    """generate_image tool adapter. / generate_image工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "generate_image"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Generates an image based on a descriptive text prompt. Input: 'prompt' (string). Output: A JSON object containing the generation result is in the 'data' field. The local path of the saved image can be found in 'data.saved_paths'.",
            "properties": {
                "prompt": {
                    "type": "string",
                    "description": "Visual description for creating images, artwork, pictures, drawings, paintings, or any visual content. ⚠️ IMPORTANT: This tool ONLY creates visual/image content. It CANNOT and should NOT be used for text creation like poems, articles, stories, or documents. For any text-based content, use write_document tool instead."
                }
            },
            "required": ["prompt"]
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "image_urls": {"type": "array"},
                "status": {"type": "string"}
            }
        }
    
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            result = self.tools_manager.generate_image(tool_input.arguments)

            # The generate_image method now returns a dictionary, which is placed directly in the data field. / 现在 generate_image 返回的是一个字典，直接放入 data 字段。
            # Determine success based on the returned result. / 根据返回结果判断成功与否。
            success = isinstance(result, dict) and result.get('status') == 'success'
            
            output = ToolOutput(
                tool_name=self.tool_name,
                success=success,
                data=result,  # Place the complete dictionary in the data field. / 将完整的字典放入 data 字段。
                metadata={"prompt": tool_input.arguments.get("prompt", "")}
            )
            if output.success:
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                pass # Log has been commented out. / 日志已注释。
            return output
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )




class ScanTableToolAdapter(ToolInterface):
    """scan_table tool adapter. / scan_table工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "scan_table"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Scans an image to find and extract tabular data. Input: 'image_path' (string). Output: The extracted table content in Markdown format can be found in the 'data' field. The path to the saved markdown file can be found in 'metadata.file_path'.",
            "properties": {
                "image_path": {
                    "type": "string",
                    "description": "Path to image file containing table (can be auto-filled from previous image capture)"
                }
            },
            "required": []
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "table_content": {"type": "string"},
                "file_path": {"type": "string"}
            }
        }
    
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            # Call the scan_table method which now returns a structured dictionary. / 调用已返回结构化字典的scan_table方法。
            result = self.tools_manager.scan_table(tool_input.arguments)

            if result and isinstance(result, dict):
                # Correctly wrap the structured data into ToolOutput. / 将结构化数据正确地包装到ToolOutput中。
                output = ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("table_content"),
                    metadata={
                        "file_path": result.get("file_path"),
                        "image_path": tool_input.arguments.get("image_path", "")
                    }
                )
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                return output
            else:
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=False,
                    data=None,
                    metadata={},
                    error_message="Failed to get structured data from scan_table"
                )
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )


class VisualPositioningToolAdapter(ToolInterface):
    """visual_positioning tool adapter. / visual_positioning工具适配器。"""
    
    def __init__(self, tools_manager):
        self.tools_manager = tools_manager
    
    @property
    def tool_name(self) -> str:
        return "visual_positioning"
    
    @property
    def input_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "description": "Locates a specific object within an image. Input: 'image_path' (string), 'object_name' (string). Output: The coordinates of the found object can be found in the 'data' field. The path to the saved result file can be found in 'metadata.file_path'.",
            "properties": {
                "image_path": {
                    "type": "string",
                    "description": "Path to the image file for visual positioning"
                },
                "object_name": {
                    "type": "string",
                    "description": "Name of the object to locate in the image"
                }
            },
            "required": ["object_name"]
        }
    
    @property
    def output_schema(self) -> Dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "coordinates": {"type": "string"},
                "file_path": {"type": "string"}
            }
        }
    
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        try:
            # Call the visual_positioning method which now returns a structured dictionary. / 调用已返回结构化字典的visual_positioning方法。
            result = self.tools_manager.visual_positioning(tool_input.arguments)

            if result and isinstance(result, dict):
                # Correctly wrap the structured data into ToolOutput. / 将结构化数据正确地包装到ToolOutput中。
                output = ToolOutput(
                    tool_name=self.tool_name,
                    success=True,
                    data=result.get("coordinates_content"),
                    metadata={
                        "file_path": result.get("file_path"),
                        "explanation": result.get("explanation_content")
                    }
                )
                # self.tools_manager.node.get_logger().info(f"Tool {self.tool_name} successful output: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                return output
            else:
                return ToolOutput(
                    tool_name=self.tool_name,
                    success=False,
                    data=None,
                    metadata={},
                    error_message="Failed to get structured data from visual_positioning"
                )
        except Exception as e:
            return ToolOutput(
                tool_name=self.tool_name,
                success=False,
                data=None,
                metadata={},
                error_message=str(e)
            )
