#!/usr/bin/env python3
"""
MCP (Model Context Protocol) Server Implementation
A tool server compliant with the MCP standard.
/
MCP (Model Context Protocol) 服务器实现
符合MCP标准的工具服务器。
"""

import json
import uuid
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from enum import Enum

# Import the tool chain manager. / 导入工具链管理器。
try:
    from utils.tool_chain_manager import ToolChainManager, ToolInterface
    TOOL_CHAIN_AVAILABLE = True
except ImportError:
    TOOL_CHAIN_AVAILABLE = False


class MCPErrorCode(Enum):
    """MCP standard error codes. / MCP标准错误码。"""
    PARSE_ERROR = -32700
    INVALID_REQUEST = -32600
    METHOD_NOT_FOUND = -32601
    INVALID_PARAMS = -32602
    INTERNAL_ERROR = -32603
    
    # MCP specific error codes. / MCP特定错误码。
    TOOL_NOT_FOUND = -32000
    TOOL_EXECUTION_ERROR = -32001
    RESOURCE_NOT_FOUND = -32002
    PERMISSION_DENIED = -32003


@dataclass
class MCPRequest:
    """MCP request format. / MCP请求格式。"""
    jsonrpc: str = "2.0"
    method: str = ""
    params: Dict[str, Any] = field(default_factory=dict)
    id: Optional[str] = None


@dataclass
class MCPResponse:
    """MCP response format. / MCP响应格式。"""
    jsonrpc: str = "2.0"
    result: Optional[Dict[str, Any]] = None
    error: Optional[Dict[str, Any]] = None
    id: Optional[str] = None


@dataclass
class MCPTool:
    """MCP tool definition. / MCP工具定义。"""
    name: str
    description: str
    input_schema: Dict[str, Any]


class MCPServer:
    """
    MCP Server Implementation
    Manages tool registration, discovery, and invocation.
    /
    MCP服务器实现
    管理工具注册、发现和调用。
    """
    
    def __init__(self, node, tools_manager):
        self.node = node
        self.tools_manager = tools_manager
        self.tools_registry = {}
        self.server_info = {
            "name": "YahBoom Robot MCP Server",
            "version": "1.0.0",
            "protocol_version": "2024-11-05"
        }

        # Prioritize using the tool chain manager, with fallback to traditional registration. / 优先使用工具链管理器，回退到传统注册。
        if TOOL_CHAIN_AVAILABLE and hasattr(tools_manager, 'tool_chain_manager'):
            self.tool_chain_manager = tools_manager.tool_chain_manager
            self._register_tools_from_chain_manager()
            self.node.get_logger().info("MCP Server initialized with Tool Chain Manager")
    
    def _register_tools_from_chain_manager(self):
        """Register tools from the tool chain manager (new method). / 从工具链管理器注册工具（新方式）。"""
        if not hasattr(self, 'tool_chain_manager'):
            self.node.get_logger().error("Tool Chain Manager not available")
            return

        # Get registered tools from the tool chain manager. / 从工具链管理器获取已注册的工具。
        for tool_name, tool_interface in self.tool_chain_manager.tools.items():
            # Convert tool chain manager's tools to MCP tool format. / 将工具链管理器的工具转换为MCP工具格式。
            mcp_tool = MCPTool(
                name=tool_name,
                description=self._get_tool_description(tool_interface),
                input_schema=tool_interface.input_schema
            )
            self.tools_registry[tool_name] = mcp_tool

        self.node.get_logger().info(f"Registered {len(self.tools_registry)} tools from Tool Chain Manager")

    def _get_tool_description(self, tool_interface):
        """Get tool description. / 获取工具描述。"""
        descriptions = {
            "summon_user": "Make the robot come to the user's current position. Use this when the user asks the robot or trash bin to come over. Trigger keywords in English: come here, come to me, approach me, summon. Trigger keywords in Chinese: 过来, 来一下, 到我这里, 来我这边, 垃圾桶过来.",
            "seewhat": "Capture camera image and perform AI analysis of the current environment. Use this when the user wants to see what the robot can see, or when explicitly asked to look at the surroundings/environment. Trigger keywords in English: look, see, view, observe, surroundings, environment, what do you see. Trigger keywords in Chinese: 看看, 看见, 观察, 环境, 周围, 看一下.",
            "analyze_video": "Analyze video file content from specified path and provide detailed description. If no path is specified, the system will automatically use default test videos. Use this when the user wants to analyze local video files, or when asked to analyze/watch videos. Trigger keywords in English: analyze video, watch video, video analysis, video content, what is in this video. Trigger keywords in Chinese: 分析视频, 观看视频, 视频内容, 看视频.",
            "generate_image": "Generate image based on text description using AI image generation. Use this when the user wants to create images from text descriptions, or when asked to draw/create/generate pictures/images. Trigger keywords in English: draw, create image, generate picture, make art, image from text, paint, sketch. Trigger keywords in Chinese: 画画, 画图, 生成图片, 创作图像, 绘画.",
            "visual_positioning": "Locate and identify coordinates of specified objects in an image. Use this when the user wants to find specific objects in an image, or when asked to locate/identify objects. Trigger keywords in English: find object, locate object, object position, where is object, object coordinates, detect object. Trigger keywords in Chinese: 找到物体, 定位物体, 物体位置, 在哪里, 坐标.",
            "scan_table": "Scan and extract table content from image, convert to Markdown format. If no image path is specified, the system will automatically use default test images. Use this when the user wants to extract tabular data from images, or when asked to read/scan tables from pictures. Trigger keywords in English: scan table, read table, extract table, table from image, get data from table. Trigger keywords in Chinese: 扫描表格, 读取表格, 提取表格, 表格数据.",
            "write_document": "Create and save document with specified content and format. Use this when the user wants to generate text documents, or when asked to write/create documents/reports. Trigger keywords in English: write document, create report, generate text, make document, write report, compose article. Trigger keywords in Chinese: 写文档, 写报告, 生成文档, 创作文章, 写一篇文章.",
            "agent_call": "Execute complex AI Agent tasks with intelligent planning and multi-step coordination. Use this for complex multi-step tasks that require planning and coordination between multiple tools. Trigger keywords in English: plan, complex task, multi-step, coordinate tasks, need to think, multi stage, complicated. Trigger keywords in Chinese: 计划, 复杂任务, 多步骤, 协调任务, 需要思考, 多阶段."
        }
        return descriptions.get(tool_interface.tool_name, f"Execute {tool_interface.tool_name} tool")

    
    def handle_request(self, request_data: str) -> str:
        """Handle MCP request. / 处理MCP请求。"""
        try:
            # Parse JSON-RPC request. / 解析JSON-RPC请求。
            request_json = json.loads(request_data)
            request = MCPRequest(
                jsonrpc=request_json.get("jsonrpc", "2.0"),
                method=request_json.get("method", ""),
                params=request_json.get("params", {}),
                id=request_json.get("id")
            )
            
            # Route to the corresponding handler method. / 路由到相应的处理方法。
            if request.method == "initialize":
                response = self._handle_initialize(request)
            elif request.method == "tools/list":
                response = self._handle_tools_list(request)
            elif request.method == "tools/call":
                response = self._handle_tools_call(request)
            elif request.method == "resources/list":
                response = self._handle_resources_list(request)
            else:
                response = self._create_error_response(
                    request.id, 
                    MCPErrorCode.METHOD_NOT_FOUND, 
                    f"Method '{request.method}' not found"
                )
            
            return json.dumps(response.__dict__, ensure_ascii=False, indent=2)
            
        except json.JSONDecodeError:
            error_response = self._create_error_response(
                None, 
                MCPErrorCode.PARSE_ERROR, 
                "Invalid JSON"
            )
            return json.dumps(error_response.__dict__, ensure_ascii=False, indent=2)
        except Exception as e:
            error_response = self._create_error_response(
                None, 
                MCPErrorCode.INTERNAL_ERROR, 
                str(e)
            )
            return json.dumps(error_response.__dict__, ensure_ascii=False, indent=2)
    
    def _handle_initialize(self, request: MCPRequest) -> MCPResponse:
        """Handle initialize request. / 处理初始化请求。"""
        return MCPResponse(
            id=request.id,
            result={
                "protocolVersion": "2024-11-05",
                "capabilities": {
                    "tools": {},
                    "resources": {}
                },
                "serverInfo": self.server_info
            }
        )
    
    def _handle_tools_list(self, request: MCPRequest) -> MCPResponse:
        """Handle tool list request. / 处理工具列表请求。"""
        tools_list = []
        for tool_name, tool in self.tools_registry.items():
            tools_list.append({
                "name": tool.name,
                "description": tool.description,
                "inputSchema": tool.input_schema
            })
        
        return MCPResponse(
            id=request.id,
            result={"tools": tools_list}
        )
    
    def _handle_tools_call(self, request: MCPRequest) -> MCPResponse:
        """Handle tool call request. / 处理工具调用请求。"""
        try:
            params = request.params or {}
            tool_name = params.get("name")
            arguments = params.get("arguments", {})
            
            if tool_name not in self.tools_registry:
                return self._create_error_response(
                    request.id,
                    MCPErrorCode.TOOL_NOT_FOUND,
                    f"Tool '{tool_name}' not found"
                )
            
            # Prioritize executing the tool using the tool chain manager. / 优先使用工具链管理器执行工具。
            tool_call = {
                "name": tool_name,
                "arguments": arguments
            }

            if hasattr(self, 'tool_chain_manager'):
                # Use the new tool chain manager for execution (supports data passing). / 使用新的工具链管理器执行（支持数据传递）。
                outputs = self.tool_chain_manager.execute_tool_chain([tool_call])
                if outputs:
                    output = outputs[0]
                    if output.success:
                        result = output.data
                        self.node.get_logger().info(f"Tool {tool_name} executed via Tool Chain Manager: success")
                    else:
                        raise Exception(output.error_message or "Tool execution failed")
                else:
                    raise Exception("No output from tool chain manager")
            
            return MCPResponse(
                id=request.id,
                result={
                    "content": [
                        {
                            "type": "text",
                            "text": str(result) if result else "Tool executed successfully"
                        }
                    ]
                }
            )
            
        except Exception as e:
            return self._create_error_response(
                request.id,
                MCPErrorCode.TOOL_EXECUTION_ERROR,
                str(e)
            )
    
    def _handle_resources_list(self, request: MCPRequest) -> MCPResponse:
        """Handle resource list request. / 处理资源列表请求。"""
        # This can list accessible resources (files, databases, etc.). / 这里可以列出可访问的资源（文件、数据库等）。
        return MCPResponse(
            id=request.id,
            result={"resources": []}
        )
    
    def _create_error_response(self, request_id: Optional[str], error_code: MCPErrorCode, message: str) -> MCPResponse:
        """Create an error response. / 创建错误响应。"""
        return MCPResponse(
            id=request_id,
            error={
                "code": error_code.value,
                "message": message
            }
        )
    
    def get_tools_json_schema(self) -> List[Dict[str, Any]]:
        """
        Generate a structured list of tool JSON schemas for the LLM.
        This method replaces the old get_tools_for_prompt, which generated plain text descriptions,
        to provide structured data that the LLM can parse accurately.
        /
        为LLM生成结构化的工具JSON Schema列表。
        这个方法将替代旧的、生成纯文本描述的 get_tools_for_prompt，
        以提供让LLM能够精确解析的结构化数据。
        """
        tools_list = []
        # self.tools_registry has already been populated with descriptions during initialization via _get_tool_description. / self.tools_registry 已在初始化时通过 _get_tool_description 填充了描述。
        for tool_name, tool in self.tools_registry.items():
            tool_schema = {
                "name": tool.name,
                "description": tool.description,
                "parameters": tool.input_schema
            }
            tools_list.append(tool_schema)
        
        return tools_list
