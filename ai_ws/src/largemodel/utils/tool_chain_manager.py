"""
Tool chain data passing manager.
Implements a generic data flow mechanism between tools.
/
工具链数据传递管理器。
实现通用的工具间数据流转机制。
"""

from typing import Dict, Any, List, Optional, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod
import json
import logging

@dataclass
class ToolOutput:
    """Standardized tool output format. / 标准化的工具输出格式。"""
    tool_name: str
    success: bool
    data: Any  # Main output data / 主要输出数据
    metadata: Dict[str, Any]  # Metadata (e.g., file paths, coordinates) / 元数据（如文件路径、坐标等）
    error_message: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary format. / 转换为字典格式。"""
        return {
            "tool_name": self.tool_name,
            "success": self.success,
            "data": self.data,
            "metadata": self.metadata,
            "error_message": self.error_message
        }

@dataclass
class ToolInput:
    """Standardized tool input format. / 标准化的工具输入格式。"""
    arguments: Dict[str, Any]  # Original parameters / 原始参数
    previous_outputs: List[ToolOutput]  # Outputs from preceding tools / 前置工具的输出
    context: Dict[str, Any]  # Global context / 全局上下文
    
    def get_previous_output_by_tool(self, tool_name: str) -> Optional[ToolOutput]:
        """Get previous output by tool name. / 根据工具名获取前置输出。"""
        for output in self.previous_outputs:
            if output.tool_name == tool_name:
                return output
        return None

    def to_dict(self) -> Dict[str, Any]:
        """Convert ToolInput to a printable dictionary. / 将ToolInput转换为可打印的字典。"""
        return {
            "arguments": self.arguments,
            "previous_outputs": [o.to_dict() for o in self.previous_outputs],
            "context": self.context
        }
    
    def get_latest_output(self) -> Optional[ToolOutput]:
        """Get the latest tool output. / 获取最新的工具输出。"""
        return self.previous_outputs[-1] if self.previous_outputs else None

class ToolInterface(ABC):
    """Base class for tool interfaces. / 工具接口基类。"""
    
    @property
    @abstractmethod
    def tool_name(self) -> str:
        """Tool name. / 工具名称。"""
        pass
    
    @property
    @abstractmethod
    def input_schema(self) -> Dict[str, Any]:
        """Input parameter schema definition. / 输入参数模式定义。"""
        pass
    
    @property
    @abstractmethod
    def output_schema(self) -> Dict[str, Any]:
        """Output data schema definition. / 输出数据模式定义。"""
        pass
    
    @abstractmethod
    def execute(self, tool_input: ToolInput) -> ToolOutput:
        """Execute tool logic. / 执行工具逻辑。"""
        pass
    

class ToolChainManager:
    """Tool chain data passing manager. / 工具链数据传递管理器。"""
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self.tools: Dict[str, ToolInterface] = {}

    def register_tool(self, tool: ToolInterface):
        """Register a tool. / 注册工具。"""
        self.tools[tool.tool_name] = tool
    
    def execute_tool_chain(self, tool_calls: List[Dict[str, Any]]) -> List[ToolOutput]:
        """Execute the tool chain. / 执行工具链。"""
        outputs = []
        context = {}
        
        for i, tool_call in enumerate(tool_calls):
            tool_name = tool_call.get("name")
            arguments = tool_call.get("arguments", {})
            
            if tool_name not in self.tools:
                self.logger.error(f"Tool not found: {tool_name}")
                continue
            
            # Create tool input. / 创建工具输入。
            tool_input = ToolInput(
                arguments=arguments,
                previous_outputs=outputs.copy(),
                context=context
            )
                        
            # Execute the tool. / 执行工具。
            try:
                # Debug log: print tool input. / 调试日志：打印工具输入。
                # self.logger.info(f"↓↓↓ [ToolChain] Executing Tool: {tool_name} ↓↓↓")
                # self.logger.info(f"ToolInput: {json.dumps(tool_input.to_dict(), indent=2, ensure_ascii=False)}")

                tool = self.tools[tool_name]
                output = tool.execute(tool_input)
                outputs.append(output)

                # Debug log: print tool output. / 调试日志：打印工具输出。
                # self.logger.info(f"ToolOutput: {json.dumps(output.to_dict(), indent=2, ensure_ascii=False)}")
                # self.logger.info(f"↑↑↑ [ToolChain] Finished Tool: {tool_name}, Success: {output.success} ↑↑↑")
                
                # Update context. / 更新上下文。
                context[f"{tool_name}_result"] = output.data
                
            except Exception as e:
                error_output = ToolOutput(
                    tool_name=tool_name,
                    success=False,
                    data=None,
                    metadata={},
                    error_message=str(e)
                )
                outputs.append(error_output)
                self.logger.error(f"Tool {tool_name} execution failed: {e}")
        
        return outputs
    

