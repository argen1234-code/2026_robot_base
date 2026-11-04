import json
import re
from typing import List, Dict, Any
from datetime import datetime
from typing import Dict, List, Any, Optional


class AIAgent:
    """
    Core AI Agent class. / AI Agent核心类。
    Responsible for planning, decomposing, and executing complex tasks. / 负责复杂任务的规划、分解和执行。
    """
    
    def __init__(self, node, tools_manager):
        """
        Initializes the AI Agent. / 初始化AI Agent。
        :param node: ROS2 node instance. / ROS2节点实例。
        :param tools_manager: ToolsManager instance. / 工具管理器实例。
        """
        self.node = node
        self.tools_manager = tools_manager
        self.feedback_callback = None # Callback for providing feedback. / 用于提供反馈的回调。
 
        # Agent configuration. / Agent配置。
        self.max_iterations = 10
        self.max_execution_time = 300
        self.retry_attempts = 2
        self.retry_interval = 1

        # Current task status. / 当前任务状态。
        self.current_task = None
        self.task_steps = []

        # Simple execution history (only saves the last 10 tasks). / 简单的执行历史（只保存最近10个任务）。
        self.execution_history = []
        self.max_history = 10

        self.node.get_logger().info("AI Agent initialized successfully")
    
    def execute_task(self, task_description: str) -> Dict[str, Any]:
        """
        Main entry point for executing an AI Agent task. / 执行AI Agent任务的主入口。
        """
        self.node.get_logger().info(f"AI Agent starting task: {task_description}")
        
        try:
            # Start the task. / 开始任务。
            self.current_task = {
                "description": task_description,
                "start_time": datetime.now(),
                "status": "executing",
                "steps": []
            }

            # Execute the agent workflow. / 执行Agent工作流。
            result = self._execute_agent_workflow(task_description)

            # Complete the task and save to history. / 完成任务并保存到历史。
            self.current_task["end_time"] = datetime.now()
            self.current_task["success"] = result["success"]
            self.current_task["message"] = result["message"]

            # Save to history (keeping only recent tasks). / 保存到历史记录（只保留最近的任务）。
            self.execution_history.append(self.current_task.copy())
            if len(self.execution_history) > self.max_history:
                self.execution_history.pop(0)

            return result

        except Exception as e:
            self.node.get_logger().error(f"AI Agent task execution failed: {e}")
            error_result = {"success": False, "message": f"Agent execution failed: {str(e)}"}

            # Record the failed task. / 记录失败的任务。
            if self.current_task:
                self.current_task["end_time"] = datetime.now()
                self.current_task["success"] = False
                self.current_task["message"] = error_result["message"]
                self.execution_history.append(self.current_task.copy())

            return error_result
    
    def _execute_agent_workflow(self, task_description: str) -> Dict[str, Any]:
        """
        Executes the agent workflow: Plan -> Execute -> Reflect -> Adjust. / 执行Agent工作流：规划 -> 执行 -> 反思 -> 调整。
        """
        try:
            # Step 1: Task Planning. / 第一步：任务规划。
            self.node.get_logger().info("AI Agent starting task planning phase")
            plan_result = self._plan_task(task_description)
            self.node.get_logger().info(f"Task planning result: {plan_result}")

            if not plan_result["success"]:
                self.node.get_logger().error(f"Task planning failed: {plan_result.get('message', 'Unknown error')}")
                return plan_result

            self.task_steps = plan_result["steps"]
            self.node.get_logger().info(f"Task planning completed with {len(self.task_steps)} steps")
            
            # Send detailed planning results to the frontend. / 发送详细的规划结果给前端。
            if self.feedback_callback:
                plan_details = "\\n".join([f"  - Step {s['id']}: {s['description']}" for s in self.task_steps])
                self.feedback_callback(f"🤖 AI Agent task planning complete, {len(self.task_steps)} steps total:\\n{plan_details}")

            # Step 2: Execute Task Steps. / 第二步：执行任务步骤。
            execution_results = []
            step_outputs = {}  # Store step outputs for subsequent steps. / 存储步骤输出，用于后续步骤。
            tool_outputs = []  # Store outputs in ToolOutput format for toolchain data passing. / 存储ToolOutput格式的输出，用于工具链数据传递。

            for i, step in enumerate(self.task_steps):
                if self.node.language == 'zh':
                    step_info = f"▶️ 执行步骤 {i+1}/{len(self.task_steps)}: {step['description']}"
                else:
                    step_info = f"▶️ Executing step {i+1}/{len(self.task_steps)}: {step['description']}"
                self.node.get_logger().info(step_info)
                if self.feedback_callback:
                    self.feedback_callback(step_info)

                # Skip steps without a tool. / 跳过没有工具的步骤。
                if not step.get("tool") or step.get("tool").strip() == "":
                    self.node.get_logger().warn(f"Step {i+1} has no tool specified, skipping execution")
                    execution_results.append({
                        "success": True,
                        "step_id": step.get("id"),
                        "description": step["description"],
                        "message": "Skipped (no tool)"
                    })
                    continue

                # Step 1: Process parameters before execution. / 步骤1：在执行前，先调用参数解析器处理参数。
                # This method finds {{...}} references and replaces them with actual outputs from previous steps. / 这个方法会查找 {{...}} 引用，并用之前步骤的真实输出替换它们。
                try:
                    processed_parameters = self._process_step_parameters(step.get("parameters", {}), tool_outputs)
                    step["parameters"] = processed_parameters # Update parameters in the step. / 更新步骤中的参数。
                except Exception as e:
                    self.node.get_logger().error(f"Parameter processing failed, step {i+1} will be skipped: {e}")
                    step_result = {
                        "success": False,
                        "step_id": step.get("id"),
                        "description": step["description"],
                        "error": f"Parameter processing failed: {e}",
                        "message": f"Step execution failed: {e}"
                    }
                    execution_results.append(step_result)
                    # If parameter processing fails, terminate the entire task. / 如果参数处理失败，则直接终止整个任务。
                    return {"success": False, "message": f"Task terminated due to parameter processing failure in step {i+1} ('{step['description']}'): {e}", "results": execution_results}

                # Record the step in the current task. / 记录步骤到当前任务。
                step_record = {
                    "step_id": step.get("id"),
                    "description": step["description"],
                    "start_time": datetime.now()
                }

                # Step 2: Execute the step with processed parameters. / 步骤2：使用处理过的参数执行步骤。
                step_result = self._execute_step(step, tool_outputs)
                execution_results.append(step_result)

                # Step 3: Save the output for subsequent steps to reference. / 步骤3：保存输出以供后续步骤引用。
                if step_result.get("success"):
                    # Use the complete ToolOutput object returned by the tool adapter. / 使用工具适配器返回的完整ToolOutput对象。
                    if step_result.get("tool_output"):
                        tool_outputs.append(step_result["tool_output"])
                        self.node.get_logger().info(f"Step {i+1} output added to the tool chain passing list")
                else:
                    # Also add failed steps to the tool output list, but mark them as failed. / 失败的步骤也要添加到工具输出列表，但标记为失败。
                    if step_result.get("tool_output"):
                        tool_outputs.append(step_result["tool_output"])
                    else:
                        from utils.tool_chain_manager import ToolOutput
                        tool_output = ToolOutput(
                            tool_name=step.get("tool"),
                            success=False,
                            data=None,
                            metadata={"step_id": step.get("id"), "description": step.get("description")},
                            error_message=step_result.get("error", "Step execution failed and did not generate a standard output object")
                        )
                        tool_outputs.append(tool_output)

                # Complete the step record. / 完善步骤记录。
                step_record["end_time"] = datetime.now()
                step_record["success"] = step_result["success"]
                step_record["result"] = step_result
                if self.current_task:
                    self.current_task["steps"].append(step_record)

                # If a step fails, abort the entire task. / 如果步骤失败，则中止整个任务。
                if not step_result.get("success", False):
                    self.node.get_logger().error(f"Step {i+1} execution failed: {step_result.get('message', 'Unknown error')}")
                    self.node.get_logger().error(f"Task '{task_description}' terminated early due to a failed step.")
                    # Return a final result with failure information. / 返回一个包含失败信息的最终结果。
                    return {
                        "success": False,
                        "message": f"Task terminated because step '{step['description']}' failed.",
                        "steps_executed": i + 1,
                        "results": execution_results
                    }
                else:
                    self.node.get_logger().info(f"Step {i+1} executed successfully")
                    # Send successful tool execution result. / 发送成功的工具执行结果。
                    if self.feedback_callback:
                        tool_output = step_result.get("tool_output")
                        if tool_output:
                            feedback = self._format_tool_output_for_feedback(tool_output)
                            self.feedback_callback(feedback)

            summary = self._summarize_execution(task_description, execution_results)

            # Check for generated documents. / 检查是否有文档生成。
            generated_files = []
            for result in execution_results:
                if result.get("success") and "Saved to:" in str(result.get("result", "")):
                    # Extract file path. / 提取文件路径。
                    result_text = str(result.get("result", ""))
                    if "Saved to:" in result_text:
                        file_path = result_text.split("Saved to:")[-1].strip()
                        generated_files.append(file_path)

            return {
                "success": True,
                "message": summary,
                "steps_executed": len(execution_results),
                "results": execution_results,
                "generated_files": generated_files
            }

        except Exception as e:
            self.node.get_logger().error(f"AI Agent workflow execution exception: {e}")
            import traceback
            self.node.get_logger().error(f"Exception stack: {traceback.format_exc()}")
            if self.node.language == 'zh':
                error_msg = f"工作流执行失败: {str(e)}"
            else:
                error_msg = f"Workflow execution failed: {str(e)}"
            return {"success": False, "message": error_msg}
    
    def _plan_task(self, task_description: str) -> Dict[str, Any]:
        """
        Uses the large model for task planning and decomposition. / 使用大模型进行任务规划和分解。
        """
        try:
            self.node.get_logger().info("Starting to construct planning prompt")
            
            # ✅ [重构] 动态生成工具列表及其描述
            tool_descriptions = []
            for name, adapter in self.tools_manager.tool_chain_manager.tools.items():
                # ✅ 关键修改：直接、可靠地从schema获取高质量描述
                description = adapter.input_schema.get("description", "No description available.")
                
                # 提取参数名用于提示
                props = adapter.input_schema.get("properties", {})
                params = ", ".join(props.keys())
                
                tool_descriptions.append(f"- {name}({params}): {description}")
            
            available_tools_str = "\n".join(tool_descriptions)

            if self.node.language == 'zh':
                planning_prompt = f"""
作为一个专业的任务规划Agent，请将用户任务分解为一系列具体的、可执行的JSON步骤。

**# 可用工具:**
{available_tools_str}

**# 核心规则:**
1.  **数据传递**: 当后续步骤需要使用之前步骤的输出时，**必须**使用 `{{{{steps.N.outputs.KEY}}}}` 格式进行引用。
    - `N` 是步骤的ID（从1开始）。
    - `KEY` 是之前步骤输出数据中的具体字段名。
    - `outputs` 后面可以是 `data` (主要数据) 或 `metadata.sub_key` (元数据)。
2.  **JSON格式**: 必须严格返回JSON对象，不要包含任何markdown包装 (如 ```json```)。
3.  **工具选择**: 严格根据工具描述选择最合适的工具。

**# 输出格式:**
```json
{{
    "steps": [
        {{
            "id": 1,
            "description": "对第一步的简要中文描述",
            "tool": "工具名称",
            "parameters": {{ "参数1": "值1" }}
        }},
        {{
            "id": 2,
            "description": "对第二步的简要中文描述",
            "tool": "另一个工具",
            "parameters": {{ "参数A": "{{{{steps.1.outputs.data}}}}", "参数B": "{{{{steps.1.outputs.metadata.file_path}}}}" }}
        }}
    ]
}}
```

**# 示例: "先看看周围有什么，然后把看到的写成文档"**
```json
{{
    "steps": [
        {{
            "id": 1,
            "description": "首先，使用seewhat工具观察当前环境。",
            "tool": "seewhat",
            "parameters": {{}}
        }},
        {{
            "id": 2,
            "description": "然后，将观察到的环境描述写入一个TXT文档。",
            "tool": "write_document",
            "parameters": {{
                "format": "txt",
                "title": "环境观察报告",
                "content": "{{{{steps.1.outputs.data}}}}",
                "filename": "env_report.txt"
            }}
        }}
    ]
}}
```

**# 用户任务:**
{task_description}
"""
            else:
                planning_prompt = f"""
As a professional task planning Agent, please break down the user's task into a series of specific, executable JSON steps.

**# Available Tools:**
{available_tools_str}

**# Core Rules:**
1.  **Data Passing**: When a subsequent step needs to use the output of a previous step, you **must** use the `{{{{steps.N.outputs.KEY}}}}` format for referencing.
    - `N` is the step ID (starting from 1).
    - `KEY` is the specific field name in the output data of the previous step.
    - `outputs` can be followed by `data` (for primary data) or `metadata.sub_key` (for metadata).
2.  **JSON Format**: You must strictly return a JSON object. Do not include any Markdown wrappers (like ```json```).
3.  **Tool Selection**: Strictly select the most appropriate tool based on its description.

**# Output Format:**
```json
{{
    "steps": [
        {{
            "id": 1,
            "description": "A brief description of the first step",
            "tool": "tool_name",
            "parameters": {{ "param1": "value1" }}
        }},
        {{
            "id": 2,
            "description": "A brief description of the second step",
            "tool": "another_tool",
            "parameters": {{ "paramA": "{{{{steps.1.outputs.data}}}}", "paramB": "{{{{steps.1.outputs.metadata.file_path}}}}" }}
        }}
    ]
}}
```

**# Example: "First, look around to see what's there, then write what you see into a document"**
```json
{{
    "steps": [
        {{
            "id": 1,
            "description": "First, use the seewhat tool to observe the current environment.",
            "tool": "seewhat",
            "parameters": {{}}
        }},
        {{
            "id": 2,
            "description": "Then, write the observed environmental description into a TXT document.",
            "tool": "write_document",
            "parameters": {{
                "format": "txt",
                "title": "Environmental Observation Report",
                "content": "{{{{steps.1.outputs.data}}}}",
                "filename": "env_report.txt"
            }}
        }}
    ]
}}
```

**# User Task:**
{task_description}
"""
            
            # 调用大模型进行规划
            self.node.get_logger().info("Start calling LLM for task planning")
            messages_to_use = [{"role": "user", "content": planning_prompt}]
            result = self.node.model_client.infer_with_text("", message=messages_to_use)

            if isinstance(result, dict):
                response_text = result.get('response', '')
            else:
                response_text = str(result)

            # 处理markdown格式的JSON响应
            json_text = response_text
            if "```json" in response_text:
                # 提取markdown中的JSON部分
                start_marker = "```json"
                end_marker = "```"
                start_idx = response_text.find(start_marker)
                if start_idx != -1:
                    start_idx += len(start_marker)
                    end_idx = response_text.find(end_marker, start_idx)
                    if end_idx != -1:
                        json_text = response_text[start_idx:end_idx].strip()
                        self.node.get_logger().info(f"Extracted JSON text: {json_text}")

            # 解析JSON响应
            try:
                plan_data = json.loads(json_text)
                steps = plan_data.get("steps", [])

                self.node.get_logger().info(f"Parsed steps: {steps}")

                if not steps:
                    self.node.get_logger().error("Task planning failed: no valid steps were generated")
                    if self.node.language == 'zh':
                        error_msg = "任务规划失败：未生成有效步骤"
                    else:
                        error_msg = "Task planning failed: no valid steps were generated"
                    return {"success": False, "message": error_msg}

                return {"success": True, "steps": steps}

            except json.JSONDecodeError as e:
                # 如果JSON解析失败，尝试提取步骤信息
                self.node.get_logger().error(f"JSON parsing failed: {e}, response content: {response_text}")
                if self.node.language == 'zh':
                    error_msg = f"任务规划失败：响应格式错误 - {str(e)}"
                else:
                    error_msg = f"Task planning failed: response format error - {str(e)}"
                return {"success": False, "message": error_msg}

        except Exception as e:
            self.node.get_logger().error(f"Task planning exception: {e}")
            if self.node.language == 'zh':
                error_msg = f"任务规划失败: {str(e)}"
            else:
                error_msg = f"Task planning failed: {str(e)}"
            return {"success": False, "message": error_msg}

    def _process_step_parameters(self, parameters: Dict[str, Any], previous_outputs: List[Any]) -> Dict[str, Any]:
        """
        Parses parameter dictionary, finds and replaces all {{...}} references using regex.
        This implementation can handle multiple references within a single string.
        /
        解析参数字典，使用正则表达式查找并替换所有 {{...}} 引用。
        这个新实现可以处理一个字符串中包含多个引用的情况。
        """
        processed_params = parameters.copy()
        pattern = re.compile(r"\{\{steps\.(\d+)\.outputs\.(.+?)\}\}")

        for key, value in processed_params.items():
            if not isinstance(value, str):
                continue

            # Use an inner function as the replacement argument for re.sub to execute find logic for each match.
            #/ 使用一个内部函数作为 re.sub 的替换参数，这样可以对每个匹配项执行查找逻辑。
            def replacer(match):
                step_id = int(match.group(1))
                path = match.group(2).strip() # .strip() ensures the path has no extra spaces. / .strip() 确保路径没有多余的空格。
                
                self.node.get_logger().info(f"Found reference in parameter '{key}': step_id={step_id}, path='{path}'")

                if not (1 <= step_id <= len(previous_outputs)):
                    raise ValueError(f"Invalid step ID referenced: {step_id} (available steps: {len(previous_outputs)})")

                # Get the corresponding step output. / 获取对应的步骤输出。
                target_output = previous_outputs[step_id - 1]
                
                # Extract the real value from the ToolOutput object based on the path.
                #/ 从ToolOutput对象中根据路径提取真实值。
                retrieved_value = self._get_value_from_path(target_output, path)
                
                if retrieved_value is None:
                    available_keys = list(target_output.to_dict().keys())
                    available_metadata_keys = list(target_output.to_dict().get('metadata', {}).keys())
                    raise ValueError(f"Path '{path}' not found in the output of step {step_id}. Available top-level keys: {available_keys}. Available metadata keys: {available_metadata_keys}")

                # The returned value must be a string to be embedded in the original parameter string.
                #/ 返回的值必须是字符串，以便嵌入到原始参数字符串中。
                return str(retrieved_value)

            try:
                # Use re.sub and our replacer function to replace all found placeholders.
                #/ 使用 re.sub 和我们的 replacer 函数来替换所有找到的占位符。
                # Check if the placeholder pattern exists in the original value.
                #/ 检查原始值中是否存在占位符模式。
                if pattern.search(value):
                    new_value = pattern.sub(replacer, value)
                    
                    single_match = pattern.fullmatch(value)
                    if single_match:
                        step_id = int(single_match.group(1))
                        path = single_match.group(2).strip()
                        retrieved_value = self._get_value_from_path(previous_outputs[step_id - 1], path)
                        if retrieved_value is not None:
                             processed_params[key] = retrieved_value
                             self.node.get_logger().info(f"Parameter '{key}' has been directly replaced with the value (type: {type(retrieved_value).__name__})")
                        else:
                            raise ValueError(f"Path '{path}' not found in the output of step {step_id}")
                    else:
                        processed_params[key] = new_value
                        self.node.get_logger().info(f"Parameter '{key}' updated. Old value: '{value}', New value: '{new_value}'")

            except ValueError as e:
                raise e
        
        return processed_params

    def _get_value_from_path(self, output_obj, path: str) -> Any:
        parts = path.split('.')
        current_value = output_obj.to_dict()  # Start with the full object as a dictionary

        for part in parts:
            match = re.match(r"(\w+)\[(\d+)\]", part)
            
            if match:
                key = match.group(1)
                index = int(match.group(2))
                
                # First, get the list using the key
                if isinstance(current_value, dict) and key in current_value:
                    target_list = current_value.get(key)
                    
                    # Then, access the element using the index
                    if isinstance(target_list, list):
                        if 0 <= index < len(target_list):
                            current_value = target_list[index]
                        else:
                            self.node.get_logger().error(f"Path parsing error: index {index} is out of range for list '{key}' (size: {len(target_list)})")
                            return None  # Index out of bounds
                    else:
                        self.node.get_logger().error(f"Path parsing error: '{key}' is not a list and cannot be accessed by index.")
                        return None # Trying to index a non-list
                else:
                    self.node.get_logger().error(f"Path parsing error: key '{key}' not found in the current data.")
                    return None # Key not found
            else:
                # Fallback to simple dictionary key access
                if isinstance(current_value, dict) and part in current_value:
                    current_value = current_value[part]
                else:
                    available_keys = list(current_value.keys()) if isinstance(current_value, dict) else []
                    self.node.get_logger().error(f"Path parsing error: key '{part}' not found in the current data. Available keys: {available_keys}")
                    return None  # Path is invalid
                    
        return current_value

    def _execute_step(self, step: Dict[str, Any], previous_tool_outputs: Optional[List[Any]] = None) -> Dict[str, Any]:
        """
        Execute a single task step - Using tool chain to support data passing
        /
        执行单个任务步骤 - 使用工具链支持数据传递
        """
        try:
            tool_name = step.get("tool")
            parameters = step.get("parameters", {})
            description = step.get("description", "")

            # 确保previous_tool_outputs不为None
            if previous_tool_outputs is None:
                previous_tool_outputs = []

            self.node.get_logger().info(f"Executing tool: {tool_name}, parameters: {parameters}")
            if previous_tool_outputs:
                self.node.get_logger().info(f"Available preceding tool outputs: {[output.tool_name for output in previous_tool_outputs]}")

            # 构造工具调用
            tool_call = {
                "name": tool_name,
                "arguments": parameters
            }

            # ✅ 修改：使用工具链执行，支持数据传递
            if hasattr(self.tools_manager, 'tool_chain_manager'):
                self.node.get_logger().info(f"Executing tool with tool chain manager: {tool_name}")

                # ✅ 关键修改：手动创建ToolInput来传递前置输出
                from utils.tool_chain_manager import ToolInput, ToolOutput

                # 创建工具输入，包含前置工具的输出
                tool_input = ToolInput(
                    arguments=parameters,
                    previous_outputs=previous_tool_outputs,
                    context={}
                )

                # 获取工具适配器并执行
                tool_adapter = self.tools_manager.tool_chain_manager.tools.get(tool_name)
                if tool_adapter:
                    output = tool_adapter.execute(tool_input)

                    if output.success:
                        tool_result = output.data
                        # ✅ 保存完整的ToolOutput对象，用于后续步骤的数据传递
                        current_tool_output = output
                        self.node.get_logger().info(f"Tool chain execution successful: {tool_name}")
                    else:
                        error_msg = output.error_message or "Tool execution failed"
                        self.node.get_logger().error(f"Tool chain execution failed: {error_msg}")
                        if self.node.language == 'zh':
                            tool_result = f"工具执行失败: {error_msg}"
                        else:
                            tool_result = f"Tool execution failed: {error_msg}"
                        # 创建失败的ToolOutput对象
                        current_tool_output = output
                else:
                    # 回退到工具链管理器的execute_tool_chain方法
                    outputs = self.tools_manager.tool_chain_manager.execute_tool_chain([tool_call])
                    if outputs and outputs[0].success:
                        tool_result = outputs[0].data
                        current_tool_output = outputs[0]
                        self.node.get_logger().info(f"Tool chain execution successful: {tool_name}")
                    else:
                        error_msg = outputs[0].error_message if outputs else "No output from tool chain"
                        self.node.get_logger().error(f"Tool chain execution failed: {error_msg}")
                        if self.node.language == 'zh':
                            tool_result = f"工具执行失败: {error_msg}"
                        else:
                            tool_result = f"Tool execution failed: {error_msg}"
                        current_tool_output = outputs[0] if outputs else None
            else:
                # 回退到原有方式
                self.node.get_logger().warning("Tool chain manager is not available, falling back to traditional execution")
                tool_result = self.tools_manager.execute_tool(tool_call, allow_follow_up_tools=True)

            if self.node.language == 'zh':
                success_msg = f"步骤执行成功: {description}"
            else:
                success_msg = f"Step executed successfully: {description}"
                
            return {
                "success": True,
                "step_id": step.get("id"),
                "description": description,
                "tool_used": tool_name,
                "result": tool_result,
                "tool_output": current_tool_output if 'current_tool_output' in locals() else None,
                "message": success_msg
            }

        except Exception as e:
            if self.node.language == 'zh':
                error_msg = f"步骤执行失败: {str(e)}"
            else:
                error_msg = f"Step execution failed: {str(e)}"
                
            return {
                "success": False,
                "step_id": step.get("id"),
                "description": step.get("description", ""),
                "error": str(e),
                "message": error_msg
            }
    
    
    def _summarize_execution(self, task_description: str, execution_results: List[Dict[str, Any]]) -> str:
        """
        Summarizes the execution results of a task. / 总结任务执行结果。
        """
        successful_steps = [r for r in execution_results if r.get("success", False)]
        failed_steps = [r for r in execution_results if not r.get("success", False)]
        
        summary = f"Task '{task_description}' execution completed."
        summary += f" Executed {len(execution_results)} steps in total: "
        summary += f"{len(successful_steps)} succeeded, {len(failed_steps)} failed."
        
        if failed_steps:
            summary += f" Failed steps: {', '.join([s.get('description', '') for s in failed_steps])}"
        
        return summary
    
    def get_status(self) -> Dict[str, Any]:
        """
        Gets the current status of the Agent. / 获取Agent当前状态。
        """
        successful_tasks = len([t for t in self.execution_history if t.get("success", False)])
        total_tasks = len(self.execution_history)

        return {
            "current_task": self.current_task,
            "total_tasks": total_tasks,
            "successful_tasks": successful_tasks,
            "success_rate": f"{(successful_tasks/total_tasks*100):.1f}%" if total_tasks > 0 else "0%",
            "recent_tasks": self.execution_history[-3:] if self.execution_history else []
        }

    def _format_tool_output_for_feedback(self, tool_output) -> str:
        """
        :param tool_output: The ToolOutput object returned after tool execution. / 工具执行后返回的ToolOutput对象。
        :return: A formatted string ready for display on the frontend. / 一个格式化好的、准备在前端显示的字符串。
        """
        tool_name = tool_output.tool_name
        data = tool_output.to_dict().get('data', {})

        header = f"   Tool '{tool_name}' executed successfully. Output:"
        
        # Customize clear and specific feedback formats for different tools. / 为不同工具定制专属的、清晰的反馈格式。
        if tool_name == 'generate_image' and isinstance(data, dict):
            saved_paths = data.get('saved_paths', [])
            if saved_paths:
                return f"{header}\n   Image saved to: {saved_paths}"
            else:
                return f"{header}\n   Image generation succeeded, but failed to get the save path."

        elif tool_name in ['seewhat', 'analyze_video']:
            description = str(data)[:500]
            return f"{header}\n---\n{description}\n---"
            
        elif tool_name == 'scan_table' and isinstance(data, str):
            table_content = data[:500]
            return f"{header}\n---\n{table_content}\n---"

        else:
            try:
                if isinstance(data, dict):
                    pretty_output = json.dumps(data, ensure_ascii=False, indent=2)
                else:
                    pretty_output = str(data)
                return f"{header}\n---\n{pretty_output[:500]}\n---"
            except TypeError:
                return f"{header}\n---\n{str(data)[:500]}\n---"
