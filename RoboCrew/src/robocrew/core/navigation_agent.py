import cv2
import base64
import numpy as np
import logging
import time
import os
from typing import List, Optional, Dict, Any
from dotenv import load_dotenv
from langchain_core.messages import HumanMessage, SystemMessage, ToolMessage
from langchain.chat_models import init_chat_model

load_dotenv()

from robocrew.core.robot_system import RobotSystem
from robocrew.core.utils import capture_image

logger = logging.getLogger(__name__)

class NavigationAgent:
    def __init__(
        self, 
        robot_system: RobotSystem,
        model_name: str, 
        tools: List[Any],
        system_prompt: str = None,
        history_len: int = 10
    ):
        self.robot = robot_system
        self.tools = tools
        self.tool_map = {t.name: t for t in tools}
        self.history_len = history_len
        
        # Initialize LLM
        self.llm = init_chat_model(model_name).bind_tools(tools)
        
        # System Prompt
        base_prompt = (
            "You are an intelligent mobile robot. Your goal is to navigate safely and efficiently.\n"
            "SAFETY RULES:\n"
            "1. Do not run into walls or obstacles.\n"
            "2. If you are stuck, try moving backward or turning.\n"
            "3. Plan your path before moving.\n"
            "4. If the path is blocked, find an alternative route.\n"
        )
        self.system_prompt = system_prompt or base_prompt
        self.message_history = [SystemMessage(content=self.system_prompt)]
        
        # State
        self.current_task = "Idle"
        self.last_image = None
        self.stuck_counter = 0

    def set_task(self, task: str):
        """Set a new task for the agent."""
        self.current_task = task
        self.message_history.append(HumanMessage(content=f"New Task: {task}"))
        logger.info(f"Agent task set: {task}")

    def _check_safety(self, image: np.ndarray) -> bool:
        """
        Basic CV2 safety check.
        Returns True if safe to proceed, False if obstacle detected.
        """
        if image is None:
            return False
            
        # 1. Check for "stuck" condition (image identical to last frame despite movement)
        # This requires knowing if we moved, which we can track via state or just heuristics
        # For now, let's just check for extremely close obstacles (blur/darkness/uniformity)
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Check bottom area (immediate path)
        h, w = gray.shape
        bottom_area = gray[int(h*0.7):, :]
        
        # Check for low variance (flat wall/floor filling view)
        variance = np.var(bottom_area)
        mean_brightness = np.mean(bottom_area)
        
        # Heuristics for "face against wall"
        # Very low variance often means looking at a blank wall close up
        if variance < 50: 
            logger.warning(f"Safety Warning: Low variance ({variance:.1f}) - Wall likely close")
            return False
            
        return True

    def step(self) -> str:
        """
        Execute one step of the agent loop.
        Returns status string.
        """
        if not self.robot.running:
            return "Robot stopped"
            
        # 1. Capture observation
        frame = self.robot.get_frame()
        if frame is None:
            return "Camera error"
            
        # 2. Safety Check
        if not self._check_safety(frame):
            # If unsafe, override with safety maneuver (e.g., stop or back up)
            # For now, just warn and let LLM decide, but inject warning
            safety_warning = "WARNING: Visual safety check failed. You may be too close to a wall. Consider backing up."
        else:
            safety_warning = ""

        # 3. Prepare Prompt
        _, buffer = cv2.imencode('.jpg', frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        content = [
            {"type": "text", "text": f"Task: {self.current_task}"},
            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}}
        ]
        
        if safety_warning:
            content.append({"type": "text", "text": safety_warning})
            
        self.message_history.append(HumanMessage(content=content))
        
        # 4. LLM Inference
        try:
            response = self.llm.invoke(self.message_history)
            self.message_history.append(response)
            
            # Prune history
            if len(self.message_history) > self.history_len * 2:
                self.message_history = [self.message_history[0]] + self.message_history[-self.history_len:]
            
            # 5. Execute Tools
            if response.tool_calls:
                for tool_call in response.tool_calls:
                    tool_name = tool_call["name"]
                    args = tool_call["args"]
                    logger.info(f"Agent executing: {tool_name}({args})")
                    
                    if tool_name in self.tool_map:
                        tool = self.tool_map[tool_name]
                        try:
                            result = tool.invoke(args)
                        except Exception as e:
                            result = f"Error executing {tool_name}: {e}"
                            
                        self.message_history.append(ToolMessage(content=str(result), tool_call_id=tool_call["id"]))
                    else:
                        logger.error(f"Unknown tool: {tool_name}")
                return f"Executed {len(response.tool_calls)} actions"
            else:
                return "Thinking..."
                
        except Exception as e:
            logger.error(f"Agent step error: {e}")
            return f"Error: {e}"
