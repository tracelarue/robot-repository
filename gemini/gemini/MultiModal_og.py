# usr/bin/env python3.11

"""
## Documentation
Quickstart: https://github.com/google-gemini/cookbook/blob/main/quickstarts/Get_started_LiveAPI.py

## Setup

To install the dependencies for this script, run:

```
pip install google-genai opencv-python pyaudio pillow mss
```
"""

import os
import asyncio
import base64
import json
import traceback
import cv2
import pyaudio
import PIL.Image
import mss
import argparse
from google import genai
from google.genai import types
from google.genai.types import RealtimeInputConfig, ActivityHandling

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


FORMAT = pyaudio.paInt16
CHANNELS = 1
SEND_SAMPLE_RATE = 16000
RECEIVE_SAMPLE_RATE = 24000
CHUNK_SIZE = 2048  # Increased from 2048 to reduce underruns


GOOGLE_API_KEY = os.environ.get('GOOGLE_API_KEY')
MODEL = "models/gemini-2.0-flash-live-001"
DEFAULT_MODE = "none"

client = genai.Client(
    http_options={"api_version": "v1beta"},
    api_key=os.environ.get("GOOGLE_API_KEY"),
)


# Define your tools - adding navigate_to_location
navigate_to_location = {
    "name": "navigate_to_location",
    "description": "Navigate the robot to a specific location",
    "parameters": {
        "type": "object",
        "properties": {
            "location": {
                "type": "string",
                "description": "The location name to navigate to (e.g., 'kitchen', 'living room', 'bedroom')"
            },
            "x": {
                "type": "number",
                "description": "The x-coordinate on the map"
            },
            "y": {
                "type": "number",
                "description": "The y-coordinate on the map"
            }
        },
        "required": ["location", "x", "y"]
    },
}


# Add the new navigation tool to the tools list
tools = [{"function_declarations": [navigate_to_location]}]

# Configuring the AI with navigation information
CONFIG = types.LiveConnectConfig(
    response_modalities=["AUDIO"],
    context_window_compression=(
        # Configures compression with default parameters.
        types.ContextWindowCompressionConfig(
            sliding_window=types.SlidingWindow(),
        )
    ),
    speech_config=types.SpeechConfig(
        voice_config=types.VoiceConfig(
            prebuilt_voice_config=types.PrebuiltVoiceConfig(voice_name="Charon")
        )
    ),

    system_instruction = types.Content(
        parts=[
            types.Part(
                text=f"""You are a helpful robot assistant that can control a ROS2-based robot.
                        When asked to navigate somewhere, use the navigate_to_location tool with the location name and coordinates.
                        Available locations:
                        - kitchen (x=0.88, y=-3.45)
                        - origin (x=0.0, y=0.0)
                        - office (x=-3.9, y=-3.55)
                        

                        """
            )
        ]
    ),
    tools=tools,
    realtime_input_config = RealtimeInputConfig(
        #start_sensitivity = types.StartSensitivity.START_SENSITIVITY_LOW,
        activity_handling = types.ActivityHandling.NO_INTERRUPTION,  # Default behavior: user activity interrupts AI
    ),
    
)

pya = pyaudio.PyAudio()

class MultiModalGeminiNode(Node):
    def __init__(self, video_mode=DEFAULT_MODE):
        super().__init__('MultiModalGeminiNode')
        self.video_mode = None
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.send_text_task = None
        self.receive_audio_task = None
        self.play_task = None
        self.get_logger().info('Gemini Multi-Modal Node started')
        
# Create the action client to send navigation goals
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.wait_for_nav_server()

# Text Input Function
    async def send_text(self):
        while True:
            text = await asyncio.to_thread(
                input,
                "message > ",
            )
            if text.lower() == "q":
                break
            await self.session.send(input=text or ".", end_of_turn=True)

# Vision   
    def _get_frame(self, cap):
        # Read the frameq
        ret, frame = cap.read()
        # Check if the frame was read successfully
        if not ret:
            return None
        # Fix: Convert BGR to RGB color space
        # OpenCV captures in BGR but PIL expects RGB format
        # This prevents the blue tint in the video feed
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = PIL.Image.fromarray(frame_rgb)  # Now using RGB frame
        img.thumbnail([1024, 1024])

        image_io = io.BytesIO()
        img.save(image_io, format="jpeg")
        image_io.seek(0)

        mime_type = "image/jpeg"
        image_bytes = image_io.read()
        return {"mime_type": mime_type, "data": base64.b64encode(image_bytes).decode()}

    async def get_frames(self):
        # This takes about a second, and will block the whole program
        # causing the audio pipeline to overflow if you don't to_thread it.
        cap = await asyncio.to_thread(
            cv2.VideoCapture, 0
        )  # 0 represents the default camera

        while True:
            frame = await asyncio.to_thread(self._get_frame, cap)
            if frame is None:
                break

            await asyncio.sleep(1.0)

            await self.out_queue.put(frame)

        # Release the VideoCapture object
        cap.release()

# Audio Input Function
    async def send_realtime(self):
        while True:
            msg = await self.out_queue.get()
            await self.session.send(input=msg)


    async def listen_audio(self):
        mic_info = pya.get_default_input_device_info()
        self.audio_stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=SEND_SAMPLE_RATE,
            input=True,
            input_device_index=mic_info["index"],
            frames_per_buffer=CHUNK_SIZE,
        )
        # Remove exception_on_overflow to prevent audio processing issues
        while True:
            data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, exception_on_overflow=False)
            await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
    
# Recieve and Play Audio
    async def receive_audio(self):
        "Background task to reads from the websocket and write pcm chunks to the output queue"
        while True:
            turn = self.session.receive()
            async for response in turn:
                if data := response.data:
                    self.audio_in_queue.put_nowait(data)
                    continue
                if text := response.text:
                    print(text, end="")

            # If you interrupt the model, it sends a turn_complete.
            # For interruptions to work, we need to stop playback.
            # So empty out the audio queue because it may have loaded
            # much more audio than has played yet.
            while not self.audio_in_queue.empty():
                self.audio_in_queue.get_nowait()

    async def play_audio(self):
        stream = await asyncio.to_thread(
            pya.open,
            format=FORMAT,
            channels=CHANNELS,
            rate=RECEIVE_SAMPLE_RATE,
            output=True,
        )
        while True:
            bytestream = await self.audio_in_queue.get()
            await asyncio.to_thread(stream.write, bytestream)


# Wait for the action server to be available        
    def wait_for_nav_server(self):
        server_ready = self.nav_to_pose_client.wait_for_server(timeout_sec=5.0)
        if server_ready:
            self.get_logger().info('Nav2 action server is available!')
        else:
            self.get_logger().warn('Nav2 action server not available after waiting. Will try again when needed.')
        return server_ready
            
    def send_nav_goal(self, x, y, yaw=0.0):
        # Create a NavigateToPose action goal
        goal_msg = NavigateToPose.Goal()
        
        # Build the pose
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation using quaternion from yaw angle
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw/2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw/2)
        
        self.get_logger().info(f'Sending navigation goal to x={x}, y={y}, yaw={yaw}')
        
        # Send the goal
        self.nav_to_pose_client.wait_for_server()
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 action server')
            return
        
        self.get_logger().info('Goal accepted by Nav2 action server')
        
        # Request the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info(f'Navigation finished with status {status}')
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')
        
    def get_tool_response(self, tool_name, parameters):
        if tool_name == "navigate_to_location":
            location = parameters.get("location", "unknown")
            x = parameters.get("x", 0.0)
            y = parameters.get("y", 0.0)
            
            # Log the navigation command
            self.get_logger().info(f'Navigating to {location} at coordinates ({x}, {y})')
            
            # Check if Nav2 action server is available, wait if not
            if not self.nav_to_pose_client.server_is_ready():
                server_ready = self.wait_for_nav_server()
                if not server_ready:
                    return f"Navigation to {location} failed: Nav2 action server not available."
            
            # Send the actual navigation goal
            success = self.send_nav_goal(x, y)
            
            if success:
                return f"Navigation initiated to {location} at coordinates ({x}, {y})."
            else:
                return f"Failed to send navigation goal to {location}."
        else:
            return "Unknown tool."

    async def run_session(self):
        # Connect to the API
        async with (
            client.aio.live.connect(model=MODEL, config=CONFIG) as session
        ):
            while rclpy.ok():
                # Process any pending callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                
                self.session = session
                self.audio_in_queue = asyncio.Queue()
                self.out_queue = asyncio.Queue()

                # Create and gather tasks without using TaskGroup (for Python 3.10 compatibility)
                tasks = []
                
                send_text_task = asyncio.create_task(self.send_text())
                tasks.append(send_text_task)
                
                tasks.append(asyncio.create_task(self.send_realtime()))
                tasks.append(asyncio.create_task(self.listen_audio()))
                
                if self.video_mode == "camera":
                    tasks.append(asyncio.create_task(self.get_frames()))
                
                tasks.append(asyncio.create_task(self.receive_audio()))
                tasks.append(asyncio.create_task(self.play_audio()))

                # Wait for the send_text_task to complete
                await send_text_task
                
                # Cancel all other tasks
                for task in tasks:
                    if not task.done():
                        task.cancel()
                
                # Wait for all tasks to be cancelled
                await asyncio.gather(*tasks, return_exceptions=True)
                
                raise asyncio.CancelledError("User requested exit")

                # Process the response from the model
                '''
                async for chunk in session.receive():
                    # Print the tool call
                    if chunk.tool_call:
                        print(f"\nTool call received")
                        function_responses = []
                        for fc in chunk.tool_call.function_calls:
                            # Print details about the function call
                            print(f"Function: {fc.name}, ID: {fc.id}")
                            print(f"Arguments: {fc.args}")
                            
                            # Process the tool call and get result
                            result = self.get_tool_response(fc.name, fc.args)
                            
                            function_response = types.FunctionResponse(
                                id=fc.id,
                                name=fc.name,
                                response={"result": result}
                            )
                            function_responses.append(function_response)

                        # Send all function responses back to the model
                        await session.send_tool_response(function_responses=function_responses)
                '''


def main(args=None):
    rclpy.init(args=args)
    node = MultiModalGeminiNode()
    
    try:
        asyncio.run(node.run_session())
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Gemini Text-to-Text Node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
