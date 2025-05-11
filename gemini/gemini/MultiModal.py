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
import io
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
CHUNK_SIZE = 4096  # Increased from 2048 to reduce underruns
RECEIVED_AUDIO_BUFFER = 11520  # Google returns 11520 bytes of audio data at a time

# Set up debug logging
import logging
logging.basicConfig(
    level=logging.INFO,  # Changed from DEBUG to INFO to reduce log volume
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler()]
)


GOOGLE_API_KEY = os.environ.get('GOOGLE_API_KEY')
MODEL = "models/gemini-2.0-flash-live-001"
DEFAULT_MODE = "camera"

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
tools = [{'google_search': {}}, {'code_execution': {}}, {"function_declarations": [navigate_to_location]}]

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
                text=f"""Your name is Wilson. 
                        You are my robot companion. 
                        You have a deep voice and talk quickly. 
                        You call me sir. 
                        You can see and hear me, and I can see and hear you.
                        You can use the google search tool to find information on the internet.
                        You can use the code execution tool to run Python code and use ROS2 functionality.
                        You can also navigate to specific locations in the house.
                        When asked to navigate somewhere, use the navigate_to_location tool with the location name and coordinates,
                        and respond with a message like: "Right away sir, on my way to the kitchen.". Be creative with your responses.
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
        activity_handling = types.ActivityHandling.NO_INTERRUPTION,  # Default behavior: user activity interrupts AI
    ),
    
)

pya = pyaudio.PyAudio()

class MultiModalGeminiNode(Node):
    def __init__(self, video_mode=DEFAULT_MODE):
        super().__init__('MultiModalGeminiNode')
        self.video_mode = video_mode  # Changed from None to use the provided parameter
        self.audio_in_queue = None
        self.out_queue = None
        self.session = None
        self.send_text_task = None
        self.receive_audio_task = None
        self.play_task = None
        
        # Control flags for audio management
        self.mic_active = True
        self.mic_lock = asyncio.Lock()
        
        # Set up node-specific logger
        self.logger = logging.getLogger('gemini_node')
        self.logger.setLevel(logging.INFO)  # Changed from DEBUG to INFO
        
        self.get_logger().info('Gemini Multi-Modal Node started')
        self.logger.info('Python logger initialized')
        
        # Print audio device info for debugging
        self.print_audio_devices()
        
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
        try:
            self.logger.info("Starting send_realtime task")
            print("🎙️ Starting realtime message processing")
            
            msg_counter = 0
            while True:
                try:
                    msg = await self.out_queue.get()
                    msg_counter += 1
  
                    # Log occasional stats - reduced frequency for less noise
                    if msg_counter % 200 == 0:
                        self.logger.info(f"Sent {msg_counter} messages to Gemini API")
                        if msg.get('mime_type') == 'audio/pcm':
                            print(f"🎙️ Sent {msg_counter} audio chunks to Gemini")
                        elif msg.get('mime_type') == 'image/jpeg':
                           print(f"📸 Sent {msg_counter} image frames to Gemini")

                    await self.session.send(input=msg)
                except Exception as e:
                    self.logger.error(f"Error sending message to Gemini: {str(e)}")
                    print(f"🔴 Error sending to Gemini: {str(e)}")
                    # Sleep briefly to avoid high CPU usage in case of persistent errors
                    await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.logger.info("send_realtime task cancelled")
            raise
        except Exception as e:
            self.logger.error(f"Fatal error in send_realtime: {str(e)}")
            print(f"🔴 Fatal error in send_realtime: {str(e)}")
            traceback.print_exc()


    async def listen_audio(self):
        try:
            self.logger.info("Starting audio input stream")
            print("🎤 Initializing microphone...")
            
            mic_info = pya.get_default_input_device_info()
            self.logger.info(f"Using microphone: {mic_info['name']} (index: {mic_info['index']})")
            print(f"🎤 Using microphone: {mic_info['name']}")
            
            # Additional buffer management for stability
            self.audio_stream = await asyncio.to_thread(
                pya.open,
                format=FORMAT,
                channels=CHANNELS,
                rate=SEND_SAMPLE_RATE,
                input=True,
                input_device_index=mic_info["index"],
                frames_per_buffer=CHUNK_SIZE,
            )
            self.logger.info(f"Audio input stream opened with rate={SEND_SAMPLE_RATE}, format={FORMAT}, channels={CHANNELS}")
            print("🎤 Microphone initialized successfully")
            
            # Remove exception_on_overflow to prevent audio processing issues
            counter = 0
            while True:
                try:
                    # Check if mic should be active
                    async with self.mic_lock:
                        mic_currently_active = self.mic_active
                    
                    if mic_currently_active:
                        # Higher latency but more stability with a slight delay
                        await asyncio.sleep(0.01)  # Small sleep to prevent CPU overuse
                        data = await asyncio.to_thread(self.audio_stream.read, CHUNK_SIZE, exception_on_overflow=False)
                        await self.out_queue.put({"data": data, "mime_type": "audio/pcm"})
                        
                        # Only log occasionally to reduce noise
                        counter += 1
                        if counter % 500 == 0:  # Reduced from 500 for even less logging
                            self.logger.debug(f"Audio input: processed {counter} chunks")
                    else:
                        # Microphone is muted, just wait a bit before checking again
                        await asyncio.sleep(0.1)
                except Exception as e:
                    self.logger.error(f"Error reading audio: {str(e)}")
                    print(f"🔴 Microphone error: {str(e)}")
                    await asyncio.sleep(0.1)
        except Exception as e:
            self.logger.error(f"Error initializing audio: {str(e)}")
            print(f"🔴 Failed to initialize microphone: {str(e)}")
            traceback.print_exc()
        finally:
            if hasattr(self, 'audio_stream') and self.audio_stream:
                self.logger.info("Closing audio input stream")
                self.audio_stream.close()
    
# Recieve and Play Audio
    async def receive_audio(self):
        """Background task to reads from the websocket and write pcm chunks to the output queue"""
        try:
            self.logger.info("Starting receive_audio task")
            print("🤖 Initializing Gemini response handler")
            
            msg_counter = 0
            tool_call_counter = 0
            
            while True:
                try:
                    self.logger.debug("Waiting for response from Gemini API")
                    turn = self.session.receive()
                    self.logger.debug("Got new turn from Gemini API")
                    
                    async for response in turn:
                        msg_counter += 1
                        if data := response.data:
                            # Debug log only every 20th chunk to reduce log volume
                            if msg_counter % 20 == 0:
                                self.logger.debug(f"Received audio data chunk of size {len(data)}")
                            self.audio_in_queue.put_nowait(data)
                            continue
                            
                        if text := response.text:
                            print(text, end="")
                            self.logger.debug(f"Received text: {text}")
                            
                        # Process tool calls
                        if hasattr(response, 'tool_call') and response.tool_call:
                            tool_call_counter += 1
                            print(f"\n🛠️ Tool call received (#{tool_call_counter})")
                            self.logger.info(f"Tool call received: #{tool_call_counter}")
                            
                            function_responses = []
                            for fc in response.tool_call.function_calls:
                                print(f"📋 Function: {fc.name}, ID: {fc.id}")
                                print(f"📋 Arguments: {fc.args}")
                                self.logger.info(f"Function call: {fc.name}, args: {fc.args}")
                                
                                # Process the tool call and get result
                                result = self.get_tool_response(fc.name, fc.args)
                                self.logger.info(f"Tool response: {result}")
                                
                                function_response = types.FunctionResponse(
                                    id=fc.id,
                                    name=fc.name,
                                    response={"result": result}
                                )
                                function_responses.append(function_response)

                            # Send all function responses back to the model
                            if function_responses:
                                print(f"📤 Sending tool responses back to Gemini")
                                await self.session.send_tool_response(function_responses=function_responses)
                                self.logger.info(f"Sent {len(function_responses)} tool responses to Gemini")

                    # If you interrupt the model, it sends a turn_complete.
                    # For interruptions to work, we need to stop playback.
                    # Only clear the queue if explicitly instructed or if there's an error
                    # Don't clear during normal conversation as it cuts off audio
                    if hasattr(response, 'turn_end') and response.turn_end:
                        queue_size = self.audio_in_queue.qsize()
                        if queue_size > 0:
                            self.logger.info(f"Turn ended, but preserving {queue_size} audio items to finish playback")
                            print(f"🔄 Turn complete, finishing audio playback ({queue_size} chunks remaining)")
                              
                    # Log occasional stats
                    if msg_counter % 50 == 0:  # Reduced frequency (was 20)
                        self.logger.info(f"Processed {msg_counter} messages from Gemini API")
                        print(f"📊 Processed {msg_counter} messages, {tool_call_counter} tool calls")
                        
                except Exception as e:
                    self.logger.error(f"Error in receive_audio: {str(e)}")
                    print(f"🔴 Error receiving from Gemini: {str(e)}")
                    traceback.print_exc()
                    await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            # Handle task cancellation gracefully
            self.logger.info("Receive audio task cancelled")
            print("🛑 Receive audio task cancelled")
            raise
        except Exception as e:
            self.logger.error(f"Fatal error in receive_audio: {str(e)}")
            print(f"🔴 Fatal error in receive_audio: {str(e)}")
            traceback.print_exc()

    async def play_audio(self):
        try:
            self.logger.info("Starting audio output stream")
            print("🔊 Initializing audio output...")
            
            # Get default output device info for logging
            output_info = pya.get_default_output_device_info()
            self.logger.info(f"Using output device: {output_info['name']} (index: {output_info['index']})")
            print(f"🔊 Using speaker: {output_info['name']}")
            
            # Increased buffer size to prevent underruns
            stream = await asyncio.to_thread(
                pya.open,
                format=FORMAT,
                channels=CHANNELS,
                rate=RECEIVE_SAMPLE_RATE,
                output=True,
                frames_per_buffer=RECEIVED_AUDIO_BUFFER,  # Double the buffer size
            )
            self.logger.info(f"Audio output stream opened with rate={RECEIVE_SAMPLE_RATE}, format={FORMAT}, channels={CHANNELS}")
            print("🔊 Audio output initialized successfully")
            
            audio_chunks_played = 0
            audio_playing = False
            
            while True:
                try:
                    bytestream = await self.audio_in_queue.get()
                    
                    # If this is the first audio chunk in a sequence, mute the microphone
                    if not audio_playing:
                        async with self.mic_lock:
                            self.mic_active = False
                            audio_playing = True
                            print("🔇 Microphone muted while audio is playing")
                            self.logger.info("Microphone disabled during audio playback")
                    
                    # Only log occasionally to reduce noise
                    if audio_chunks_played % 20 == 0:
                        self.logger.debug(f"Got audio chunk of size {len(bytestream)} bytes")
                    
                    # Prevent running another operation while writing audio to avoid underruns
                    await asyncio.to_thread(stream.write, bytestream)
                    audio_chunks_played += 1
                    
                    # Check if the queue is now empty (reached end of audio)
                    if self.audio_in_queue.qsize() == 0:
                        # Wait briefly to make sure no more chunks are coming
                        await asyncio.sleep(1.5)
                        if self.audio_in_queue.qsize() == 0:
                            # No more audio chunks, re-enable microphone if it was disabled
                            async with self.mic_lock:
                                if not self.mic_active:
                                    self.mic_active = True
                                    audio_playing = False
                                    print("🎤 Microphone unmuted after audio playback")
                                    self.logger.info("Microphone re-enabled after audio playback")
                    
                    # Log occasional stats
                    if audio_chunks_played % 50 == 0:  # Reduced frequency (was 20)
                        print(f"🔊 Played {audio_chunks_played} audio chunks")
                        self.logger.debug(f"Audio output: played {audio_chunks_played} chunks")
                except Exception as e:
                    self.logger.error(f"Error playing audio: {str(e)}")
                    print(f"🔴 Audio output error: {str(e)}")
                    
                    # Re-enable microphone in case of error
                    async with self.mic_lock:
                        self.mic_active = True
                        audio_playing = False
                        print("🎤 Microphone unmuted after audio error")
                    
                    await asyncio.sleep(0.1)
        except Exception as e:
            self.logger.error(f"Error initializing audio output: {str(e)}")
            print(f"🔴 Failed to initialize audio output: {str(e)}")
            traceback.print_exc()
        finally:
            # Make sure to re-enable microphone before closing
            async with self.mic_lock:
                self.mic_active = True
                print("🎤 Microphone unmuted during cleanup")
                
            if 'stream' in locals() and stream:
                self.logger.info("Closing audio output stream")
                stream.stop_stream()
                stream.close()


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
            self.logger.info(f'Navigation request: {location} at ({x}, {y})')
            
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
            
    def print_audio_devices(self):
        """Print information about audio devices for debugging"""
        try:
            print("\n===== AUDIO DEVICE INFO =====")
            info = pya.get_host_api_info_by_index(0)
            num_devices = info.get('deviceCount')
            
            input_devices = []
            output_devices = []
            
            for i in range(num_devices):
                device_info = pya.get_device_info_by_host_api_device_index(0, i)
                if device_info.get('maxInputChannels') > 0:
                    input_devices.append((i, device_info.get('name')))
                if device_info.get('maxOutputChannels') > 0:
                    output_devices.append((i, device_info.get('name')))
            
            print("Input devices:")
            for idx, name in input_devices:
                if idx == pya.get_default_input_device_info().get('index'):
                    print(f" * {idx}: {name} (DEFAULT)")
                else:
                    print(f"   {idx}: {name}")
                    
            print("\nOutput devices:")
            for idx, name in output_devices:
                if idx == pya.get_default_output_device_info().get('index'):
                    print(f" * {idx}: {name} (DEFAULT)")
                else:
                    print(f"   {idx}: {name}")
                    
            print("===== END AUDIO DEVICE INFO =====\n")
        except Exception as e:
            print(f"Error getting audio device info: {str(e)}")

    async def run_session(self):
        # Connect to the API
        self.logger.info("Connecting to Gemini API...")
        print("\n🚀 Connecting to Gemini API...")
        
        try:
            async with client.aio.live.connect(model=MODEL, config=CONFIG) as session:
                self.logger.info("Connected to Gemini API!")
                print("✅ Connected to Gemini API successfully!")
                print("🤖 Gemini model: " + MODEL)
                print("💬 Type your messages at the prompt. Type 'q' to exit.\n")
                
                while rclpy.ok():
                    try:
                        # Process any pending callbacks
                        rclpy.spin_once(self, timeout_sec=0.1)
                        
                        self.session = session
                        self.audio_in_queue = asyncio.Queue()
                        self.out_queue = asyncio.Queue()
                        
                        self.logger.info("Creating and starting tasks")
                        print("🔄 Starting audio and processing tasks...")

                        # Create and gather tasks without using TaskGroup (for Python 3.10 compatibility)
                        tasks = []
                        
                        send_text_task = asyncio.create_task(self.send_text())
                        tasks.append(send_text_task)
                        self.logger.debug("Created send_text task")
                        
                        tasks.append(asyncio.create_task(self.send_realtime()))
                        self.logger.debug("Created send_realtime task")
                        
                        tasks.append(asyncio.create_task(self.listen_audio()))
                        self.logger.debug("Created listen_audio task")
                        
                        if self.video_mode == "camera":
                            tasks.append(asyncio.create_task(self.get_frames()))
                            self.logger.info(f"Created get_frames task with video_mode={self.video_mode}")
                            print("📷 Camera mode enabled")
                        else:
                            self.logger.info(f"Skipping video frames, video_mode={self.video_mode}")
                        
                        tasks.append(asyncio.create_task(self.receive_audio()))
                        self.logger.debug("Created receive_audio task")
                        
                        tasks.append(asyncio.create_task(self.play_audio()))
                        self.logger.debug("Created play_audio task")
                        
                        print("✅ All tasks started successfully!")
                        self.logger.info("All tasks started, waiting for user exit signal")

                        # Wait for the send_text_task to complete
                        await send_text_task
                        self.logger.info("User exited text input, cancelling other tasks")
                        print("\n🛑 Stopping system...")
                        
                        # Cancel all other tasks
                        for task in tasks:
                            if not task.done():
                                task.cancel()
                                self.logger.debug(f"Cancelled task: {task}")
                        
                        # Wait for all tasks to be cancelled
                        self.logger.debug("Waiting for all tasks to complete cancellation")
                        await asyncio.gather(*tasks, return_exceptions=True)
                        
                        # User requested exit (typed 'q')
                        self.logger.info("User requested exit")
                        print("✅ All tasks stopped cleanly")
                        break
                        
                    except Exception as e:
                        if isinstance(e, asyncio.CancelledError):
                            self.logger.info("Tasks cancelled")
                            print("🛑 Tasks cancelled")
                            break
                        self.logger.error(f"Error in run_session loop: {str(e)}")
                        self.logger.error(traceback.format_exc())
                        print(f"🔴 Error in main loop: {str(e)}")
                        print("⏳ Restarting in 1 second...")
                        await asyncio.sleep(1.0)  # Avoid busy-waiting in case of errors
                        
        except Exception as e:
            self.logger.error(f"Error connecting to Gemini API: {str(e)}")
            self.logger.error(traceback.format_exc())
            print(f"🔴 Failed to connect to Gemini API: {str(e)}")
            print("Check your API key and internet connection.")


def main(args=None):
    parser = argparse.ArgumentParser(description='MultiModal Gemini Node')
    parser.add_argument('--video', choices=['none', 'camera', 'screen'], default=DEFAULT_MODE, 
                        help='Video input mode (none, camera, or screen)')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    parsed_args, remaining = parser.parse_known_args(args)
    
    # Set logging level based on command line argument
    if parsed_args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        print("🔍 Debug logging enabled")
    
    print("\n===== 🤖 Gemini Multi-Modal ROS2 Node =====")
    print(f"🔑 API Key: {'✓ Set' if GOOGLE_API_KEY else '❌ Missing'}")
    
    if not GOOGLE_API_KEY:
        print("\n❌ ERROR: GOOGLE_API_KEY environment variable not set.")
        print("Please set your API key with:\n")
        print("export GOOGLE_API_KEY=your_api_key_here\n")
        return
    
    rclpy.init(args=remaining)
    node = MultiModalGeminiNode(video_mode=parsed_args.video)
    
    try:
        print(f"🎬 Starting node with video mode: {parsed_args.video}")
        asyncio.run(node.run_session())
    except KeyboardInterrupt:
        print("\n⛔ Interrupted by user (KeyboardInterrupt)")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {str(e)}")
        node.get_logger().error(traceback.format_exc())
        print(f"\n🔴 Fatal error: {str(e)}")
        traceback.print_exc()
    finally:
        print("\n🛑 Shutting down Gemini Multi-Modal Node")
        node.get_logger().info('Shutting down Gemini Multi-Modal Node')
        # Clean up any audio resources
        if hasattr(node, 'audio_stream') and node.audio_stream:
            node.audio_stream.close()
            print("🎤 Closed audio input stream")
        # Destroy the node and shut down ROS
        node.destroy_node()
        rclpy.shutdown()
        print("✅ ROS2 node shut down successfully")
        print("\n===== End of Session =====\n")

if __name__ == "__main__":
    main()
