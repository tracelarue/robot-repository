import rclpy
from rclpy.node import Node
import base64
import json
import os
import queue
import socket
import subprocess
import threading
import time
import pyaudio
import socks
import websocket

# Set up SOCKS5 proxy
socket.socket = socks.socksocket

# Use the provided OpenAI API key and URL
API_KEY = os.getenv("OPENAI_API_KEY")
if not API_KEY:
    raise ValueError("API key is missing. Please set the 'OPENAI_API_KEY' environment variable.")

WS_URL = 'wss://api.openai.com/v1/realtime?model=gpt-4o-mini-realtime-preview'

CHUNK_SIZE = 1024
MIC_RATE = 48000  # Update to a supported sample rate
SPEAKER_RATE = 48000 # Update to a supported sample rate
FORMAT = pyaudio.paInt16

audio_buffer = bytearray()
mic_queue = queue.Queue()

stop_event = threading.Event()

mic_on_at = 0
mic_active = None
REENGAGE_DELAY_MS = 500

# Function to clear the audio buffer
def clear_audio_buffer():
    global audio_buffer
    audio_buffer = bytearray()
    print('🔵 Audio buffer cleared.')

# Function to stop audio playback
def stop_audio_playback():
    global is_playing
    is_playing = False
    print('🔵 Stopping audio playback.')

# Function to handle microphone input and put it into a queue
def mic_callback(in_data, frame_count, time_info, status):
    global mic_on_at, mic_active

    if mic_active != True:
        print('🎙️🟢 Mic active')
        mic_active = True
    mic_queue.put(in_data)

    return (None, pyaudio.paContinue)


# Function to send microphone audio data to the WebSocket
def send_mic_audio_to_websocket(ws):
    try:
        while not stop_event.is_set():
            if not mic_queue.empty():
                mic_chunk = mic_queue.get()
                encoded_chunk = base64.b64encode(mic_chunk).decode('utf-8')
                message = json.dumps({'type': 'input_audio_buffer.append', 'audio': encoded_chunk})
                try:
                    ws.send(message)
                except Exception as e:
                    print(f'Error sending mic audio: {e}')
    except Exception as e:
        print(f'Exception in send_mic_audio_to_websocket thread: {e}')
    finally:
        print('Exiting send_mic_audio_to_websocket thread.')


# Function to handle audio playback callback
def speaker_callback(in_data, frame_count, time_info, status):
    global audio_buffer, mic_on_at

    bytes_needed = frame_count * 2  # 2 channels * 2 bytes per sample
    current_buffer_size = len(audio_buffer)

    if current_buffer_size >= bytes_needed:
        audio_chunk = bytes(audio_buffer[:bytes_needed])
        audio_buffer = audio_buffer[bytes_needed:]
        mic_on_at = time.time() + REENGAGE_DELAY_MS / 1000
    else:
        audio_chunk = bytes(audio_buffer) + b'\x00' * (bytes_needed - current_buffer_size)
        audio_buffer.clear()

    # Duplicate mono audio to stereo
    if len(audio_chunk) % 4 == 0:  # Ensure it's divisible by 4 (2 channels * 2 bytes)
        stereo_chunk = b''.join([audio_chunk[i:i+2] * 2 for i in range(0, len(audio_chunk), 2)])
    else:
        stereo_chunk = audio_chunk

    return (stereo_chunk, pyaudio.paContinue)


# Function to receive audio data from the WebSocket and process events
def receive_audio_from_websocket(ws):
    global audio_buffer

    try:
        while not stop_event.is_set():
            try:
                message = ws.recv()
                if not message:  # Handle empty message (EOF or connection close)
                    print('🔵 Received empty message (possibly EOF or WebSocket closing).')
                    break

                # Now handle valid JSON messages only
                message = json.loads(message)
                event_type = message['type']
                print(f'⚡️ Received WebSocket event: {event_type}')

                if event_type == 'session.created':
                    send_fc_session_update(ws)

                elif event_type == 'response.audio.delta':
                    audio_content = base64.b64decode(message['delta'])
                    audio_buffer.extend(audio_content)
                    print(f'🔵 Received {len(audio_content)} bytes, total buffer size: {len(audio_buffer)}')

                elif event_type == 'input_audio_buffer.speech_started':
                    print('🔵 Speech started, clearing buffer and stopping playback.')
                    clear_audio_buffer()
                    stop_audio_playback()

                elif event_type == 'response.audio.done':
                    print('🔵 AI finished speaking.')

                elif event_type == 'response.function_call_arguments.done':
                    handle_function_call(message,ws)

                elif event_type == 'error':
                    print(f"⚠️ WebSocket error: {message}")

            except Exception as e:
                print(f'Error receiving audio: {e}')
    except Exception as e:
        print(f'Exception in receive_audio_from_websocket thread: {e}')
    finally:
        print('Exiting receive_audio_from_websocket thread.')


# Function to handle function calls
def handle_function_call(event_json, ws):
    try:

        name= event_json.get("name","")
        call_id = event_json.get("call_id", "")

        arguments = event_json.get("arguments", "{}")
        function_call_args = json.loads(arguments)

        if name == "navigate_and_confirm":
            # Extract arguments from the event JSON
            coordinates = function_call_args.get("coordinates", {})
            audio_confirmation = function_call_args.get("audio_confirmation", "")

            # Extract the call_id from the event JSON

            # If the coordinates are provided, send them back
            if coordinates:
                x = coordinates.get("x", 0)
                y = coordinates.get("y", 0)
                result = f"{x}, {y}"
                send_function_call_result(result, call_id, ws)
            else:
                print("Coordinates not provided for navigate_and_confirm function.")
    except Exception as e:
        print(f"Error parsing function call arguments: {e}")

# Function to send the result of a function call back to the server
def send_function_call_result(result, call_id, ws):
    # Create the JSON payload for the function call result
    result_json = {
        "type": "conversation.item.create",
        "item": {
            "type": "function_call_output",
            "output": result,
            "call_id": call_id
        }
    }

    # Convert the result to a JSON string and send it via WebSocket
    try:
        ws.send(json.dumps(result_json))
        print(f"Sent function call result: {result_json}")

        # Create the JSON payload for the response creation and send it
        rp_json = {
            "type": "response.create"
        }
        ws.send(json.dumps(rp_json))
        print(f"json = {rp_json}")
    except Exception as e:
        print(f"Failed to send function call result: {e}")

# Function to send session configuration updates to the server
def send_fc_session_update(ws):
    session_config = {
        "type": "session.update",
        "session": {
            "instructions": (
                "When you are told to go somewhere. confirm that you received the command and state it is being executed without announcing arrival at the location."
                "Your voice should sound like a british robot. "
                "Talk quickly. "
                "You should always call a function if you can. "
                "Kitchen: x=10, y=5"
                "Living room: x=20, y=15"
                "Refrigerator: x=5, y=7"
            ),
            "turn_detection": {
                "type": "server_vad",
                "threshold": 0.1,
                "prefix_padding_ms": 300,
                "silence_duration_ms": 500
            },
            "voice": "ash",
            "temperature": 1,
            "max_response_output_tokens": 4096,
            "modalities": ["text", "audio"],
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {
                "model": "whisper-1"
            },
            "input_audio_noise_reduction": {
                "type": "near_field"
            },
            "tool_choice": "auto",
            "tools": [
                {
                    "type": "function",
                    "name": "navigate_and_confirm",
                    "description": "Use this tool when told to go somewhere or do something at that place. Responds with the coordinates of a location and an audio confirmation when commanded to navigate to or perform a task at that location.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                        "coordinates": {
                            "type": "object",
                            "properties": {
                            "x": {
                                "type": "number",
                                "description": "X coordinate of the location"
                            },
                            "y": {
                                "type": "number",
                                "description": "Y coordinate of the location"
                            }
                            },
                            "description": "The coordinates of the location to navigate to"
                        },
                        "audio_confirmation": {
                            "type": "string",
                            "description": "Audio confirmation message that command has been recieved and is being executed."
                        }
                        },
                        "required": ["coordinates", "audio_confirmation"]
                    }
                    }
            ]
        }
    }
    # open notepad fc

    # Convert the session config to a JSON string
    session_config_json = json.dumps(session_config)
    print(f"Send FC session update: {session_config_json}")

    # Send the JSON configuration through the WebSocket
    try:
        ws.send(session_config_json)
    except Exception as e:
        print(f"Failed to send session update: {e}")



# Function to create a WebSocket connection using IPv4
def create_connection_with_ipv4(*args, **kwargs):
    # Enforce the use of IPv4
    original_getaddrinfo = socket.getaddrinfo

    def getaddrinfo_ipv4(host, port, family=socket.AF_INET, *args):
        return original_getaddrinfo(host, port, socket.AF_INET, *args)

    socket.getaddrinfo = getaddrinfo_ipv4
    try:
        return websocket.create_connection(*args, **kwargs)
    finally:
        # Restore the original getaddrinfo method after the connection
        socket.getaddrinfo = original_getaddrinfo

# Function to establish connection with OpenAI's WebSocket API
def connect_to_openai():
    ws = None
    try:
        ws = create_connection_with_ipv4(
            WS_URL,
            header=[
                f'Authorization: Bearer {API_KEY}',
                'OpenAI-Beta: realtime=v1'
            ]
        )
        print('Connected to OpenAI WebSocket.')

        # Start the recv and send threads
        receive_thread = threading.Thread(target=receive_audio_from_websocket, args=(ws,))
        receive_thread.start()

        mic_thread = threading.Thread(target=send_mic_audio_to_websocket, args=(ws,))
        mic_thread.start()

        # Wait for stop_event to be set
        while not stop_event.is_set():
            time.sleep(0.1)

        # Send a close frame and close the WebSocket gracefully
        print('Sending WebSocket close frame.')
        ws.send_close()

        receive_thread.join()
        mic_thread.join()

        print('WebSocket closed and threads terminated.')
    except Exception as e:
        print(f'Failed to connect to OpenAI: {e}')
        print('Retrying connection in 5 seconds...')
        time.sleep(5)
        connect_to_openai()  # Retry connection
    finally:
        if ws is not None:
            try:
                ws.close()
                print('WebSocket connection closed.')
            except Exception as e:
                print(f'Error closing WebSocket connection: {e}')



# Main function to start audio streams and connect to OpenAI
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('openai_realtime_node')
    node.get_logger().info('Starting OpenAI realtime node…')

    p = pyaudio.PyAudio()

    # List available audio devices
    print("Available audio devices:")
    mic_device_index = None
    speaker_device_index = None

    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        print(f"Device {i}: {device_info['name']} (Input Channels: {device_info['maxInputChannels']}, Output Channels: {device_info['maxOutputChannels']})")

        # Automatically select the correct microphone (card 2, device 0)
        if "usb microphone" in device_info['name'].lower() and device_info['maxInputChannels'] > 0:
            mic_device_index = i

        # Automatically select the correct speaker (card 3, device 0)
        if "uacdemov1.0" in device_info['name'].lower() and device_info['maxOutputChannels'] > 0:
            speaker_device_index = i

    if mic_device_index is None:
        print("Error: Could not find the USB microphone.")
        return

    if speaker_device_index is None:
        print("Error: Could not find the USB speaker.")
        return

    print(f"Using microphone: Device {mic_device_index}")
    print(f"Using speaker: Device {speaker_device_index}")

    # Open microphone stream
    mic_stream = p.open(
        format=FORMAT,
        channels=1,
        rate=MIC_RATE,
        input=True,
        input_device_index=mic_device_index,  # Use the selected microphone
        stream_callback=mic_callback,
        frames_per_buffer=CHUNK_SIZE
    )

    # Open speaker stream
    speaker_stream = p.open(
        format=FORMAT,
        channels=2,
        rate=SPEAKER_RATE,
        output=True,
        output_device_index=speaker_device_index,  # Use the selected speaker
        stream_callback=speaker_callback,
        frames_per_buffer=CHUNK_SIZE
    )

    try:
        mic_stream.start_stream()
        speaker_stream.start_stream()
        connect_to_openai()

        # Run until ROS shutdown or streams stop
        while rclpy.ok() and mic_stream.is_active() and speaker_stream.is_active():
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info('Gracefully shutting down…')
        stop_event.set()

    finally:
        mic_stream.stop_stream()
        mic_stream.close()
        speaker_stream.stop_stream()
        speaker_stream.close()
        p.terminate()
        node.get_logger().info('Audio streams stopped and resources released. Exiting.')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()