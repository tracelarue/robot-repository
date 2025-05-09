import asyncio
import os
import json
from google import genai
from google.genai import types

GOOGLE_API_KEY = os.environ.get('GOOGLE_API_KEY')
client = genai.Client(api_key=GOOGLE_API_KEY)
model = "gemini-2.0-flash-live-001"

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
config = {
    "system_instruction": types.Content(
        parts=[
            types.Part(
                text="""You are a helpful robot assistant that can control a ROS2-based robot.
                        When asked to navigate somewhere, use the navigate_to_location tool with the location name and coordinates.
                        Available locations:
                        - kitchen (x=1.0, y=2.0)
                        - living room (x=3.0, y=1.5)
                        - bedroom (x=4.0, y=-2.0)
                        - office (x=-1.0, y=3.0)
                        - bathroom (x=2.5, y=-1.5)

                        You can also control lights with turn_on_the_lights and turn_off_the_lights functions.
                        """
            )
        ]
    ),
    "response_modalities": ["TEXT"],
    "tools": tools
}

def get_tool_response(tool_name, parameters):
    if tool_name == "navigate_to_location":
        location = parameters.get("location", "unknown")
        x = parameters.get("x", 0.0)
        y = parameters.get("y", 0.0)
        
        # Create the ROS2 navigation command
        navigation_command = f"""ros2 action send_goal /navigate_to_pose \\
            nav2_msgs/action/NavigateToPose \\
                {{pose:{{ 
                    header:{{frame_id: map}},
                    pose:{{
                    position:{{x: {x}, y: {y}, z: 0.0}},
                    orientation:{{z: 0.0, w: 1.0}}
                }}}}}}"""

        print(f"\n[SIMULATION] Executing navigation command for {location}:")
        print(navigation_command)
        
        return f"Navigation initiated to {location} at coordinates ({x}, {y})."
    else:
        return "Unknown tool."

async def main():
# Connect to the API
    async with client.aio.live.connect(model=model, config=config) as session:
        while True:
            message = input("User> ")
            if message.lower() == "exit":
                break

               
            await session.send_client_content(
                turns={"role": "user", "parts": [{"text": message}]},
                turn_complete=True
            )

            # Process the response from the model
            async for chunk in session.receive():

                # Check if the chunk is a server content or a tool call

                # Print the server content
                if chunk.server_content:
                    if chunk.text is not None:
                        print(chunk.text, end="")

                # Print the tool call
                elif chunk.tool_call:
                    print(f"\nTool call received")
                    function_responses = []
                    for fc in chunk.tool_call.function_calls:
                        # Print details about the function call
                        print(f"Function: {fc.name}, ID: {fc.id}")
                        print(f"Arguments: {fc.args}")
                        # Process the tool call and get result
                        # Convert fc.args from string to dictionary if needed
                        
                        
                        result = get_tool_response(fc.name, fc.args)
                        
                        function_response = types.FunctionResponse(
                            id=fc.id,
                            name=fc.name,
                            response={"result": result}
                        )
                        function_responses.append(function_response)

                    # Send all function responses back to the model
                    await session.send_tool_response(function_responses=function_responses)

if __name__ == "__main__":
    asyncio.run(main())
