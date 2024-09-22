import os

from langchain_nvidia_ai_endpoints import ChatNVIDIA
from langchain_core.output_parsers import StrOutputParser

# --- Set the API key ---
os.environ['NVIDIA_API_KEY'] = "Please replace with your own key"

# --- Load model ---
foundation_model="ai-llama3-70b" # Or use ChatNVIDIA.get_available_models() to get a list of available models.
llm = ChatNVIDIA(model=foundation_model, temperature=0.1, max_tokens=100, top_p=1.0)| StrOutputParser()

#######################################
# Query the room label
#######################################

# --- Load the example yaml ---
with open('config/room_prompt_example.yaml', 'r') as file:
    room_prompt_example = file.read()

# --- Construct the example prompt ---
example_prompt = "\
Please only answer the room label change (only one line), Example:\n" + \
'Question:\n' + \
room_prompt_example + '\n' + \
"Answer: R(0) -> bedroom, R(2) -> diving room"

# --- Construct the question ---
room_posible = "bedroom, bathroom, diving room"

question = "\
Here is the basic 3D scene graph with hierarchical inofrmation, \
please label each room into semantic label, room have: " + room_posible

# --- Load the output yaml file ---
with open('config/test.yaml', 'r') as file:
    output = file.read()

# --- Construct the final question ---
question += '\n' + output + '\n\n' + example_prompt

# --- Invoke the model and get the answer ---
answer = llm.invoke(question)
print(question)
print("LLM answer: " + answer + "\n")

#######################################
# Query the goal position
#######################################

# --- Define the question ---
q = "I want to go to sleep, where should I go?"

prompt = "\
Please only answer the position (only in number, don't output english char), \
Example: 4.36, -1.2, 1.22 (round to 2 decimal places, if the number is negative, \
please round to 1 decimal places)"

question = "\
Here is the basic 3D scene graph with hierarchical inofrmation, \
please answer where should we go based on the question,\n\
Q: " + q + '\n'

# --- Construct the final question ---
question += '\n' + output + '\nroom label:' + answer + '\n' + prompt

# --- Invoke the model and get the answer ---
answer = llm.invoke(question)
print(question)
print("LLM answer: ", answer)

# --- Extract the point from the llm answer ---
try:
    point = answer.split(',')
    point = [round(float(p), 2) for p in point]

    # Write the point to the result file
    with open('result/result.txt', 'w') as file:
        file.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]))

except Exception as e:
    print("Error: ", e)