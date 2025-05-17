from ollama import chat
from ollama import ChatResponse

response: ChatResponse = chat(model='xarm_sorting', messages=[
  {
    'role': 'user',
    'content': 'Place the red squares on top',
  },
])

real = response['message']['content'].split("</think>").split("/")
print(real[1])
# or access fields directly from the response object
