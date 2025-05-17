from ollama import chat
from ollama import ChatResponse

def translate(text):
    response: ChatResponse = chat(model='xarm_sorting', messages=[
    {
        'role': 'user',
        'content': text,
    },
    ])

    real = response['message']['content'].split("</think>")
    return(real[1])
# or access fields directly from the response object
