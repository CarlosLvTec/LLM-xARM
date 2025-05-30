from ollama import ChatResponse, chat
from random import randint
from xarm_basic import set_position, grab, drop
from quaroni_vision import get_layout

def move(characteristics, end_position):
  '''
  Is a movement protocol that takes one specific 
  object to a specific position.
  '''
  n,c,pos = get_layout()

  for i in range (0,len(pos)):
    if characteristics in c[i]:
      set_position(pos[i])
      grab()
      set_position(end_position + (randint(1,10),randint(1,10)))
      '''
      El número random es para que no ponga todos los objetos
      en la misma posición, posible rutina para mejorar
      '''
      drop()

  return "Complete"

def add_two_numbers(a: int, b: int) -> int:
  """
  Add two numberseccem@ubuntu:~/llm_xarm/LLM-xARM$ git push
To github.com:CarlosLvTec/LLM-xARM.git
 ! [rejected]        main -> main (non-fast-forward)
error: failed to push some refs to 'git@github.com:CarlosLvTec/LLM-xARM.git'
hint: Updates were rejected because the tip of your current branch is behind
hint: its remote counterpart. Integrate the remote changes (e.g.
hint: 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
teccem@ubuntu:~/llm_xarm/LLM-xARM$ 


  Args:
    a (int): The first number
    b (int): The second number

  Returns:
    int: The sum of the two numbers
  """

  # The cast is necessary as returned tool call arguments don't always conform exactly to schema
  # E.g. this would prevent "what is 30 + 12" to produce '3012' instead of 42
  return int(a) + int(b)


def subtract_two_numbers(a: int, b: int) -> int:
  """
  Subtract two numbers
  """

  # The cast is necessary as returned tool call arguments don't always conform exactly to schema
  return int(a) - int(b)


# Tools can still be manually defined and passed into chat
subtract_two_numbers_tool = {
  'type': 'function',
  'function': {
    'name': 'subtract_two_numbers',
    'description': 'Subtract two numbers',
    'parameters': {
      'type': 'object',
      'required': ['a', 'b'],
      'properties': {
        'a': {'type': 'integer', 'description': 'The first number'},
        'b': {'type': 'integer', 'description': 'The second number'},
      },
    },
  },
}

prompt = input(">>> ")

messages = [{'role': 'user', 'content': prompt}]
print('Prompt:', messages[0]['content'])

available_functions = {
  'add_two_numbers': add_two_numbers,
  'subtract_two_numbers': subtract_two_numbers,
}

response: ChatResponse = chat(
  'xarm_mistral',
  messages=messages,
  tools=[add_two_numbers, subtract_two_numbers_tool],
)

if response.message.tool_calls:
  # There may be multiple tool calls in the response
  for tool in response.message.tool_calls:
    # Ensure the function is available, and then call it
    if function_to_call := available_functions.get(tool.function.name):
      print('Calling function:', tool.function.name)
      print('Arguments:', tool.function.arguments)
      output = function_to_call(**tool.function.arguments)
      print('Function output:', output)
    else:
      print('Function', tool.function.name, 'not found')

# Only needed to chat with the model using the tool call results
if response.message.tool_calls:
  # Add the function response to messages for the model to use
  messages.append(response.message)
  messages.append({'role': 'tool', 'content': str(output), 'name': tool.function.name})

  # Get final response from model with function outputs
  final_response = chat('mistral', messages=messages)
  print('Final response:', final_response.message.content)

else:
  print('No tool calls returned from model')