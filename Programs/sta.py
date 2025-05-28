import ast

string = '[[(157, 132), (164, 336), (225, 59), (245, 197), (316, 242), (328, 228), (330, 396), (427, 305), (435, 147)], ["red", "red", "red", "green", "red", "red", "green", "red", "green"]]'

array = ast.literal_eval(string)
pos, c = array

print(pos)