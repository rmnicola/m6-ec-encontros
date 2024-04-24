from collections import deque

# Criando e utilizando a pilha
stack = deque()
stack.append(10)  # Push
stack.append(20)  # Push
stack.append(30)  # Push

print("Stack contents:", list(stack))

stack.pop()  # Pop
print("Stack after pop:", list(stack))
