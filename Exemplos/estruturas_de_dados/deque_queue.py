from collections import deque

# Criando e utilizando a fila
queue = deque()
queue.append(10)  # Enqueue
queue.append(20)  # Enqueue
queue.append(30)  # Enqueue

print("Queue contents:", list(queue))

queue.popleft()  # Dequeue
print("Queue after dequeue:", list(queue))
