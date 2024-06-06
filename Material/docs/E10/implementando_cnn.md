---
title: Implementando CNN
sidebar_position: 4
---

# Implementando CNN

Para realizar a implementação do nosso modelo do nosso modelo de rede neural convolucional. Vamos carregar uma imagem e processar ela.

```python title="implementando_cnn.py" showLineNumbers=true
import cv2
import numpy as np
# Carrega o modelo
from tensorflow.keras.models import load_model
modelo_2 = load_model('modelo_mnist.h5')

# Usa o modelo para realizar uma predição
img = cv2.imread('imagem-teste.jpeg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# predicao = modelo_2.predict(img.reshape(1, 28, 28, 1))
img = cv2.resize(img, (28,28))
img = img / img.max()
_, img = cv2.threshold(img, img.mean(), 255, cv2.THRESH_BINARY)
plt.imshow(img, cmap="gray_r")
img = img.reshape(1,28,28,1)
predicao = modelo_2.predict(img)
print(predicao)
# Exibe a classe com a maior probabilidade de ser a correta da predição
np.argmax(predicao)
```

  