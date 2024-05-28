---
title: Utilizando MLPs
sidebar_position: 1
sidebar_class_name: ponderada
slug: /ponderada5
---

# Utilizando MLPs

**Atividade com prazo de entrega até 03/06, mas sugiro fortemente já resolver
em sala ;)**

## 1. Objetivo

Implementar um MLP para resolver o problema do XOR.

## 2. Enunciado

Considerando a implementação do Perceptron abaixo:

```python showLineNumbers title=perceptron.py
class Perceptron:
    def __init__(self, input_size):
        self.weights = np.zeros(input_size + 1)  # Including bias

    def activation(self, x):
        return 1 if x >= 0 else 0

    def predict(self, x):
        x = np.insert(x, 0, 1)  # Add bias term
        weighted_sum = np.dot(self.weights, x)
        return self.activation(weighted_sum)

    def loss(self, y_true, y_pred):
        return y_true - y_pred

    def train(self, X, y, epochs=10, lr=0.1):
        for _ in range(epochs):
            for xi, target in zip(X, y):
                error = self.loss(target, self.predict(xi))
                self.weights += lr * error * np.insert(xi, 0, 1)
```

Modifique a implementação para que hajam 2 camadas escondidas e, assim, a rede
passar a conseguir representar uma porta lógica XOR.

## 3. Padrão de entrega

:::warning ATENÇÃO

Esses são os critérios mínimos para que eu considere a atividade como entregue.
Fique atento, pois o não cumprimento de qualquer um desses critérios pode, no
melhor dos casos, gerar um desconto de nota e, no pior deles, invalidar a
atividade.

:::

1. A atividade deve ser feita em um repositório **aberto** no Github. Seu link
deve ser fornecido no card da adalove;
2. No README do repositório deve ter instruções claras de como instalar e rodar o
sistema criado, comandos em blocos de código e uma expliação sucinta do que
fazem;
3. Ainda no README, deve haver um vídeo gravado demonstrando plenamente o
funcionamento do sistema criado;

## 4. Padrão de qualidade

<table>
  <tr>
    <th>Supera<br/>(9,0 - 10,0)</th>
    <th>Atende<br/>(6,5 - 9,0)</th>
    <th>Quase lá<br/>(4,0 - 6,5)</th>
    <th>Insuficiente<br/>(1,0 - 4,0)</th>
    <th>Desalinhado<br/>(0,0 - 1,0)</th>
  </tr>
  <tr>
    <td>Implementou a função de perda de entropia cruzada e a função de
    ativação sigmoide.</td>
    <td>O MLP consistentemente consegue ser treinado para representar uma porta
    lógica XOR.</td>
    <td>O MLP foi implementado corretamente, mas ainda não é capaz de
    representar uma porta lógica XOR consistentemente.</td>
    <td>Há erros na implementação das camadas escondidas do MLP.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>
