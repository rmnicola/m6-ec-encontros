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
    def __init__(self, weights=None, bias=-1, activation_threshold=0.5):
        if weights == None:
            self.weights = np.array([1, 1])
        else:
            self.weights = np.array(weights)
        self.bias = bias
        self.activation_threshold = activation_threshold

    def _heaviside(self, x):
        """
        Implementa a função delta de heaviside (famoso degrau)
        Essa é uma função de ativação possível para os nós da rede neural.
        """
        return 1 if x >=  self.activation_threshold else 0

    def _sigmoid(self, x):
        """
        Implementa a função sigmoide
        Essa é uma função de ativação possível para os nós da rede neural.
        """
        return 1/(1 + math.exp(-x))

    def _activation(self, perceptron_output):
        """
        Implementação da função de ativação do perceptron
        Escolha uma das funções de ativação possíveis
        """
        return self._heaviside(perceptron_output)

    def forward_pass(self, data):
        """
        Implementa a etapa de inferência (feedforward) do perceptron.
        """
        weighted_sum = self.bias + np.dot(self.weights, data)
        return self._activation(weighted_sum)
```

Modifique a implementação para que seja possível treinar o perceptron e de modo
que ele tenha uma cama escondida. Para isso, deve-se:

1. Implementar uma função de custo para avaliar o resultado da predição
2. Implementar o processo de treinamento utilizando gradiente descendente e
   backpropagation
3. Modificar o perceptron para ter uma camada escondida 

Após essa implementação, valide o MLP criado treinando-o para funcionar como
uma porta XOR.

Para auxiliar na sua implementação, considere o artigo abaixo:

[How neural networks solve the XOR
problem](https://towardsdatascience.com/how-neural-networks-solve-the-xor-problem-59763136bdd7)

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
    <td>Além do MLP conseguir reproduzir consistentemente o comportamento da
    porta XOR, ainda há uma implementação extra do mesmo MLP utilizando o torch
    (pyTorch)</td>
    <td>O MLP consistentemente consegue ser treinado para representar uma porta
    lógica XOR.</td>
    <td>O MLP foi implementado corretamente, mas ainda não é capaz de
    representar uma porta lógica XOR consistentemente.</td>
    <td>Há erros na implementação das camadas escondidas do MLP.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>
