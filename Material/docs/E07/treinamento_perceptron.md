---
title: Treinamento Perceptrons
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /treinamento_perceptron
---

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

## Implementação de perceptrons

Pessoal até aqui temos uma implementação dos nossos perceptrons que é capaz de receber `nEntradas`, com `nPesos` e um `threshold` e retornar a saída do perceptron. Contundo existem outras funções de ativação que podem ser utilizadas, como a função de ativação degrau, a função de ativação sigmóide, a função de ativação tangente hiperbólica, entre outras.

Vamos modificar um pouco nossa classe para deixar essa função de ativação mais genérica. Para isso vamos criar um método `funcaoAtivacao` que recebe o comportamento que deseja para a função de ativação. 

```python showLineNumbers
import math

class Perceptron:
    def __init__(self, weights, activation_function):
        self.weights = weights
        self.activation_function = activation_function

    def predict(self, inputs):
        # Calcula a soma ponderada das entradas
        total = sum(w * i for w, i in zip(self.weights, inputs))
        # Aplica a função degrau para determinar a saída
        return activation_function(total) 

# Função de ativação degrau
def degrau(x):
    return 1 if x > 0 else 0

# Função de ativação sigmóide
def sigmoide(x):
    return 1 / (1 + math.exp(-x))

# Função de ativação tangente hiperbólica
def tanh(x):
    return math.tanh(x)

# Função de ativação ReLU
def relu(x):
    return max(0, x)

# Agora vamos utilizar nosso perceptron com a função de ativação sigmóide
if __name__ == "__main__":
    weights = [0.5, 0.5, 0.5]
    activation_function = sigmoide
    perceptron = Perceptron(weights, activation_function)
    inputs = [1, 1, 1]
    print(perceptron.predict(inputs))
```

Troque a função de ativação para `degrau`, `sigmoide`, `tanh` ou `relu` e veja como a saída do perceptron muda.

:::tip[Diferentes Funções de Ativação]

Pessoal aqui temos algumas diferentes funções de ativação:

<iframe width="560" height="315" src="https://www.youtube.com/embed/Fu273ovPBmQ?si=3c71H2v3Ll1h4C_y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

:::

## Treinamento de perceptrons

Para o treinamento de perceptrons, utilizamos um algoritmo chamado de regra de aprendizado do perceptron, ou regra de Hebb. A regra de aprendizado do perceptron é um algoritmo de aprendizado supervisionado que atualiza os pesos do perceptron para minimizar o erro entre a saída prevista e a saída real.

A regra de aprendizado do perceptron é baseada na regra de Hebb, que afirma que "células que disparam juntas, se conectam". Em outras palavras, a regra de Hebb sugere que os pesos de uma conexão sináptica devem ser aumentados sempre que a entrada e a saída correspondentes forem ativadas simultaneamente.

A regra de aprendizado do perceptron é uma generalização da regra de Hebb que leva em consideração o erro entre a saída prevista e a saída real. A regra de aprendizado do perceptron é dada por:

$$
w_{i} = w_{i} + \alpha \times (y - \hat{y}) \times x_{i}
$$

onde:

- $w_{i}$ é o peso da i-ésima entrada,
- $\alpha$ é a taxa de aprendizado,
- $y$ é a saída real,
- $\hat{y}$ é a saída prevista,
- $x_{i}$ é a i-ésima entrada.

:::warning[Taxa de Aprendizado e Pedido de Desculpa]

Pessoal a taxa de aprendizado é um hiperparâmetro que controla a rapidez com que o modelo é ajustado. Uma taxa de aprendizado muito alta pode fazer com que o modelo oscile em torno do mínimo global, enquanto uma taxa de aprendizado muito baixa pode fazer com que o modelo leve muito tempo para convergir.

Em geral, o valor da taxa de aprendizado é escolhido empiricamente e depende do problema específico que está sendo resolvido. Ele costuma ser um valor maior que zero e menor que um.

E aqui, um pedido de desculpas. Na nossa interação eu esqueci de levar em consideração a entrada no treinamento! Vamos corrigir isso agora.

<iframe width="560" height="315" src="https://www.youtube.com/embed/0QczhVg5HaI?si=ah3Z-8SbGLCMigA4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

:::

Agora vamos portar esse conceito para o nosso código. Vamos criar um método `fit` que recebe as entradas e saídas esperadas e atualiza os pesos do perceptron.

```python showLineNumbers
import math

class Perceptron:
    def __init__(self, weights, activation_function, learning_rate):
        self.weights = weights
        self.activation_function = activation_function
        self.learning_rate = learning_rate

    def predict(self, inputs):
        # Calcula a soma ponderada das entradas
        total = sum(w * i for w, i in zip(self.weights, inputs))
        # Aplica a função degrau para determinar a saída
        return activation_function(total) 

    def fit(self, inputs, y):
        y_hat = []
        # Faz as predições para cada entrada
        for i in range(len(inputs)):
            y_hat.append(self.predict(inputs[i]))
            
        # Atualiza os pesos
        for i in range(len(inputs)):
            self.update_weights(inputs[i], y[i], y_hat[i])

    def update_weights(self, inputs, y, y_hat):
        for i in range(len(self.weights)):
            self.weights[i] += self.learning_rate * (y - y_hat) * inputs[i]

# Função de ativação degrau
def relu(x):
    return max(0, x)

# rotina principal
if __name__ == "__main__":
    weights = [0.8, 0.2]
    activation_function = relu
    learning_rate = 0.1
    perceptron = Perceptron(weights, activation_function, learning_rate)
    inputs = [[1, 1], [1, 0], [0, 1], [0, 0]]
    y = [1,1,1,0]
    perceptron.fit(inputs, y)
    print(perceptron.weights)
```

Aqui pessoal temos um exemplo de treinamento de um perceptron para a função lógica OR. O perceptron tem duas entradas e uma saída. As entradas são representadas por uma lista de listas, onde cada lista interna representa uma entrada. A saída esperada é representada por uma lista de valores.

Quando rodamos o código, o perceptron é treinado para a função lógica OR e os pesos são atualizados de acordo com a regra de aprendizado do perceptron. Os pesos finais são impressos na tela. Contudo, apenas um treinamento não é suficiente para que o perceptron aprenda a função OR. Como poderiamos melhorar nosso código para que o perceptron aprenda a função OR?

```python showLineNumbers
# rotina principal
if __name__ == "__main__":
    weights = [0.8, 0.2]
    activation_function = relu
    learning_rate = 0.1
    perceptron = Perceptron(weights, activation_function, learning_rate)
    inputs = [[1, 1], [1, 0], [0, 1], [0, 0]]
    y = [1,1,1,0]
    for i in range(100):
        perceptron.fit(inputs, y)
    print(perceptron.weights)
```

Percebam que agora estamos treinando o perceptron 100 vezes. Isso faz com que a saída do perceptron seja mais próxima da saída esperada. Como vocês poderiam implementar um critério de parada para o treinamento?

```python showLineNumbers
# rotina principal
if __name__ == "__main__":
    weights = [0.8, 0.2]
    activation_function = relu
    learning_rate = 0.1
    perceptron = Perceptron(weights, activation_function, learning_rate)
    inputs = [[1, 1], [1, 0], [0, 1], [0, 0]]
    y = [1,1,1,0]
    for i in range(100):
        print(f"Iteração(Epoch) {i}: Pesos {perceptron.weights}")
        perceptron.fit(inputs, y)
        if perceptron.predict([1, 1]) == 1 and perceptron.predict([1, 0]) == 1 and perceptron.predict([0, 1]) == 1 and perceptron.predict([0, 0]) == 0:
            break
    print(perceptron.weights)
```

Façam modificações e testem o código!!