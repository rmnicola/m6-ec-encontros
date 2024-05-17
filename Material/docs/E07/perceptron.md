---
title: Perceptrons
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /perceptron
---

import Admonition from '@theme/Admonition';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Perceptrons

O conteúdo dessa seção foi em grande parte retirado e traduzido do seguinte
livro gratuito:

:::warning Autoestudo

[Neural networks and deep
learning](https://static.latexstudio.net/article/2018/0912/neuralnetworksanddeeplearning.pdf)

:::

Os perceptrons são um conceito fundamental no campo da inteligência artificial,
representando um dos primeiros modelos de neurônios artificiais. Desenvolvidos
por Frank Rosenblatt nas décadas de 1950 e 1960, os perceptrons foram
inspirados nos trabalhos anteriores de Warren McCulloch e Walter Pitts. Eles
servem como a base para a compreensão de estruturas mais complexas em redes
neurais modernas.

## 1. Funcionamento do perceptron

<img 
  src="img/perceptron.png"
  alt="Representação de perceptron" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '90vh',
    marginRight: 'auto'
  }} 
/>
<br/>

O perceptron opera tomando várias entradas binárias e produzindo uma única
saída binária. Matematicamente, a operação de um perceptron pode ser descrita
pela seguinte função:

$$
\text{output} = 
\begin{cases} 
0 & \text{se } \sum_{j} w_j x_j \leq \text{threshold} \\
1 & \text{se } \sum_{j} w_j x_j > \text{threshold}
\end{cases}
$$

Nesta equação, $$x_1, x_2, \ldots, x_n$$ representam as entradas binárias do
perceptron. Os $$w_1, w_2, \ldots, w_n$$ são os pesos atribuídos a cada
entrada, indicando a importância relativa de cada uma delas para a saída do
perceptron.

## 2. Simplificação e Bias

Para simplificar a notação, a soma ponderada das entradas é frequentemente
expressa como um produto escalar entre dois vetores, $$\mathbf{w} \cdot
\mathbf{x}$$, onde $$\mathbf{w}$$ e $$\mathbf{x}$$ são vetores de pesos e
entradas, respectivamente. Além disso, o limiar (threshold) é substituído por
um "bias" (b), redefinindo a regra do perceptron como:

$$
\text{output} = 
\begin{cases} 
0 & \text{se } \mathbf{w} \cdot \mathbf{x} + b \leq 0 \\
1 & \text{se } \mathbf{w} \cdot \mathbf{x} + b > 0
\end{cases}
$$

O bias é uma medida de quão fácil é para o perceptron ativar uma saída de 1,
facilitando ou dificultando sua ativação com base no valor do bias.

## 3. Aplicações e Importância

<img 
  src="img/mlp.png"
  alt="Perceptron" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '90vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Apesar de sua simplicidade, os perceptrons são capazes de realizar decisões
complexas ao serem agrupados em redes. Cada perceptron em uma camada de uma
rede neural pode tomar decisões simples baseadas nas entradas, enquanto os
perceptrons nas camadas subsequentes podem usar essas decisões para realizar
inferências mais complexas e abstratas.

Os perceptrons não são apenas um modelo histórico, mas também um conceito
fundamental para entender redes neurais mais complexas, como as redes neurais
convolucionais e as redes neurais recorrentes, amplamente utilizadas em
aplicações de aprendizado de máquina modernas.

## 4. Perceptron com portas NAND

Perceptrons podem ser usados para implementar várias funções lógicas, incluindo
a porta NAND, que é fundamental na computação. Para entender como um perceptron
pode representar uma porta NAND, consideremos um perceptron com duas entradas e
pesos específicos.

Suponhamos que temos um perceptron com duas entradas $$x_1$$ e $$x_2$$, cada
uma com um peso de $$-2$$, e um viés de $$+3$$. A função do perceptron pode ser
expressa como:

$$
\text{output} = 
\begin{cases} 
0 & \text{se } (-2) \times x_1 + (-2) \times x_2 + 3 \leq 0 \\
1 & \text{se } (-2) \times x_1 + (-2) \times x_2 + 3 > 0
\end{cases}
$$

Para diferentes combinações de entradas, o perceptron produzirá saídas que
correspondem à função lógica NAND:

- Quando ambas as entradas são 0 ($$x_1 = 0, x_2 = 0$$), o resultado da soma
  ponderada é $$3$$ (positivo), então a saída é $$1$$.
- Para as entradas $$01$$ e $$10$$, o resultado é novamente positivo, então a
  saída é $$1$$.
- Quando ambas as entradas são 1 ($$x_1 = 1, x_2 = 1$$), o resultado da soma
  ponderada é $$-1$$ (negativo), e a saída é $$0$$.

Portanto, este perceptron específico funciona como uma porta NAND. Isto é
significativo porque a porta NAND é universal para computação, o que significa
que qualquer função computacional pode ser construída a partir de portas NAND.
Consequentemente, redes de perceptrons têm a capacidade de computar qualquer
função lógica, o que destaca a versatilidade e a importância dos perceptrons na
computação e no aprendizado de máquina.

## 5. Perceptron em Python

O exemplo a seguir implementa um perceptron capaz de reproduzir o exemplo
encontrado no livro do autoestudo, em que a saída é a decisão de ir ou não a um
festival. No exemplo, o perceptron tem 3 entradas. Elas representam: 

1. Se o tempo está bom
2. Se o/a namorado/a que ir junto
3. Se o festival está perto de transporte publico

A saída é se a decisão é de ir ou não ao festival. O código que implementa pode
ser visto a seguir: 

```python showLineNumbers
class Perceptron:
    def __init__(self, weights, threshold):
        self.weights = weights
        self.threshold = threshold

    def predict(self, inputs):
        # Calcula a soma ponderada das entradas
        total = sum(w * i for w, i in zip(self.weights, inputs))
        # Aplica a função degrau para determinar a saída
        return 1 if total > self.threshold else 0

# Exemplo de uso
if __name__ == "__main__":
    # Pesos para: tempo, companhia do namorado/namorada, proximidade do transporte público
    weights = [6, 2, 2]  # O peso do tempo é maior, indicando maior importância
    threshold = 5  # Limiar para a decisão

    # Cria o perceptron com os pesos e limiar pré-definidos
    perceptron = Perceptron(weights, threshold)

    # Testa o perceptron com diferentes entradas
    print(perceptron.predict([1, 0, 0]))  # Bom tempo, sem companhia, longe do transporte
    print(perceptron.predict([0, 1, 1]))  # Tempo ruim, com companhia, perto do transporte
    print(perceptron.predict([1, 1, 1]))  # Bom tempo, com companhia, perto do transporte
```
