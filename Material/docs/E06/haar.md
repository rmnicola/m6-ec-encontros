---
title: Haar cascade
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /haar
---

# Algoritmo Haar Cascade para detecção de objetos

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/kThRJyQCW-8" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

:::

:::warning Aviso

Este texto foi gerado pelo chatGPT e é um esboço que será atualizado em breve.

:::

## 1. Introdução ao Haar Cascade

O algoritmo Haar Cascade é uma técnica poderosa e eficiente para a detecção de
objetos em imagens, como rostos, carros, e outros objetos comuns. Desenvolvido
por Paul Viola e Michael Jones, este método é conhecido por sua rapidez e
precisão. Vamos explorar como ele funciona de uma maneira simples e acessível.

O Haar Cascade utiliza uma série de características simples, chamadas de
"Haar-like features", que são calculadas de forma rápida usando uma imagem
integral. Ele se baseia em três componentes principais:
- **Imagem Integral**: Uma representação da imagem que permite cálculos
  rápidos.
- **Seleção de Características**: Usa o algoritmo AdaBoost para selecionar as
  características mais importantes.
- **Cascade de Classificadores**: Uma série de classificadores que processam a
  imagem em etapas, rejeitando rapidamente regiões que não são de interesse.

### 1.2. Imagem Integral

A imagem integral é uma representação intermediária que facilita o cálculo
rápido das características. Ela é construída de forma que cada ponto na imagem
integral contém a soma de todos os pixels acima e à esquerda daquele ponto na
imagem original.

**Fórmula da Imagem Integral:**

$$
ii(x, y) = \sum_{x' \leq x, y' \leq y} i(x', y')
$$

Onde:

* $$ii(x, y)$$ é a imagem integral no ponto $$(x, y)$$; e

* $$i(x, y)$$ é a imagem original.

### 1.3. Características Haar-Like

As características Haar-like são retângulos simples que calculam diferenças de
intensidades entre regiões claras e escuras na imagem. Existem três tipos
principais:
- **Características de dois retângulos**: Calculam a diferença entre duas áreas
  adjacentes.
- **Características de três retângulos**: Calculam a diferença entre a soma de
  duas áreas externas e uma área central.
- **Características de quatro retângulos**: Calculam a diferença entre pares
  diagonais de retângulos.

Essas características ajudam a identificar bordas, linhas e outras estruturas
simples na imagem.

### 1.4. Algoritmo AdaBoost

Para selecionar as características mais importantes, o Haar Cascade utiliza o
algoritmo AdaBoost, que é um método de aprendizado de máquina. AdaBoost combina
várias classificações fracas (classificadores simples) para criar um forte. No
contexto do Haar Cascade, cada classificador fraco usa uma única característica
Haar-like.

**Processo do AdaBoost:**
1. Inicializa pesos iguais para todas as amostras de treinamento.
2. Treina um classificador simples com base em uma única característica.
3. Ajusta os pesos das amostras, aumentando o peso das amostras incorretamente
   classificadas.
4. Repete o processo para selecionar mais características, combinando-as para
   formar um classificador forte.

### 1.5. Cascade de Classificadores

O Haar Cascade usa uma estrutura em cascata para melhorar a eficiência. Esta
estrutura consiste em vários estágios de classificadores, onde cada estágio
rejeita rapidamente regiões da imagem que não são de interesse.

**Funcionamento da Cascata:**
- **Primeiro Estágio**: Um classificador simples que rejeita a maioria das
  regiões não relevantes.
- **Estágios Subsequentes**: Classificadores progressivamente mais complexos
  que processam apenas as regiões que passaram pelos estágios anteriores.

Se uma região é rejeitada em qualquer estágio, ela não é processada nos
estágios seguintes, o que acelera significativamente a detecção.

## 2. Exemplo em Python com OpenCV

A biblioteca OpenCV fornece uma implementação do Haar Cascade que é fácil de
usar. Aqui está um exemplo de código para detecção de rostos:

```python
import cv2

# Carregar o classificador pré-treinado
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Ler a imagem
img = cv2.imread('face.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detectar rostos
faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

# Desenhar retângulos em torno dos rostos detectados
for (x, y, w, h) in faces:
    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

# Mostrar a imagem resultante
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

Neste exemplo, carregamos uma imagem, convertendo-a para escala de cinza e
usando o classificador Haar Cascade para detectar rostos. Depois, desenhamos
retângulos em torno dos rostos detectados.

