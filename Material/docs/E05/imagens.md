---
title: Imagens
sidebar_position: 1
sidebar_class_name: autoestudo
slug: /imagens
---

# Introdução à imagens

## 1. O que são imagens digitais e como são feitas

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/LWxu4rkZBLw"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>
<br />

:::

## 2. Como o computador armazena imagens

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/EXZWHumclx0"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>
<br />

:::

## 3. Estrutura de dados para armazenar imagens

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/kP0M1_740Mc"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>
<br />

:::

## 4. O openCV

:::warning Aviso

Esse conteúdo é um esboço inicial feito pelo chatGPT. Vou revisar mais tarde.

:::

### 4.1. O que é OpenCV?

OpenCV (Open Source Computer Vision Library) é uma biblioteca de programação
voltada para visão computacional e aprendizado de máquina. Desenvolvida
inicialmente pela Intel, ela é uma ferramenta poderosa e amplamente utilizada
em aplicações de visão computacional, como detecção de objetos, reconhecimento
de faces, processamento de imagem, entre outros. A biblioteca é escrita em C++
mas possui bindings para outras linguagens como Python, Java, e MATLAB, o que
facilita sua utilização em diferentes projetos.

### 4.2. Como funciona o OpenCV?

OpenCV fornece uma coleção de mais de 2500 algoritmos otimizados que podem ser
usados para diversas tarefas de visão computacional. Esses algoritmos permitem
desde o processamento básico de imagens (como filtragem, transformação, e
análise de contornos) até tarefas mais complexas (como segmentação de objetos,
reconhecimento de faces e gestos).

### 4.3. Instalando OpenCV

Para usar OpenCV com Python, primeiro precisamos instalar a biblioteca. Podemos
fazer isso facilmente utilizando o pip:

```bash
pip install opencv-python
pip install opencv-python-headless # Para máquinas sem GUI
```

### 4.4. Exemplo: Capturando e Exibindo Imagens da Webcam em Tempo Real

Vamos agora criar um exemplo prático utilizando a biblioteca `cv2` do OpenCV
para capturar imagens da webcam e exibi-las em uma janela em tempo real.

Primeiro, precisamos importar as bibliotecas necessárias:

```python
import cv2
```

Vamos acessar a webcam utilizando a função `cv2.VideoCapture()`. O parâmetro
`0` indica que estamos acessando a webcam padrão do sistema:

```python
cap = cv2.VideoCapture(0)
```

Vamos agora criar um loop que captura os quadros da webcam e os exibe em uma
janela. Para interromper o loop, podemos pressionar a tecla `q`:

```python
while True:
    # Captura o quadro
    ret, frame = cap.read()
    
    # Verifica se a captura foi bem-sucedida
    if not ret:
        break
    
    # Exibe o quadro em uma janela
    cv2.imshow('Webcam', frame)
    
    # Aguarda a tecla 'q' para sair
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera a captura e fecha as janelas
cap.release()
cv2.destroyAllWindows()
```

1. **Importação:** Importamos a biblioteca `cv2`.
2. **Acessar a Webcam:** Utilizamos `cv2.VideoCapture(0)` para acessar a webcam
   padrão.
3. **Loop de Captura:** Dentro do loop, lemos cada quadro da webcam usando
   `cap.read()`.
    - `ret`: Um booleano que indica se a captura foi bem-sucedida.
    - `frame`: O quadro capturado.
4. **Exibição:** Utilizamos `cv2.imshow('Webcam', frame)` para exibir cada
   quadro em uma janela chamada "Webcam".
5. **Espera por Tecla:** `cv2.waitKey(1)` aguarda por 1 milissegundo. Se a
   tecla `q` for pressionada, o loop é interrompido.
6. **Liberação de Recursos:** Após sair do loop, liberamos a captura da webcam
   (`cap.release()`) e fechamos todas as janelas abertas
   (`cv2.destroyAllWindows()`).
