---
title: Deep learning
sidebar_position: 2
sidebar_class_name: autoestudo
slug: /dl
---

# Introdução ao deep learning

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/aircAruvnKk" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

:::

Redes neurais representam uma abordagem revolucionária na
programação, diferindo dos métodos convencionais ao permitir que o computador
aprenda a resolver problemas por meio de dados observacionais, ao invés de
seguir instruções precisamente definidas. Antes de 2006, o treinamento eficaz
de redes neurais era limitado a problemas especializados, mas com o advento do
deep learning, houve um avanço significativo. Atualmente, redes neurais
profundas são utilizadas com sucesso em áreas como visão computacional e
reconhecimento de fala, superando muitas abordagens tradicionais

## 1. Como as redes neurais aprendem?

:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/IHZwWFHWa-w" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

:::

As redes neurais aprendem ajustando seus parâmetros, como pesos e viéses, para
minimizar uma função de custo, um processo frequentemente realizado através do
algoritmo de gradiente descendente. Este algoritmo funciona alterando
iterativamente os parâmetros da rede na direção que reduz o custo. Na prática,
o gradiente descendente prova ser uma maneira eficaz de encontrar o mínimo
local da função de custo, ajudando assim a rede a aprender a partir dos dados.
Embora não seja garantido encontrar sempre o mínimo global, na maioria dos
casos, o gradiente descendente é uma estratégia ótima para a busca de um
mínimo, especialmente em redes neurais complexas. O processo envolve o cálculo
do gradiente da função de custo e a realização de ajustes nos parâmetros na
direção oposta ao gradiente, o que, em teoria, leva à redução do custo.

## 2. O que é retropropagação?


:::warning Autoestudo

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/Ilg3gGewQ5U" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

:::

Backpropagation é um algoritmo fundamental no aprendizado de redes neurais,
responsável por calcular os gradientes necessários para ajustar os pesos e
viéses da rede durante o treinamento. Introduzido originalmente nos anos 1970,
sua importância foi amplamente reconhecida após um famoso artigo de 1986 de
David Rumelhart, Geoffrey Hinton e Ronald Williams. O algoritmo de
backpropagation permite que as redes neurais aprendam de maneira mais eficiente
e rápida do que métodos anteriores, tornando possível resolver problemas
complexos que antes eram considerados insolúveis. Atualmente, backpropagation é
o principal mecanismo de aprendizado nas redes neurais, proporcionando uma
maneira robusta e eficaz de otimizar seus parâmetros internos para melhor
desempenho em tarefas de classificação, reconhecimento e outras aplicações de
inteligência artificial
