---
title: MNIST
sidebar_position: 1
sidebar_class_name: ponderada
slug: /ponderada6
---

# Aplicação prática de Redes Convolucionais

**Atividade com prazo de entrega até 18/06**

## 1. Objetivo

Utilizar redes neurais convolucionais para tarefas de visão computacional.

## 2. Enunciado

A atividade ponderada desta sprint consiste em duas partes:

1. Modelo convolucional treinado para detectar algarismos numéricos escritos
   manualmente;
2. Backend que apresente duas rotas:
    - Uma rota que receba uma imagem de um algarismo e retorne o valor do
      algarismo. Não é necessário tratar os casos que nenhum foi encontrado,
      utilizar como referencia o modelo de aula;
    - Uma rota que exiba uma página HTML com um formulário para envio de
      uma imagem e exiba o valor do algarismo encontrado.

O processo de treinamento do modelo convolucional deve estar no repositório,
pode ser por arquivo Python ou notebook. Este modelo deve ser salvo em um
arquivo chamado pesos.h5. O backend deve ser feito em Flask.

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

### 4.1. Modelo (até 5,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(4,0 - 5,0)</th>
    <th>Atende<br/>(3,0 - 4,0)</th>
    <th>Quase lá<br/>(1,5 - 3,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,5)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Além de fazer um bom modelo e exportá-lo, fez uma versão de modelo
    linear para resolver o mesmo problema e comparou as duas em termos de tempo
    de treinamento, desempenho e tempo de inferência.</td>
    <td>Treinou um bom modelo e exportou ele em um arquivo compatível com a
    implementação do backend</td>
    <td>Treinou um modelo capaz de reconhecer dígitos com acurácia de pelo
    menos 95%</td>
    <td>Fez o treinamento da CNN mas ainda não está com bons resultados</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

### 4.2. Backend (at́é 4,0 pontos)

<table>
  <tr>
    <th>Supera<br/>(3,0 - 4,0)</th>
    <th>Atende<br/>(2,0 - 3,0)</th>
    <th>Quase lá<br/>(1,0 - 2,0)</th>
    <th>Insuficiente<br/>(0,5 - 1,0)</th>
    <th>Desalinhado<br/>(0,0 - 0,5)</th>
  </tr>
  <tr>
    <td>Existe uma API com as rotas especificadas e uma implementação distinta
    para o modelo linear e o modelo convolucional.</td>
    <td>Além de implementar uma API com as rotas especificadas para o modelo
    convolucional, há uma documentação explicando o que cada rota faz.</td>
    <td>Implementou uma API com as rotas especificadas, mas não há
    documentação</td>
    <td>Não implementou corretamente ou de forma completa as rotas
    especificadas no enunciado da atividade.</td>
    <td>Entrega fora de contexto.</td>
  </tr>
</table>

### 4.3. Interface (até 1,0 ponto)

Fez uma interface de usuário em que é possível enviar uma imagem de algarismo e
receber o resultado da predição? 1,0 ponto =D
