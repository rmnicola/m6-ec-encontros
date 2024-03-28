# Módulo de Robótica móvel e visão computacional

Este é o repositório que abriga o material da área de computação do módulo 6 do
curso de Engenharia de Computação do Inteli, cujo tema é Robótica móvel e visão
computacional. O objetivo do módulo é introduzir os conceitos básicos
envolvidos no uso de robôs móveis e também fazer uma incursão no mundo do
aprendizado profundo, em específico para identificação de padrões em imagens e
vídeos.

Os principais temas envolvidos nesse módulo são:
1. Fundamentos de robótica móvel;
2. Sistemas de controle;
3. Utilização do ROS2 como framework de troca de mensagens;
4. Fundamentos da visão computacional; e
5. Redes neurais convolucionais.

Este material foi feito utilizando o [docusaurus](https://docusaurus.io/), com
a página gerada hospedada no Github pages. Para acessar o material renderizado,
acesse [esse link](https://rmnicola.github.io/m6-ec-encontros/).

## Estrutura de pastas

Este repositório é relativamente simples e conta apenas com dois diretórios:
`Exemplos` e `Material`. O diretório `Material` é a pasta raíz do docusaurus,
enquanto o diretório `Exemplos` conta com o código fonte dos exemplos
desenvolvidos no decorrer do curso para demonstrar as tecnologias e conceitos
aplicados no módulo.

## Instalando e rodando a documentação localmente

> [!IMPORTANT]  
> As instruções a seguir assumem que você possuí `npm >= 16` instalado e
> devidamente configurado em seu sistema. A princípio, as instruções são
> agnósticas a sistema operacional.

Para instalar e rodar localmente uma documentação feita com o docusaurus, basta
seguir as instruções delineadas pela [documentação do
docusaurus](https://docusaurus.io/docs/installation#running-the-development-server),
o que consiste em:

1. Para rodar o projeto localmente utilizando o `npm`, navegue até a pasta raíz
   do docusaurus e rode:

```bash
npm run start
```

Esta forma de execução é primariamente utilizada para desenvolvimento de
conteúdo, pois ela conta com atualizações instantâneas à pagina local sempre
que os arquivos de conteúdo forem alterados.

2. Para criar artefatos de distribuição do site estático, rode:

```bash
npm run build
```

Esta forma deve ser utilizada somente para o deploy da página estática
(atualmente é utilizada no workflow do github pages)

## Considerações de uso

Ver [descrição da licença](./LICENSE).

## Como contribuir

Quer ajudar a construir e/ou melhorar o material? Fico feliz s2! Para que não
haja atrito na adição de suas contribuições, considere as seguintes instruções:

1. Antes de mais nada, abra um issue. Notou um erro de português, algum
   tutorial não funciona, alguma abstração está difícil de entender? Expresse o
   problema de forma clara em um issue nesse repositório. Essa é a forma mais
   rápida e de menor atrito para iniciar uma discussão sobre o problema em
   questão.

2. Issue aberto, mas você quer resolver a parada sozinho? Deus te abençõe. Para
   ter sua contribuição oficialmente como parte desse projeto, você deve:

   * Criar um fork deste repositório
   * Crie um branch para suas edições
   * Referencie issues em seus commits
   * Abra um PR para este repositório que fecha a issue em questão
   * Após passar em todos os testes e passar por uma revisão, o seu PR será
     integrado ao material. Muito obrigado! =)
