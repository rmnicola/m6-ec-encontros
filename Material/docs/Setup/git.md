---
title: Git
sidebar_position: 5
sidebar_class_name: opcional
slug: /git
---

# Configuração do Git

Git é um programa de versionamento de arquivos (funciona com todo o tipo de
arquivo, mas é mais poderoso com arquivos de texto) que ao mesmo tempo segue a
filosofia UNIX (tem apenas uma função, executada com maestria) e oferece
ferramentas de versionamento extremamente robustas. Por essas característica,
Git virou o padrão de versionamento de projeto de programação. Ser um
desenvolvedor e não saber usar o Git é a mesma coisa que ser um jogador de
futebol que não sabe calçar as chuteiras; pode até ser que consiga entrar em
campo, mas é provável que passe o jogo todo escorregando.

Sendo assim, vamos aprender a calçar as chuteiras?

<img 
  src="https://i.makeagif.com/media/6-28-2018/YYDpk9.gif"
  alt="Boot process" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

## 1. Instalando o git

Se você está usando o Linux, é bastante provável que já tenha o git instalado.
Caso não tenha, basta instalar utilizando o gerenciador de pacotes:

```bash
sudo apt install git
```

:::tip

A maioria das distribuições de Linux oferecem metapacotes que juntam todas as
ferramentas mais comuns para desenvolvimento de software, bastando um único
comando para instalá-las. No Ubuntu, esse metapacote se chama `base-devel`.

```bash
sudo apt install base-devel
```

:::

Agora que temos uma instalação do git, vamos configurá-lo corretamente?

## 2. Configurando o git

### 2.1. Adicionando seu nome e email

O git é uma ferramenta que pode ser usada para ~atribuir culpa~ registrar os
responsáveis por cada parte do código do projeto. Sendo assim, você precisa
registrar ao menos seu nome completo e email antes de conseguir fazer um
`commit`.

```bash
git config --global user.name "Seu Nome Aqui"
git config --global user.email "seu.email@aqui"
```

Sempre que você usar o comando `git commit`, o git vai usar um editor para que
você possa escrever sua mensagem de commit. Caso você não mexa nessa
configuração, o padrão do git será o vim. Se você nuna usou o vim na vida, a
experiência de dar um commit e ser jogado dentro do editor pode ser um pouco
traumática.

<img 
  src="https://preview.redd.it/b5bt8nv08qm61.jpg?width=640&crop=smart&auto=webp&s=38e18aba6ce0a20846a573a2ac033b54705f92c7"
  alt="Boot process" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '40vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Para configurar seu editor padrão do vim, use:

```bash
git config --global core.editor "code"
```

O comando acima configura como editor padrão o **vscode**.

## 2.2. Configurando o acesso aos repositórios remotoso

É **bastante** incomum o uso do git sem um serviço central de repositórios
remotos. Os principais que temos são o **github** e o **gitlab**. Para
conseguir sincronizar o acesso do seu git com esses dois serviços, você vai
precisar gerar e cadastrar uma chave SSH para **autenticar** o seu usuário.

Embora não seja nada
[difícil](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
fazer isso, eu acabei criando um script que automatiza o processo pois eu
sempre esquecia como fazer e tinha que ler esse tutorial novamente 😅

Para usar meu script, clone meu repositório de scripts (aqui tem vários scripts
úteis. Usem sem parcimônia ~e quebrem seu sistema operacional~).

```bash
git clone https://github.com/rmnicola/Scripts.git && cd Scripts
```

Como eu sou ~preguiçoso~ muito organizado, eu criei um script que basicamente
pega todos os scripts existentes nessa pasta de scripts e cria um link
simbólico para que eles fiquem disponíveis em `/usr/bin/`, que é a pasta onde
ficam os arquivos binários executáveis do seu usuário no Linux. Eu fiz isso
pois a partir do momento em que essa configuração é feita, todos os scripts
passam a ser acessíveis de qualquer lugar do sistema, como se fosse um programa
que você instalou (o motivo disso é porque ele **é** um programa que você
instalou).

```bash
sudo ./install.sh
```

Se o output do comando tiver essa carinha:

```bash
Symlink for ilovecandy already exists
Symlink for configure-bt-autosuspend already exists
Symlink for configure-git already exists
Symlink for install-charm-tools already exists
Symlink for install-fonts already exists
Symlink for install-go already exists
Symlink for install-logiops already exists
Symlink for install-node already exists
Symlink for install-rust already exists
Symlink for install-starship already exists
Symlink for install-zsh already exists
Symlink for link-configs already exists
Symlink for set-gpg-key already exists
Symlink for set-ssh-key already exists
Symlink for configure-flatpak already exists
Symlink for gnome-backup already exists
Symlink for gnome-restore already exists
Symlink for install-neovim already exists
Symlink for install-ros already exists
Symlink for ros-env already exists
```

Significa que deu tudo certo.

Agora, basta usar o `set-ssh-key` para criar a chave ssh e **já copiar ela para
o seu clipboard**.

```bash
set-ssh-key git
```

Se tudo deu certo, você agora tem uma chave SSH pública prontinha para dar
**CTRL-V** na interface do Github ou Gitlab. Basta acessar a página de
configuração por aqui:

[Github](https://github.com/settings/keys)
[Gitlab](https://gitlab.com/-/profile/keys)

Se tudo der certo, você consegue testar a configuração usando:

```bash
ssh git@github.com
```

ou

```bash
ssh git@gitlab.com
```

## 3. Tutorial Git

Para saber mais sobre git, leia [esse livro
gratuito](https://git-scm.com/book/en/v2)
