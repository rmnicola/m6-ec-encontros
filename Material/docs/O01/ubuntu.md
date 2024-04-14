---
title: Linux (Ubuntu)
sidebar_position: 1
sidebar_class_name: autoestudo
---

# Instalação do Ubuntu

Vou começar essa seção logo de cara endereçando uma dúvida muito comum:
**precisa** usar Linux para ser um desenvolvedor? **NÃO**. É perfeitamente
possível seguir sua carreira toda sem enconstar em um sistema baseado em Unix.
Na verdade, para algumas áreas é até melhor que não encoste (e.g. game dev,
stack .net, mobile). No entanto, também é verdade que Linux/Unix é o **padrão**
quando se fala de sistemas operacionais para **servidores**. 

Neste módulo em específico, vamos utilizar o **ROS**, que é um sistema feito
para conversar especificamente com o **Ubuntu**, especificamente a versão
**22.04**. 
* Tem como usar ROS em containers? Tem. 
* Tem como usar o WSL? Tem. 
* Tem como usar outra distro e compilar o ROS da fonte? Tem.
* Tem como instalar o visual studio e configurar o ROS para Windows? Não
  exagera, vai (mas tem). 

Tem como fazer tudo isso, mas seguir por qualquer um desses caminhos
alternativos vai minar a sua capacidade de encontrar soluções para os seus
problemas online, pois sempre vão assumir que você está no Ubuntu. Essa
documentação também vai assumir que você está no Ubuntu. Sendo assim, nossa
recomendação é que **instalem o Ubuntu em um cartão SD**.

Antes de começarmos a instalação, vou usar essa oportunidade para dar uma visão
super rasa de como se dá a relação do seu computador com o sistema operacional
no que diz respeito ao `boot`. A figura abaixo demonstra os passos que são
executados desde o momento em que você aperta o botão de ligar até a tela de
login:

<img 
  src="https://miro.medium.com/v2/resize:fit:720/format:webp/1*wjWc0sSBV1VWRr374WHm7g.jpeg"
  alt="Boot process" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Essa imagem apresenta o processo de `boot` sob um viés claro de sistemas Linux
(mais especificamente, distribuições que usam o `systemd`), mas para qualquer
sistema operacional a sequência de etapas é mais ou menos a mesma:

1. **Power-On Self-Test (POST):** Quando o computador é ligado, a primeira
   etapa é o POST. Neste momento, a BIOS (Basic Input/Output System - apenas em
   sistemas **legado**) ou o UEFI (Unified Extensible Firmware Interface - o
   padrão atual) - dependendo do tipo de firmware que o computador usa -
   verifica o hardware básico para garantir que tudo esteja funcionando
   corretamente. Isso inclui a memória RAM, o processador, o teclado, o disco
   rígido, e outros componentes essenciais. Se algum problema for detectado, o
   computador emite bipes ou códigos de erro.

2. **Detecção de Dispositivos de Armazenamento:** A BIOS/UEFI verifica os
   dispositivos de armazenamento disponíveis, como HDDs (Hard Disk Drives),
   SSDs (Solid-State Drives), drives ópticos, ou dispositivos USB, para
   encontrar um meio de boot. A ordem de prioridade desses dispositivos pode
   ser configurada nas configurações da BIOS/UEFI.

3. **Carregamento do Setor de Boot:** Uma vez que um dispositivo de
   armazenamento válido é encontrado, a BIOS/UEFI lê o primeiro setor desse
   dispositivo, conhecido como setor de boot ou MBR (Master Boot Record) em
   sistemas mais antigos, ou o EFI System Partition em sistemas mais modernos
   que usam UEFI. Esse setor contém um pequeno programa chamado carregador de
   boot (bootloader), que é necessário para carregar o sistema operacional.

4. **Execução do Carregador de Boot:** O carregador de boot inicializa e
   executa outro programa de boot mais específico, como o GRUB (GRand Unified
   Bootloader) para sistemas Linux ou o Windows Boot Manager para sistemas
   Windows. Esse estágio pode envolver a seleção de um sistema operacional em
   sistemas multiboot.

5. **Carregamento do Kernel do Sistema Operacional:** O carregador de boot
   carrega o kernel do sistema operacional na memória. O kernel é o núcleo do
   sistema operacional, responsável por gerenciar o hardware do computador e
   permitir que o software interaja com ele. Info interessante: Linux é
   **apenas** o kernel e nada mais.

6. **Inicialização do Sistema Operacional:** Com o kernel carregado, o sistema
   operacional começa a inicializar. Isso inclui a configuração de drivers de
   dispositivos, a inicialização de serviços do sistema, e a configuração de
   variáveis de ambiente. Neste ponto, o sistema operacional pode carregar uma
   interface gráfica de usuário (GUI), permitindo que o usuário interaja com o
   sistema.

7. **Logon do Usuário:** Finalmente, o sistema operacional exibe uma tela de
   logon (se configurado para fazê-lo), onde o usuário pode entrar com suas
   credenciais para acessar o ambiente de trabalho. Após o login bem-sucedido,
   o sistema está pronto para uso, com programas e arquivos do usuário
   disponíveis para acesso e execução.

Sendo assim, **instalar um sistema operacional** significa configurar algum dos
**dispositivos de armazenamento** do sistema para conter um **bootloader** em
uma partição **EFI** e um sistema operacional no restante do seu espaço.

A maneira mais conveniente de fazer isso é inserir um dispositivo de
armazenamento **removível** em que colocamos uma versão limitada de um sistema
operacional e, usando esse sistema operacional fazemos a **instalação** do
nosso sistema em um dispositivo de armazenamento **fixo** (e.g. HDD ou SSD).

Agora que temos um pouco mais de claridade com relação ao processo de
instalação de um sistema operacional, vamos criar um **pen drive** de boot?

## 1. Criando um pen drive de boot

Para poder seguir esse tutorial, você vai precisar de:
```markdown
* Um pen drive com pelo menos 2GB de espaço total (ele vai ser formatado!);
* Um cartão SD com pelo menos 16GB de espaço (sugerido 32GB ou mais); e
* O seu computador (precisa ter porta USB e SD, claro =D).
```

O site do Ubuntu fornece a imagem necessaria para criar esse pen drive de boot
[nesse
link](https://releases.ubuntu.com/22.04.3/ubuntu-22.04.3-desktop-amd64.iso). 
Essa imagem conta com uma versão portátil do Ubuntu com uma ferramenta de
instalação que é capaz de particionar e instalar o sistema operacional no
dispositivo de armazenamento de sua escolha. Sendo assim, ao ter esse pen drive
o processo de instalação torna-se apenas uma interface gráfica de fácil
navegação (assim como é com o Windows).

Legal, mas o que diabos eu faço com esse arquivo `iso`. Jogo direto no pen
drive? Não, você precisa de uma ferramenta para pegar essa imagem e gravá-la no
seu pen drive. A ferramenta que acho mais simples para isso chama-se `Balena
Etcher`. Você consegue baixar o instalador dessa ferramenta [nesse
link](https://github.com/balena-io/etcher/releases/download/v1.18.11/balenaEtcher-Setup-1.18.11.exe).

Siga o setup de instalação do Balena Etcher e execute a aplicação. O vídeo a seguir 
exibe o processo completo para gravar a imagem do Ubuntu em um pen drive. Esse processo 
resume-se em:
1. Encontrar o arquivo `iso` no diretório onde você baixou;
2. Escolher o pen drive na lista de drives do sistema; e 
3. Clicar em `flash`.

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/hjgOSuDbVoU"
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

## 2. Bootando no pen drive 

Vimos lá em cima que o processo de boot de um computador envolve uma etapa em
que o firmware da placa mãe busca por dispositivos de armazenamento que
contenham uma partição `EFI` com um bootloader configurado. Legal, nós temos
exatamente isso em nosso pen drive! 

Mas, pera aí; e se eu tiver **outro** dispositivo de armazenamento que atenda a
esse requisito? Nesse caso, a UEFI vai usar o **primeiro** que ela encontrar.
Como eu sei qual vai ser o primeiro dispositivo que ela vai encontrar? Através
da configuração de **prioridade de boot** da UEFI. Essa é uma lista que pode
ser mudada nas configurações da BIOS/UEFI (tipicamente você aperta `del` ou
`F2` no começo do processo de boot e o sistema te leva para essa tela de
configuração). Essa tela é um tanto perigosa, pois nela você pode mudar quase
todas as configurações do seu sistema, incluindo configurações que podem
**destruir o seu computador**.

Se você estiver utilizando um notebook ou um desktop com uma placa mãe mais
moderna, você está com sorte: existe uma alternativa mais segura e fácil de
navegar para **escolher** qual dispositivo será usado para boot naquele boot em
específico (a configuração não é persistente). Esse menu pode ser acessado
apertando uma outra tecla durante o processo de boot (para notebooks Dell é o
`F12`. Para outros fabricantes, consulte o Dr. Google). A imagem abaixo mostra
a variante desse menu para sistema Dell:

<img 
  src="https://i0.wp.com/accatech.com/wp-content/uploads/Boot-from-USB.jpg?w=800&ssl=1"
  alt="Boot selection screen" 
  style={{ 
    display: 'block',
    marginLeft: 'auto',
    maxHeight: '70vh',
    marginRight: 'auto'
  }} 
/>
<br/>

Aqui o que você precisa fazer é identificar qual é o dispositivo que
corresponde ao seu pen drive. Isso vai variar bastante de acordo com a
fabricante do seu pen drive, mas a maioria vai conter o nome da fabricante. No
caso da imagem acima, o pen drive é o dispositivo `UEFI: SanDisk SD6SP1...`.

Escolha o seu pen drive, entre no sistema operacional portátil e siga para a
próxima etapa.

## 3. Instalando o Ubuntu

:::danger
ATENÇÃO!! Quando estiver instalando o Ubuntu, as duas primeiras opções de 
instalador serão para criar um dual boot com Windows e apagar a partição Windows 
completamente. Não vamos usar **nenhuma dessas opções**. Vamos escolher a terceira 
opção, que é a instalação customizada.
:::

Se tudo deu certo, você vai ser levado direto para a tela de instalação do 
Ubuntu. A partir daí, siga as instruções abaixo:

<div style={{ textAlign: 'center' }}>
    <iframe 
        style={{
            display: 'block',
            margin: 'auto',
            width: '100%',
            height: '50vh',
        }}
        src="https://www.youtube.com/embed/SVK0ONyTnS8" 
        frameborder="0" 
        allowFullScreen>
    </iframe>
</div>

### 3.1. Particionando o cartão SD 

Ao criar as partições do cartão SD, sugiro a seguinte configuração mínima:

* Partição do tipo EFI com `500MB`. Essa partição serve para que o firmware da 
placa mãe consiga iniciar o processo de boot do sistema operacional. O bootloader 
utilizado vai ser instalado aqui (no caso do Ubuntu, GRUB).
* Partição do tipo EXT4 com o restante do tamanho do cartão SD com mountpoint 
em `/`. Essa partição é a raíz do seu sistema operacional.

Considerações sobre SWAP e partição montada em `/home`:

* A partição de SWAP serve para armazenar o que está na memória RAM no momento 
em que o sistema entra em modo de hibernação. Como o cartão SD que estamos 
usando tem 64GB e os notebooks comumente tem 16GB de RAM hoje em dia, uma partição 
de SWAP fica pouco viável.

* A partição montada em `/home` normalmente é utilizada por questões de segurança 
de dados. É mais fácil configurar ferramentas de backup se você tiver essa separação.
Diferente da raíz do sistema (`/`), o diretório casa (`/home`) guarda apenas arquivos
relacionados aos usuários do sistema.
