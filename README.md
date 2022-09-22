<h1>LibDrone</h1>

<p>Libdrone trata-se de uma biblioteca C# para comunicações com drones via protocolo MAVLINK, tratando-se de uma biblioteca aberta e estado inicial,
recomenda-se a qualquer um que tenha desejo em expandi-lá a contribuir com o projeto.</p>

<p>Atualmente, o projeto já possui capacidade para comunicações com drones via TCP,UDP e SERIAL, apresetando comandos testados para trocar para modos de voo,
conectar, armar e desarmar, voar para waypoint, alçar voo e pousar</p>

<p>Para uma maior segurança e garantir a eficácia do código, foi utilizado como base para os códigos de navegação e protocolos para MAVLINK o código fonte do controle
de missão MISSION PLANNER</p>

<p>Este projeto foi desenvolvido com o objetivo de ser um projeto de conclusão de curso para a faculdade de Sistemas de Informações IFES Cachoeiro de Itapemirim
do programador responsável</p>

<h3>Utilização</h3>
<p>Para utilização deste código, siga os seguintes passos:</p>
<ul>
<li>Instale o Visual Studio 2022 utilizando a seguinte configuração <a href="https://raw.githubusercontent.com/ArduPilot/MissionPlanner/master/vs2022.vsconfig">VS2022</li>
<li>Com o Visual Studio instalado, crie um programa .net framework e adicione as pastas desta biblioteca</li>
<li>Para chamar biblioteca invoque ela e crie um elemento da classe como mostrada no exemplo <a href="https://github.com/Yesod-star/LibDrone/blob/master/Program.cs">program.cs</a></li>
<li>Dica: Caso queira fazer um teste da biblioteca, recomendamos a utilização do <a href="https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html">simulador ardupilot</a></li>
</ul>
