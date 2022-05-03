## ROS 용어
* Node : 최소단위의 실행 가능한 프로세서 
* Package : 하나 이상의 Node, Node 실행을 위한 정보등을 묶어놓은 것 
* Message : 메시지를 통해 Node간의 데이터를 주고 받게된다. 
* Master : Node와 Node 사이의 연결과 메시지 통신을 위한 네임 서버와 같은 역할을 한다. roscore가 실행 명령어이며, 마스터를 실행하면 각 노드의 이름을 동록하고 필요에 따라 정보를 받을 수 있다. Master 없이는 노드간의 접속, Topic과 Service와 같은 메시지 통신을 할 수 없다. 

## Message 통신 방식 
1. Topic, Publisher, Subscriber
> * <img src="./img/ROS008.png" width="700"/> 
> * Publisher Node가 하나의 이야깃거리에 대해서 Topic으로 Master에 등록한 후, 이야깃거리에 대한 이야기를 메시지 형태로 퍼블리시한다.
> * 이야깃거리를 수신받기를 원하는 Subscriber Node는 Master에 등록된 Topic의 이름에 해당하는 Publisher Node의 정보를 받는다.
> * 이 정보를 기반으로 Subscriber Node는 Publisher Node와 직접 연결하여 메시지를 Topic으로 송수신하게 된다.  
> * Topic 통신 방식은 비동기 방식이라 필요에 따라서 주어진 데이터를 전송하고 받기에 좋은 방법이다. 
> * 한 번의 접속으로 지속적인 메시지를 송수신하기 때문에 지속해서 메시지를 발송해야 하는 센서 데이터에 적합하여 많이 사용된다. 
2. Service, Service Server, Service Client
> * <img src="./img/ROS009.png" width="700"/> 
> * 특정 목적의 작업에 해당하는 서비스를 요청하는 Service Client와 서비스 응답을 담당하는 Service Server 간의 동기적 양방향 서비스 메시지 통신을 말한다. 
> * 한 번의 접속으로 지속적 통신이 아닌 서비스의 요청과 응답이 완료되면 연결된 두 노드의 접속은 끊긴다. 
3. Action, Action Server, Action Client
> * <img src="./img/ROS010.png" width="700"/> 
> * 서비스처럼 양방향을 요구하나 요청 처리 후 응답까지 오랜 시간이 걸리고 중간 결괏값이 필요한 경우에 사용되는 메시지 통신 방식이다. 
> * 서비스와 통신방식이 비슷하지만 중간 결괏값에 해당하는 feedback이 추가되었다.
