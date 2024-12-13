# Hi6 컨트롤러 Job 코드

이 job코든느 로봇을 제어하고 현재 축 위치 값을 수신하기 위해 UDP통신을 설정하도록 설계되었습니다.

## 개요

이 Job코든느 로봇을 제어하고, 현재 축 위치 값을 수신하기 위해 UDP통신과 온라인 궤적 버퍼를 설정하도록 설계되었습니다. 추가된 코드에서는 특정 메시지`("stop")`를 수신하면 궤적 버퍼를 초기화하는 로직이 포함되어 있습니다.

## 코드 설명

```text
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
```

Job 파일의 포맷과 버전을 지정합니다.

### 네트워크 설정

```python
import enet
global enet0
enet0=enet.ENet()
enet0.ip_addr="192.168.1.2" #PC IP
enet0.lport=7000
enet0.rport=7000
enet0.open
```

- `enet` 모듈을 네트워크 통신용으로 가져옵니다.
- 글로벌 `enet`객체인 `enet0`을 생성합니다.
- IP주소를 설정합니다.
- 로컬포트`lport`와 원격포트 `rport`를 각각 7000으로 설정합니다.
- 네트워크 연결을 엽니다.

### 온라인 궤적 설정

```python
  global onl
  var msg
  onl=online.Traject()
  onl.time_from_start=-1.0 # disable
  onl.look_ahead_time=1.0 #The time it takes to execute after a value is entered into the online.Traject buffer.
  onl.interval=0.1 #Online.Traject Movement time from pose accumulated in buffer to next pose
  onl.init # online trajectory operation init (buffer init)
```
#### Configuration Steps

- 글로벌 `online.Traject` 객체인 `onl`을 생성합니다.
- 온라인 궤적 매개변수를 다음과 같이 설정합니다.:
  - `time_from_start` : -1.0으로 설정하여 비활성화
  - `look_ahead_time` : 0.1초로 설정(버퍼에 입력된 값이 실행되기까지 걸리는 시간)
  - `interval` : 0.1초로 설정(버퍼에 쌓인 자세에서 다음 자세로 이동하는 시간)
- 궤적 버퍼를 초기화 합니다.

**참고** 상제 정보는 아래 문서를 참고 하십시오.

[https://hrbook-hrc.web.app/#/view/doc-hrscript/korean/5-moving-robot/14-online.Traject](https://hrbook-hrc.web.app/#/view/doc-hrscript/korean/5-moving-robot/14-online.Traject)

### 멀티태스크 기능 설정

``` python
  task start,sub=1,job=7001
```
- task문은 멀티태스크 기능을 수행하는 프로시져
- job파일 7001을 실행 시킵니다.

### Main Loop

```python
10  enet0.recv
    msg=result()
    print msg
    if msg=="stop"
      onl.init # online trajectory operation init (buffer init)
      goto 10
    else
      onl.buf_in msg #Put the pose in the ‘po’ variable into the online.Traject buffer.
    endif
    goto 10
    end
```

- 변수 `msg`를 선언합니다.
- 루프 시작(레이블 `10`):
  1. UDP를 통해 메시지를 수신합니다.
  2. 수신된 메시지를 `msg`변수에 저장합니다.
  3. 메시지를 출력합니다.
  4. 메시지가 `"stop"`인 경우:
      - 궤적 버퍼 초기화: `onl.init`명령을 통해 궤적 버퍼를 초기화합니다.
      - 초기화 후 루프의 시작으로 돌아갑니다,`(goto 10)`.
  5. 메시지가 `"stop"`이 아닌 경우:
      - 메시지 추가 : 수신된 메시지를 온라인 궤적 버퍼에 추가합니다.`(onl.buf_in_msg)`.
  6. 루프의 시작으로 돌아갑니다 `(goto 10)`.


### Sub Loop

```python
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
     import enet
     var pos
  10 pos=cpo("joint")
     enet0.send pos.str_array()
     delay 0.01
     goto 10
     end
```
- 네트워크 통신 설정:
  - `enet`모듈을 사용하여 데이터를 송신합니다.
- 루프동작(레이블 `10`):
  1. `pos`변수에 현재 로봇 관리 위치를 저장합니다.
      - `cpo("joint")를 사용하여 현재 관절 상태(joint state)를 가져옵니다.
  2. 관절 위치 데이터를 문자열 배열 형식으로 변환한 뒤 `(pos.str_array())`, 이를 네트워크를 통해 송신합니다.
      - `enet0.send`를 사용하여 송신.
  3. 0.01초 지연을 적용합니다. `(delay 0.01)`
  4. 루프의 시작으로 돌아가서 계속 실행됩니다. `(goto 10)`

## 주요 포인트

- UDP를 사용하여 빠르고 가벼운 데이터 전송을 실현합니다.
- 온라인 궤적 버퍼를 사용하여 받은 자세를 보간(interpolation)하며 부드럽고 연속적인 움직임을 제공합니다.
- 지속저인 루프를 통해 로봇의 실시간 제어 및 위치 피드백이 가능합니다.

## code

```python
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
     import enet
     global enet0
     enet0=enet.ENet()
     enet0.ip_addr="192.168.1.2" #PC IP
     enet0.lport=7000
     enet0.rport=7001
     enet0.open
     global onl
     var msg
     onl=online.Traject()
     onl.time_from_start=-1.0 # disable
     onl.look_ahead_time=1.0 #The time it takes to execute after a value is entered into the online.Traject buffer.
     onl.interval=0.1 #Online.Traject Movement time from pose accumulated in buffer to next pose
     onl.init # online trajectory operation init (buffer init)
     task start,sub=1,job=7001
  10 enet0.recv
     msg=result()
     print msg
     if msg=="stop"
       onl.init # online trajectory operation init (buffer init)
       goto 10
     else
       onl.buf_in msg #Put the pose in the ‘po’ variable into the online.Traject buffer.
     endif
     goto 10
     end
```

```python
Hyundai Robot Job File; { version: 1.6, mech_type: "", total_axis: -1, aux_axis: 0 }
     import enet
     var pos
  10 pos=cpo("joint")
     enet0.send pos.str_array()
     delay 0.01
     goto 10
     end
```