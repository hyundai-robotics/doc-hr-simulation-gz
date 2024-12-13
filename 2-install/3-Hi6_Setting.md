# 이더넷 통신 매뉴얼

이 매뉴얼은 Hi6 컨트롤러에 기본 기능으로 제공되는 enet 모듈 및 Open API을 활용하여, Ethernet UDP/IP 및 TCP/IP 통신을 통해 외부 장비와 인터페이스하는 방법을 설명합니다.

아래 다이어그램을 참조하여 이더넷 통신의 기본 연결 구조를 확인하세요.

LAN 통신 : 동일 네트워크 내 또는 다른 호스트/PC와의 통신이 가능합니다.

<div align="center" style="background-color: white; display: inline-block; padding: 10px;">
  <img src="../_assets/hub.png" alt="hub" width="500"/>
</div>

<div align="center" style="background-color: white; display: inline-block; padding: 10px;">
  <img src="../_assets/ip_adress.png" alt="ip address" width="500"/>
</div>

## 2-1 케이블 연결

다른 장비와 통신하기 위해 사용자 이더넷 포트에 이더넷 케이블을 연결합니다.

- LAN1, LAN2, LAN3 중 하나를 선택
- Hi6-N 컨트롤러 : 도어 중간 패널의 메인 모듈 상단
- Hi6-T 컨트롤러 : 컨트롤러의 전면의 이더넷 포트

<div style="display: flex; justify-content: space-around; align-items: center; background-color: white; padding: 10px;">
  <div style="margin-right: 20px;">
    <img src="../_assets/Hi6-N.png" alt="Hi6-N" width="240"/>
  </div>
  <div>
    <img src="../_assets/Hi6-T.png" alt="Hi6-T" width="240"/>
  </div>
</div>

## 2-2 컨트롤러 네트워크 설정

티칭 팬던트를 사용하여 Hi6 컨트롤러의 IP주소를 설정합니다.

1. **[F2:시스템] → [2:제어 파라미터] → [9:네트워크]** 로 이동합니다.
2. [1:환경 설정] → 케이블이 연결된 **[LAN# (일반)]** 을 선택합니다. 
3. IP주소와 서브넷 마스크를 입력합니다.
  * 참고 : 필요한 경우 게이트웨이와 포트 포워딩을 설정합니다.
4. 설정 완료후 **[F7:확인]** 을 누르고 컨트롤러를 재부팅합니다.

<div align="center" style="background-color: white; display: inline-block; padding: 10px;">
  <img src="../_assets/ip_setting.png" alt="ip setting" width="500"/>
</div>

다른 장치외의 이더넷 통신이 가능한지 확인합니다.
 * 다른 장치가 PC인 경우, **[명령 프롬프트]** 에서 Ping 테스트를 통해 두 장치 간의 네트워크 연결 상태를 확인하세요.

<div align="center" style="background-color: white; display: inline-block; padding: 10px;">
  <img src="../_assets/ping_check.png" alt="ping check" width="500"/>
</div>→