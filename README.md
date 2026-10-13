# CARLA G29 driving client

`Pilot_Sub3.py`는 CARLA 시뮬레이터에서 Logitech G29 휠로 차량을 조작하는 실험용 클라이언트입니다. 차량 상태와 차선 침범 등을 CSV로 기록하는 코드가 포함되어 있습니다. 이 저장소에는 스크립트 한 파일만 있으며, CARLA 본체와 휠 설정 파일은 포함하지 않습니다.

## 실행 전 준비

- CARLA 서버와 버전이 맞는 Python API를 설치하거나 Python 경로에 추가합니다. 코드에는 `CARLA_0.9.14` 경로가 들어 있어 해당 버전을 기준으로 작성된 것으로 보이지만, 다른 버전에서의 동작은 확인되지 않았습니다.
- `pygame`, `numpy`, CARLA Python API 및 `agents.navigation` 모듈이 필요합니다.
- Logitech G29을 연결하고, 실행 디렉터리에 `wheel_config.ini`를 만듭니다. `[G29 Racing Wheel]` 섹션에 `steering_wheel`, `throttle`, `brake`, `reverse`, `handbrake` 값을 실제 장치의 축·버튼 번호로 지정해야 합니다. 이 파일은 저장소에 포함되어 있지 않습니다.
- 스크립트의 `agents` 탐색 경로와 CSV 저장 경로 `/home/somin/CARLA_0.9.14/Logs`를 자신의 환경에 맞게 수정합니다. 현재 코드는 이 경로를 하드코딩합니다.

## 실행

CARLA 서버를 먼저 시작한 뒤, 휠 설정 파일이 있는 디렉터리에서 실행합니다.

```bash
python Pilot_Sub3.py --host 127.0.0.1 --port 2000
```

`python Pilot_Sub3.py --help`로 사용 가능한 옵션을 확인할 수 있습니다. 기본 실행은 전체 화면 창을 열고 첫 번째 조이스틱을 사용합니다. 조이스틱이 없거나 둘 이상이면 실행이 실패할 수 있습니다.

## 재현 범위

이 코드는 특정 실험 환경의 파일 경로와 차량 설정을 포함합니다. 공개 저장소만 복제한 상태에서 즉시 실행되는 예제로 보기는 어렵습니다. 실행 전 휠 매핑, CARLA 버전, `agents` 경로, 로그 경로를 확인하세요.

파일 머리말은 Intel Labs의 CARLA manual-control 코드를 출처로 표시합니다. 수정 코드의 배포·사용 조건은 원본 코드의 라이선스와 출처를 함께 확인해야 합니다.
