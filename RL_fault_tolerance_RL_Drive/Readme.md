# 🚗 강화학습 기반의 Fault Tolerance 자율주행

## 🚩 개요

이 프로젝트는 MATLAB 및 Simulink 환경에서 **심층 결정론적 정책 경사(Deep Deterministic Policy Gradient, DDPG)** 알고리즘을 사용하여 Fault Tolerance 시스템을 위한 강화학습 에이전트를 훈련하고 테스트하는 것을 목표로 합니다.

Simulink로 구현된 가상 주행 환경에서, RL 에이전트는 선행 차량과의 안전거리를 유지하며 설정된 속도로 주행하도록 학습됩니다. 사용자는 스크립트의 설정을 변경하여 새로운 에이전트를 **훈련**하거나, 이미 훈련된 에이전트를 불러와 **성능을 테스트**할 수 있습니다.

  - **알고리즘**: DDPG (Deep Deterministic Policy Gradient)
  - **환경**: Simulink 모델 (`RL_fault_tilerance_Setup.slx`)
  - **관측 (Observations)**: 9개의 연속적인 상태 값 (차량의 동적 상태)
  - **행동 (Actions)**: 2개의 연속적인 행동 값 (가속도 `accel`, 조향각 `steer`)

-----

## 🔧 요구 사항
  - MATLAB (2024b 환경 권장)
  - Simulink
  - Reinforcement Learning Toolbox
  - Deep Learning Toolbox
  - Parallel Computing Toolbox (병렬 훈련 시 권장)
  - NVIDIA GPU 및 CUDA (병렬 훈련 시 권장)

-----

## 📂 파일 구성

```
.
├── main.m                  # 메인 스크립트 (훈련/테스트 실행)
├── RL_fault_tolerance_Setup.m # 환경 설정 스크립트
├── rl_based_ACC_test.slx   # Simulink 주행 환경 모델
├── trainedDDPGAgent.mat    # 사전 훈련된 에이전트 (테스트용)
└── trainedDDPGAgent_2.mat  # 훈련 시 새로 저장될 에이전트
```

-----

## ▶️ 실행 방법

1.  **사전 설정 스크립트 실행**
    MATLAB에서 `RL_fault_tolerance_Setup.m` 파일을 먼저 실행하여 시뮬레이션에 필요한 변수와 환경을 설정합니다.

    ```matlab
    >> RL_fault_tolerance_Setup
    ```

2.  **모드 선택 (훈련 또는 테스트)**
    `main.m` 파일을 열고 `doTraining` 변수 값을 설정하여 원하는 작업을 선택합니다.

      - **새로운 에이전트 훈련 시:**

        ```matlab
        doTraining = true;  % train
        ```

      - **기존 에이전트 테스트 시:**

        ```matlab
        doTraining = false; % test
        ```
        > **Note:** 테스트를 위해서는 `trainedDDPGAgent.mat` 파일이 반드시 경로에 있어야 합니다.

3.  **메인 스크립트 실행**
    MATLAB 편집기 또는 명령창에서 `main.m` 스크립트를 실행합니다.

    ```matlab
    >> main
    ```

      - **훈련 모드**에서는 `rlTrainingOptions`에 설정된 `StopTrainingValue=3000` 에피소드까지 학습이 진행됩니다. 학습이 완료되면 `trainedDDPGAgent.mat` 파일이 작업 폴더에 생성됩니다.
      - **테스트 모드**에서는 `trainedDDPGAgent.mat` 파일을 불러온 후, Simulink 시뮬레이션을 실행하여 사전 학습된 에이전트의 주행 성능을 시각적으로 확인합니다.

-----

## 💻 코드 상세 설명

`main.m` 스크립트는 다음과 같은 6개의 주요 단계로 구성됩니다.

1.  **GPU 및 시스템 설정**

      - `gpuDevice(1)`: 연산에 사용할 GPU를 지정합니다.
      - `open_system(...)`: ACC 테스트용 Simulink 모델을 엽니다.

2.  **RL 환경 정의**

      - `rlSimulinkEnv`: Simulink 모델(`mdl`), RL Agent 블록(`agentblk`), 관측 사양(`obsInfo`), 행동 사양(`actInfo`)을 바탕으로 강화학습 환경을 구성합니다.

3.  **신경망 및 에이전트 생성**

      - **Critic Network**: 상태(Observation)와 행동(Action)을 입력받아 행동-가치 함수 $Q(s, a)$를 근사하는 신경망을 `dlnetwork`를 이용해 구성합니다.
      - **Actor Network**: 상태(Observation)를 입력받아 최적의 행동(Action)을 결정하는 정책 $\\mu(s)$를 근사하는 신경망을 구성합니다.
      - **Agent 생성**: 정의된 Actor와 Critic 신경망, 그리고 학습 파라미터(`rlDDPGAgentOptions`)를 결합하여 `rlDDPGAgent`를 최종적으로 생성합니다.

4.  **에이전트 옵션 설정**

      - `rlDDPGAgentOptions`: 샘플 시간($T\_s$), Actor/Critic의 학습률, 경험 버퍼 크기($10^6$), 탐색을 위한 노이즈 분산 및 감소율 등 DDPG 에이전트의 세부 하이퍼파라미터를 설정합니다.

5.  **훈련 옵션 및 실행**

      - `rlTrainingOptions`: 최대 에피소드($10^4$), 에피소드당 최대 스텝, 학습 진행 상황 시각화, 병렬 학습 옵션 등을 설정합니다.
      - `doTraining` 값에 따라 `train(agent, env, trainingOpts)` 함수를 호출하여 훈련을 시작하거나, `load(...)` 함수로 기존 에이전트를 불러옵니다.

6.  **시뮬레이션**

      - `sim(mdl)`: 훈련이 완료되었거나 불러온 에이전트를 Simulink 모델에 적용하여 최종 성능을 검증하는 시뮬레이션을 실행합니다.