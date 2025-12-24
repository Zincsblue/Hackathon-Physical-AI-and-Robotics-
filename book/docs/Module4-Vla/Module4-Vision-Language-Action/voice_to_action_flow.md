# Voice-to-Action Pipeline Flowcharts

## 1. High-Level Voice-to-Action Flow

```mermaid
graph TD
    A[Voice Command] --> B[Audio Capture]
    B --> C[Audio Preprocessing]
    C --> D[Whisper ASR]
    D --> E[Transcribed Text]
    E --> F[Language Understanding]
    F --> G[Command Interpretation]
    G --> H[Action Planning]
    H --> I[Action Execution]
    I --> J[Robot Action]
    J --> K[Feedback/Sensors]
    K --> F
```

## 2. Detailed Whisper Integration Flow

```mermaid
graph TD
    A[Microphone Input] --> B{Audio Buffer<br>Ready?}
    B -->|Yes| C[Audio Normalization]
    B -->|No| B
    C --> D[Format Conversion<br>to Whisper Format]
    D --> E[Whisper Model<br>Transcription]
    E --> F{Confidence<br>Threshold?}
    F -->|Low| G[Request Repeat<br>or Clarification]
    F -->|High| H[Text Output]
    G --> A
    H --> I[Command Parsing]
    I --> J[Intent Recognition]
    J --> K[Entity Extraction]
    K --> L[Action Planning]
```

## 3. Error Handling Flow

```mermaid
graph TD
    A[Voice Input Received] --> B[Audio Quality Check]
    B --> C{Audio Quality<br>Sufficient?}
    C -->|No| D[Request Clearer Speech]
    C -->|Yes| E[Whisper Transcription]
    D --> M[End - User Feedback]
    E --> F{Transcription<br>Successful?}
    F -->|No| G[Use Alternative ASR]
    F -->|Yes| H[Validate Command]
    G --> I{Alternative<br>Successful?}
    I -->|No| J[Request Clarification]
    I -->|Yes| H
    J --> K{User Provides<br>Clarification?}
    K -->|Yes| A
    K -->|No| L[Abort Command]
    H --> N{Command<br>Valid?}
    N -->|No| J
    N -->|Yes| O[Execute Command]
    L --> M
    O --> M
```

## 4. Real-time Voice Processing Loop

```mermaid
flowchart TD
    Start([System Start]) --> Listen{Listening<br>for Command?}
    Listen -->|No| WaitForCommand[Wait & Monitor]
    WaitForCommand --> Listen
    Listen -->|Yes| Capture[Capture Audio<br>for 3-5 seconds]
    Capture --> Preprocess[Audio<br>Preprocessing]
    Preprocess --> Whisper[Whisper<br>Transcription]
    Whisper --> Validate{Valid<br>Command?}
    Validate -->|No| Discard[Discard & Listen Again]
    Validate -->|Yes| Plan[Plan Actions]
    Discard --> Listen
    Plan --> Execute[Execute Actions]
    Execute --> Monitor[Monitor Execution]
    Monitor --> Feedback{Action<br>Successful?}
    Feedback -->|Yes| Success[Report Success]
    Feedback -->|No| Error[Handle Error]
    Success --> Listen
    Error --> Listen
```

## 5. Multi-Modal Validation Flow

```mermaid
graph TD
    A[Voice Command] --> B[Whisper Transcription]
    B --> C[Extract Command Elements]
    C --> D[Visual Scene Analysis]
    D --> E{Are Objects<br>in Command<br>Visible?}
    E -->|Yes| F[Cross-Reference<br>Objects & Commands]
    E -->|No| G[Request Clarification<br>or Search]
    F --> H[Validate Action<br>Feasibility]
    G --> I[Ask for<br>Additional Info]
    H --> J{Action<br>Feasible?}
    J -->|Yes| K[Plan & Execute]
    J -->|No| L[Report Unfeasible]
    I --> B
    L --> M[End with Error]
    K --> N[Execute Action]
    N --> O[Monitor Result]
    O --> P{Result<br>Expected?}
    P -->|Yes| Q[Report Success]
    P -->|No| R[Error Recovery]
    R --> M
    Q --> M
```

## 6. Confidence-Based Processing Flow

```mermaid
graph TD
    A[Audio Input] --> B[Preprocess Audio]
    B --> C[Whisper Transcription]
    C --> D[Get Confidence Score]
    D --> E{Confidence<br>> 0.8?}
    E -->|Yes| F[Use Transcription<br>Directly]
    E -->|No| G{Confidence<br>> 0.5?}
    G -->|Yes| H[Request Confirmation]
    G -->|No| I[Request Repetition]
    H --> J{User<br>Confirms?}
    J -->|Yes| K[Use Transcription]
    J -->|No| L[Request Repetition]
    I --> M[Listen Again]
    L --> M
    M --> B
    K --> N[Process Command]
    F --> N
    N --> O[Execute Action]
```

## 7. Pipeline Integration with ROS 2

```mermaid
graph LR
    subgraph "User"
        A[Voice Command]
    end

    subgraph "ROS 2 Nodes"
        B[Audio Input Node]
        C[Whisper ASR Node]
        D[Command Processing Node]
        E[Action Planning Node]
        F[Robot Control Node]
    end

    subgraph "Robot"
        G[Physical Action]
        H[Sensor Feedback]
    end

    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
    H --> D
```

## 8. Voice Command Processing State Machine

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Recording: Voice Activity Detected
    Recording --> Processing: Audio Buffer Complete
    Processing --> Validated: Whisper Transcription
    Validated --> Planned: Command Validated
    Planned --> Executing: Action Plan Ready
    Executing --> Idle: Action Complete
    Processing --> Idle: Transcription Failed
    Validated --> Idle: Command Invalid
    Planned --> Idle: Planning Failed
    Executing --> Idle: Execution Failed
```

## 9. Component Interaction Flow

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Microphone    │───▶│ Audio Handler   │───▶│ Whisper Node    │
│                 │    │                 │    │                 │
│ Captures voice  │    │ Preprocesses    │    │ Transcribes     │
│ commands        │    │ audio for       │    │ audio to text   │
└─────────────────┘    │ Whisper         │    └─────────────────┘
                       │ requirements    │              │
                       └─────────────────┘              ▼
                                                      ┌─────────────────┐
                                                      │ Command Parser  │
                                                      │                 │
                                                      │ Interprets text │
                                                      │ command         │
                                                      └─────────────────┘
                                                                      │
                                                                      ▼
                        ┌─────────────────┐              ┌─────────────────┐
                        │ Action Planner  │<─────────────│ LLM Interpreter │
                        │                 │              │                 │
                        │ Plans robot     │              │ Determines      │
                        │ actions         │              │ action sequence │
                        └─────────────────┘              └─────────────────┘
                              │
                              ▼
                        ┌─────────────────┐
                        │ Robot Executor  │
                        │                 │
                        │ Executes planned│
                        │ actions         │
                        └─────────────────┘
```

## 10. Processing Pipeline with Feedback

```mermaid
graph TD
    A[Voice Command] --> B[Audio Capture]
    B --> C[Preprocessing]
    C --> D[Whisper ASR]
    D --> E[Text Output]
    E --> F[Intent Recognition]
    F --> G[Entity Extraction]
    G --> H[Action Planning]
    H --> I[Action Execution]
    I --> J[Sensor Feedback]
    J --> K{Action Successful?}
    K -->|No| L[Error Handling]
    K -->|Yes| M[Task Complete]
    L --> N{Retry Possible?}
    N -->|Yes| O[Replan Action]
    N -->|No| P[Report Failure]
    O --> H
    P --> Q[End Process]
    M --> Q
    Q --> R{New Command?}
    R -->|Yes| A
    R -->|No| S[Idle]
```

These flowcharts illustrate the complete voice-to-action pipeline in VLA systems, showing how voice commands are processed through Whisper ASR and transformed into robot actions, with appropriate error handling and feedback mechanisms.