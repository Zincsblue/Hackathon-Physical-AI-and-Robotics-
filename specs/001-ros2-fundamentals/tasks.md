---
description: "Task list template for feature implementation"
---

# Tasks: ROS 2 Fundamentals Module

**Input**: Design documents from `/specs/001-ros2-fundamentals/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create ROS 2 development environment on Ubuntu 22.04
- [ ] T002 [P] Configure ROS 2 workspace with proper package structure
- [ ] T003 [P] Setup Python environment with rclpy dependencies
- [ ] T004 Create base ROS 2 node templates that all stories depend on
- [ ] T005 Configure RViz for URDF visualization
- [ ] T006 Setup environment configuration management for ROS 2

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [X] T007 Create ROS 2 package structure following ament_python conventions
- [X] T008 [P] Create basic package.xml with proper dependencies
- [X] T009 [P] Create setup.py following ament_python guidelines
- [X] T010 Create base node template with error handling in my_robot_tutorial/my_node.py
- [X] T011 Setup launch directory structure in my_robot_tutorial/launch/
- [X] T012 Create URDF directory structure for humanoid models
- [X] T013 Create book chapter template structure for Docusaurus textbook with learning objectives, content sections, and exercises

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understanding ROS 2 as Middleware (Priority: P1) 🎯 MVP

**Goal**: Enable students to understand how ROS 2 acts as middleware for robot control, explaining the difference between direct hardware control and using ROS 2 as an intermediary communication layer

**Independent Test**: Students can explain the difference between direct hardware control and using ROS 2 as an intermediary communication layer, and demonstrate this understanding by describing how different robot components communicate

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T013 [P] [US1] Create conceptual documentation explaining middleware concept with biological nervous system analogy
- [ ] T014 [P] [US1] Document how ROS 2 enables decoupled communication between robot components

### Implementation for User Story 1

- [X] T015 [P] [US1] Create introduction.md explaining why ROS 2 is the robotic nervous system
- [X] T016 [P] [US1] Create ros2_basics.md covering Nodes, Topics, Services
- [X] T017 [US1] Create dds_overview.md with simple explanation tied to humanoid robots
- [X] T018 [US1] Create diagrams using Mermaid to visualize ROS 2 communication patterns
- [X] T019 [US1] Add biological nervous system analogy to documentation

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Creating Basic ROS 2 Nodes (Priority: P2)

**Goal**: Enable students to create publisher and subscriber nodes to understand the fundamental communication patterns in ROS 2 using humanoid robot examples

**Independent Test**: Students can create a working ROS 2 Python package with publisher and subscriber nodes that communicate using standard ROS 2 message types for humanoid-related data (e.g., joint angles with sensor_msgs/JointState, sensor data with sensor_msgs/Imu)

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T020 [P] [US2] Create test to verify publisher node sends joint state messages
- [ ] T021 [P] [US2] Create test to verify subscriber node receives joint state messages

### Implementation for User Story 2

- [X] T022 [P] [US2] Create ROS 2 publisher node for joint state in my_robot_tutorial/joint_state_publisher.py
- [X] T023 [P] [US2] Create ROS 2 subscriber node for joint state in my_robot_tutorial/joint_state_subscriber.py
- [X] T024 [US2] Create publisher node that uses sensor_msgs/JointState message type
- [X] T025 [US2] Create subscriber node that processes sensor_msgs/JointState messages
- [X] T026 [US2] Add basic error handling to both publisher and subscriber nodes
- [X] T027 [US2] Create publisher_subscriber_example.md with step-by-step instructions
- [X] T028 [US2] Create launch file for publisher/subscriber coordination in my_robot_tutorial/launch/publisher_subscriber_launch.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Implementing Services and URDF (Priority: P3)

**Goal**: Enable students to create service servers/clients and URDF files to understand request-response communication and robot modeling in ROS 2

**Independent Test**: Students can create a service server/client pair and a valid URDF file that loads successfully in RViz2

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T029 [P] [US3] Create test to verify service server responds to requests
- [ ] T030 [P] [US3] Create test to verify URDF file loads in RViz2 without errors

### Implementation for User Story 3

- [X] T031 [P] [US3] Create ROS 2 service server for setting joint angles in my_robot_tutorial/set_joint_service.py
- [X] T032 [P] [US3] Create ROS 2 service client for requesting joint angle changes in my_robot_tutorial/set_joint_client.py
- [X] T033 [US3] Create simple humanoid URDF with visual-only elements in urdf/simple_arm.urdf
- [X] T034 [US3] Implement service that uses example_interfaces/srv/SetBool message type
- [X] T035 [US3] Add basic error handling to service implementation
- [X] T036 [US3] Create urdf_humanoids.md with explanation of links, joints, and visual elements
- [X] T037 [US3] Create rviz_visualization.md with instructions for loading and testing URDF
- [X] T038 [US3] Create launch file for service coordination in my_robot_tutorial/launch/service_launch.py

**Checkpoint**: All user stories should now be independently functional

---

[Add more user story phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T039 [P] Book chapter documentation updates in docs/ with ROS 2 best practices, learning objectives, and exercises
- [ ] T040 Code cleanup and refactoring following ROS 2 conventions
- [ ] T041 Performance optimization for real-time robot control
- [ ] T042 [P] Additional unit tests (if requested) in tests/unit/
- [ ] T043 Safety checks for robot control code
- [ ] T044 Run quickstart.md validation with actual ROS 2 environment
- [X] T045 Create workspace_setup.md with detailed instructions, learning objectives, and exercises
- [X] T046 Create python_package.md with ament_python guidelines, examples, and practice problems
- [X] T047 Create python_ros_bridge.md explaining Python-to-ROS connection with narrative content and exercises
- [X] T048 Create hands_on_exercises.md with complete example projects, challenges, and solutions
- [X] T049 Create summary_checklist.md for knowledge verification with chapter summary and assessment questions
- [X] T050 Verify all examples run on Ubuntu 22.04 with ROS 2 Humble

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all documentation tasks for User Story 1 together:
T013 [US1] Create conceptual documentation explaining middleware concept with biological nervous system analogy
T014 [US1] Document how ROS 2 enables decoupled communication between robot components

# Launch all content creation tasks for User Story 1 together:
T015 [US1] Create introduction.md explaining why ROS 2 is the robotic nervous system
T016 [US1] Create ros2_basics.md covering Nodes, Topics, Services
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence