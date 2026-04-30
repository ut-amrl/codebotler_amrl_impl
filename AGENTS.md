# AGENTS.md

## Purpose
This ROS 2 package is the AMRL Jackal implementation of CodeBotler's robot action contract. It exposes the action servers that `../codebotler/robot_client.py` calls, and bridges those actions to AMRL navigation, OpenAI VLM perception, speech, GUI ask/response, pick, and place topics/services.

## Main Flow
- The `../../tmux/codebotler/.tmuxinator.yaml` profile launches `actions.py` directly from the repo root in the `codebotler` conda env. `gui.py` is present and may be uncommented there when needed.
- `actions.py` owns the production action servers: `/go_to_server`, `/get_current_location_server`, `/is_in_room_server`, `/say_server`, `/get_all_rooms_server`, `/ask_server`, `/pick_server`, and `/place_server`.
- `go_to` normalizes requested names by lowercasing, stripping `"'s"`, and replacing hyphens with spaces, then looks up coordinates in `data.yaml`, publishes `amrl_msgs/Localization2DMsg` goals, and watches navigation status/localization.
- `is_in_room` uses the latest camera image and asks an OpenAI vision model for a structured yes/no object-presence answer.
- `say` uses `espeak`/`aplay` and publishes the configured robot-say topic.
- `ask` publishes a JSON prompt on `ROBOT_ASK_TOPIC`, waits for `HUMAN_RESPONSE_TOPIC`, and returns that response through the action result. Blocking action callbacks and status subscriptions use a reentrant callback group because they wait on subscriber events.
- `pick` and `place` publish `/pick_request` and `/place_request`, then wait on `/pick_goal_status` and `/place_goal_status` boolean topics.
- `gui.py` is a fullscreen Tkinter UI that displays speech/ask prompts, speaks ask questions through `espeak`/`aplay`, retains the unused microphone helper, and publishes human responses.

## Key Files
- `data.yaml`: source of topic names, thresholds, map selection, distance threshold, speech timing, and named locations.
- `CMakeLists.txt`/`package.xml`: install Python executables and `data.yaml`; depend on `amrl_msgs` and `cobot_codebotler_actions`.
- `test_pick_client.py`: retained ad hoc pick action test client.

## Run And Verify
- From the workspace root, build this package with `colcon build --packages-select codebotler_amrl_impl --symlink-install`, then source `install/setup.bash`.
- Launch through `tmux/codebotler`, or run `python3 actions.py` and `python3 gui.py` from this package after the AMRL/ut_jackal stack, camera, navigation topics, `cobot_codebotler_actions`, pick/place status topics, OpenAI credentials, and audio devices are available.
- Syntax check inside the root workspace container: `./container cmd --name cobot_demo 'conda run --no-capture-output -n codebotler python3 -m py_compile src/codebotler_amrl_impl/actions.py src/codebotler_amrl_impl/gui.py src/codebotler_amrl_impl/test_pick_client.py'`.
- There is no formal test suite. Full validation needs the ROS 2 graph, AMRL messages, action definitions, camera image topic, navigation status/localization topics, pick/place status topics, OpenAI credentials, and audio devices.

## Change Guidance
- Keep the action contract synchronized with CodeBotler: generated code calls `go_to`, `get_current_location`, `get_all_rooms`, `is_in_room`, `say`, `ask`, `pick`, and `place`; the action names above are the integration boundary.
- Treat `data.yaml` as the configuration source. New location keys should match `go_to` normalization and the file's lower-case naming convention.
- Preserve cancellation/abort behavior in action callbacks, especially navigation. Long waits must not assume a single-threaded executor; `actions.py` uses `MultiThreadedExecutor`.
- Be careful with hardware side effects. `say`, GUI ask speech, navigation, pick, and place invoke real devices or external commands.
- Avoid committing generated `images/`, audio files, or local output artifacts.
