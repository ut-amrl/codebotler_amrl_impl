# CodeBotler AMRL Implementation

ROS 2 action-server implementation for the CodeBotler deployment on the AMRL Jackal/Cobot stack.

## Runtime

The `tmux/codebotler` profile launches this package directly:

```bash
cd ~/cobot_demo/src/codebotler_amrl_impl/src
conda run --no-capture-output -n codebotler python3 actions.py
conda run --no-capture-output -n codebotler python3 gui.py
```

`actions.py` exposes the CodeBotler action contract: go to, get current location, get all rooms, object presence, say, ask, pick, and place.

`gui.py` displays ask prompts, speaks ask questions through `espeak`/`aplay`, and publishes the selected answer back to `actions.py`.

## Requirements

The ROS 2 workspace must provide:

- `amrl_msgs`
- `cobot_codebotler_actions`
- the AMRL navigation/localization topics configured in `data.yaml`
- camera image topic for OpenAI VLM object checks
- pick/place request and status topics
- OpenAI credentials for `is_in_room`

Install Python dependencies with:

```bash
pip install -r requirements.txt
```

Build as a ROS package when needed:

```bash
colcon build --packages-select codebotler_amrl_impl --symlink-install
```
