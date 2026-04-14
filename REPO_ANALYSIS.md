# Repository Analysis: sound_ros_noetic

## Purpose
This repository contains a ROS Noetic-based audio pipeline for:
1. Detecting speech-like sound from microphones.
2. Recording triggered audio snippets.
3. Transcribing speech with Whisper.
4. Publishing transcription results to ROS topics.
5. (Optionally) buffering tasks and classifying them via OpenAI API.

## Architecture Overview
- Runtime environment is provided via Docker (`sound_noetic/Dockerfile`, `docker-compose.yaml`).
- ROS package: `sound_noetic/src/sound_send`.
- Main launch entry: `sound_noetic/src/sound_send/launch/whisper.launch`.

## Core Audio/ASR Flow
1. `sound_trigger.py`
   - Opens microphone stream and computes volume + simple SNR-like condition.
   - Publishes trigger to `/first_mic/sound_trigger` or `/second_mic/sound_trigger`.
2. `sound_subsc.py`
   - Subscribes to trigger topic.
   - Records microphone input while triggers continue.
   - Saves WAV when silence timeout passes.
   - Publishes saved file path to `/.../audio_path`.
3. `whisper_node.py`
   - Loads Whisper model (CUDA if available).
   - Launches `sound_subsc.py` as a subprocess.
   - Subscribes to `/.../audio_path`, denoises audio, transcribes (English), publishes to `/whisper_result`.
   - Publishes whisper state (`done`) and removes processed WAV.

## Task Queue / NLP Utilities
- `task_list.py` and `task_list_debug.py`
  - Append `/whisper_result` messages into an in-memory queue.
  - Persist queue to `sentence_task.txt`.
  - Publish next task to `/send_task` when `/please_task` is received.
- `judge_task.py`
  - Subscribes to `/send_task`.
  - Sends task text to OpenAI Chat Completions (`gpt-5-mini`) using local API key/prompt files.
  - Publishes judged result to `/GPT_result` and logs to file.
- `src/list_to_GPT.py`
  - Socket server forwarding TCP text messages to `/whisper_result`.

## Operational Notes
- Uses host networking, PulseAudio socket/cookie mount, and `/dev/snd` passthrough in Docker compose.
- Includes a `time_sync_guide.md` for Chrony setup in ROS multi-machine deployments.
- `package.xml` currently has dependency typos (`std_mssgs`) and placeholder metadata (`TODO` license).

## Current State / Risks
- Hardcoded absolute paths in task scripts target `/root/HSR/catkin_ws/...`.
- Some error logs swallow exception details.
- Typo in ROS dependencies may break clean catkin dependency resolution.
- No automated tests included in repo.
