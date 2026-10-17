#!/usr/bin/env bash
# Source once in each Bash dev-container terminal; this starts no processes.
if [[ -z "${BASH_VERSION:-}" ]]; then
  printf '%s\n' 'Use a Bash terminal, then: source /workspaces/AI_arm/scripts/chess_env.sh' >&2
  return 1
fi
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' 'Use source, not bash, so settings stay in your terminal:' \
    '  source /workspaces/AI_arm/scripts/chess_env.sh' >&2
  exit 1
fi

_chess_setup_terminal() {
  local project_root entry cleaned_path=''
  local -a path_entries
  project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    printf '%s\n' 'ROS Humble is missing. Run this in your VS Code dev container.' >&2
    return 1
  fi
  if [[ ! -f "$project_root/install/setup.bash" ]]; then
    printf '%s\n' 'Project install/setup.bash is missing. Use the prepared, built workspace.' >&2
    return 1
  fi

  # An inherited laptop virtualenv uses a different Python from ROS Humble.
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    IFS=: read -r -a path_entries <<< "$PATH"
    for entry in "${path_entries[@]}"; do
      [[ "$entry" == "$VIRTUAL_ENV/bin" ]] && continue
      cleaned_path+="${entry}:"
    done
    export PATH="${cleaned_path%:}"
    unset VIRTUAL_ENV
    hash -r
  fi

  cd -- "$project_root" || return 1
  source /opt/ros/humble/setup.bash || return 1
  source install/setup.bash || return 1
  source scripts/lab_env.sh || return 1
  # On the shared Jetson our pip packages live in the checkout, not the system
  # Python (docs/JETSON.md); the container has them system-wide and no .pydeps.
  [[ -d .pydeps && ":${PYTHONPATH:-}:" != *":$PWD/.pydeps:"* ]] && export PYTHONPATH="$PWD/.pydeps${PYTHONPATH:+:$PYTHONPATH}"
  # Ubuntu's stockfish package installs to /usr/games, which is not on PATH here;
  # without it lab_chess.py silently plays as a random mover.
  [[ -d /usr/games && ":$PATH:" != *":/usr/games:"* ]] && export PATH="$PATH:/usr/games"
  # Deliberately replace old arm addresses inherited from VS Code.
  export ROBOT_IP=192.168.1.10 LAPTOP_IP=192.168.1.2
  printf '%s\n' "Chess terminal ready: $PWD" \
    "Robot: $ROBOT_IP | Laptop: $LAPTOP_IP | ROS domain: $ROS_DOMAIN_ID" \
    'Environment only: no driver, camera, calibration or motion started.'
}

if _chess_setup_terminal; then
  unset -f _chess_setup_terminal
else
  unset -f _chess_setup_terminal
  return 1
fi
