# Activate topic-name tab completion for enum_echo.
# Sourced automatically by `source install/setup.bash` (workspace build)
# or `source /opt/ros/<distro>/setup.bash` (apt-installed package).
# No manual action required from the user.

# ── Direct: `enum_echo /topic` ───────────────────────────────────────────────
if command -v register-python-argcomplete3 >/dev/null 2>&1; then
    eval "$(register-python-argcomplete3 enum_echo 2>/dev/null)" || true
elif command -v register-python-argcomplete >/dev/null 2>&1; then
    eval "$(register-python-argcomplete enum_echo 2>/dev/null)" || true
fi

# ── Via ros2 run: `ros2 run ros2_msgs_enum_resolver enum_echo /topic` ─────────
#
# ros2cli registers `complete -F _python_argcomplete ros2` (always this name).
# We wrap it so position-4 completions return live topic names; everything else
# is delegated to the original argcomplete function unchanged.
#
# PROMPT_COMMAND is used so the wrap happens after ~/.bashrc finishes setting up
# ros2 completion — order of sourcing doesn't matter.

_enum_echo_ros2_wrap() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    if [[ ${COMP_CWORD} -eq 4 &&
          "${COMP_WORDS[1]:-}" == "run" &&
          "${COMP_WORDS[2]:-}" == "ros2_msgs_enum_resolver" &&
          "${COMP_WORDS[3]:-}" == "enum_echo" ]]; then
        mapfile -t COMPREPLY < <(ros2 topic list 2>/dev/null | grep "^${cur}")
        return 0
    fi
    # Delegate to ros2cli argcomplete for all other subcommands
    _python_argcomplete "$@"
}

# Called every prompt: re-installs our wrapper if ros2 completion was registered
# or re-registered after us (e.g. by ~/.bashrc).
_enum_echo_setup_ros2_complete() {
    local f
    f=$(complete -p ros2 2>/dev/null | sed -n 's/.*-F \([^ ]*\) *ros2$/\1/p')
    [[ -z "$f" || "$f" == "_enum_echo_ros2_wrap" ]] && return
    complete -o bashdefault -o default -o nospace -F _enum_echo_ros2_wrap ros2
}

_enum_echo_setup_ros2_complete
PROMPT_COMMAND="${PROMPT_COMMAND:+${PROMPT_COMMAND};}  _enum_echo_setup_ros2_complete"
