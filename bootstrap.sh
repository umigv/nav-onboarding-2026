#!/usr/bin/env bash
set -Eeuo pipefail

PIXI_BIN="$HOME/.pixi/bin"
# Make freshly-installed binaries visible to `command -v` within this same run.
export PATH="$PIXI_BIN:$PATH"

log() { printf '\033[1;34m===> %s\033[0m\n' "$*"; }
trap 'printf "\033[1;31mERROR at line %s: %s (exit %s)\033[0m\n" "$LINENO" "$BASH_COMMAND" "$?" >&2' ERR

assert_exists() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: '$1' not on PATH after install" >&2; exit 1; } }

detect_platform() {
    case "$(uname -s)" in
        Darwin)
            echo macos ;;
        Linux)
            if grep -qiE 'microsoft|wsl' /proc/version 2>/dev/null; then
                echo wsl
            else
                echo linux
            fi ;;
        *)
            echo "Unsupported OS: $(uname -s)" >&2; exit 1 ;;
    esac
}

case "$(detect_platform)" in
    macos) WANT_VSCODE=1; WANT_DIALOUT=0 ;;  # dialout doesn't exist on mac
    wsl)   WANT_VSCODE=0; WANT_DIALOUT=1 ;;  # editor lives on the Windows host
    linux) WANT_VSCODE=1; WANT_DIALOUT=1 ;;
esac

ensure_prereqs() {
    # Some installers (i.e. brew) needs sudo but runs non-interactively and cannot prompt for a password, so cache sudo
    # credentials first.
    sudo -v
    case "$(detect_platform)" in
        linux|wsl)
            log "Updating apt and installing base tools"
            sudo apt update
            sudo apt install -y curl wget git ca-certificates gnupg ;;
    esac
}

# ---- Tools -----------------------------------------------------------------------------------------------------------

install_pixi() {
    command -v pixi >/dev/null 2>&1 && { log "pixi already installed"; return; }
    log "Installing pixi"
    # We manage the rc PATH block ourselves, so keep the installer out of the rc file.
    curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 bash
    assert_exists pixi
}

install_tools() {
    log "Installing just, direnv, and gh via pixi global"
    pixi global install just direnv gh
    assert_exists just
    assert_exists direnv
    assert_exists gh
}

install_brew() {
    command -v brew >/dev/null 2>&1 && { log "brew already installed"; return; }
    log "Installing Homebrew"
    NONINTERACTIVE=1 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    [ -x /opt/homebrew/bin/brew ] && eval "$(/opt/homebrew/bin/brew shellenv)"
}

install_vscode() {
    [ "$WANT_VSCODE" = 1 ] || return 0   # bare `return` would propagate the failed test (1) and trip `set -e`
    command -v code >/dev/null 2>&1 && { log "VSCode already installed"; return; }
    local reply
    read -r -p "VSCode not found. Install it? Choose 'n' if you have another editor. [Y/n] " reply
    case "$reply" in
        [Nn]*) log "Skipping VSCode install (using your own editor)"; return 0 ;;
    esac
    log "Installing VSCode"
    case "$(detect_platform)" in
        macos)
            # Brew is only used for the VSCode cask, so install it lazily here.
            install_brew
            brew install --cask visual-studio-code ;;
        linux)
            if [ ! -f /etc/apt/sources.list.d/vscode.list ]; then
                keyring=/usr/share/keyrings/microsoft.gpg
                repo=https://packages.microsoft.com/repos/code
                wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /tmp/microsoft.gpg
                sudo install -D -o root -g root -m 644 /tmp/microsoft.gpg "$keyring"
                echo "deb [arch=amd64,arm64,armhf signed-by=$keyring] $repo stable main" \
                    | sudo tee /etc/apt/sources.list.d/vscode.list >/dev/null
                rm -f /tmp/microsoft.gpg
                sudo apt update
            fi
            sudo apt install -y code ;;
    esac
}

# ---- Shell integration -----------------------------------------------------------------------------------------------

configure_direnv() {
    local toml="$HOME/.config/direnv/direnv.toml"
    log "Configuring direnv"
    mkdir -p "$(dirname "$toml")"
    # We silence direnv via DIRENV_LOG_FORMAT= in the shell rc rather than log_format = "-" in the toml (direnv bug:
    # https://github.com/direnv/direnv/issues/1418). The toml must exist for direnv to pick up env var overrides, so we
    # touch it.
    touch "$toml"
}

update_or_append_block() {
    local rc_path="$1" body="$2" rc_name="${1#"$HOME"/}"
    local mark_start='# >>> ARV environment >>>'
    local mark_end='# <<< ARV environment <<<'
    local block="$mark_start
$body
$mark_end"

    mkdir -p "$(dirname "$rc_path")"
    touch "$rc_path"
    if grep -qF "$mark_start" "$rc_path"; then
        log "Updating configuration in ~/$rc_name"
        local tmp_block tmp_out
        tmp_block=$(mktemp)
        tmp_out=$(mktemp)
        printf '%s\n' "$block" > "$tmp_block"
        awk -v s="$mark_start" -v e="$mark_end" -v bf="$tmp_block" '
            $0 == s { while ((getline line < bf) > 0) print line; inside = 1; next }
            inside && $0 == e { inside = 0; next }
            !inside { print }
        ' "$rc_path" > "$tmp_out"
        mv "$tmp_out" "$rc_path"
        rm -f "$tmp_block"
    else
        log "Installing configuration in ~/$rc_name"
        printf '\n%s\n' "$block" >> "$rc_path"
    fi
}

# Single-quoted so $PATH/$HOME/$(...) stay literal for the user's shell to evaluate.
# shellcheck disable=SC2016
BASH_BODY='# Added by the ARV host bootstrap. Edit/remove this whole block, not pieces.
case ":$PATH:" in *":$HOME/.pixi/bin:"*) ;; *) export PATH="$HOME/.pixi/bin:$PATH" ;; esac
export DIRENV_LOG_FORMAT=
eval "$(direnv hook bash)"'

# shellcheck disable=SC2016
ZSH_BODY='# Added by the ARV host bootstrap. Edit/remove this whole block, not pieces.
case ":$PATH:" in *":$HOME/.pixi/bin:"*) ;; *) export PATH="$HOME/.pixi/bin:$PATH" ;; esac
export DIRENV_LOG_FORMAT=
eval "$(direnv hook zsh)"
# Register completions and initialize
fpath=("$HOME/.zsh/completions" $fpath)
command -v compdef >/dev/null 2>&1 || { autoload -Uz compinit && compinit; }'

# shellcheck disable=SC2016
FISH_BODY='# Managed by the ARV host bootstrap. Delete this file to uninstall.
fish_add_path --global "$HOME/.pixi/bin"
set -gx DIRENV_LOG_FORMAT ""
direnv hook fish | source'

install_bash_completion() {
    local dir="$HOME/.local/share/bash-completion/completions"
    log "Installing bash completions for just and pixi in ~/.local/share/bash-completion/completions"
    mkdir -p "$dir"
    just --completions bash > "$dir/just"
    pixi completion --shell bash > "$dir/pixi"
}

install_zsh_completion() {
    local dir="$HOME/.zsh/completions"
    log "Installing zsh completions for just and pixi in ~/.zsh/completions"
    mkdir -p "$dir"
    just --completions zsh > "$dir/_just"
    pixi completion --shell zsh > "$dir/_pixi"
}

install_fish_completion() {
    local dir="$HOME/.config/fish/completions"
    log "Installing fish completions for just and pixi in ~/.config/fish/completions"
    mkdir -p "$dir"
    just --completions fish > "$dir/just.fish"
    pixi completion --shell fish > "$dir/pixi.fish"
}

configure_shell() {
    local shell_name
    shell_name="$(basename "${SHELL:-}")"
    case "$shell_name" in
        zsh)
            update_or_append_block "$HOME/.zshrc" "$ZSH_BODY"
            install_zsh_completion ;;
        bash)
            update_or_append_block "$HOME/.bashrc" "$BASH_BODY"
            install_bash_completion ;;
        fish)
            update_or_append_block "$HOME/.config/fish/conf.d/arv.fish" "$FISH_BODY"
            install_fish_completion ;;
        *)
            if [ -z "$shell_name" ]; then
                log "Could not detect your login shell (\$SHELL is not set) - skipping shell configuration."
            else
                log "Login shell '$shell_name' is not bash, zsh, or fish - skipping shell configuration."
            fi
            log "Replicate the changes in configure_shell in bootstrap.sh for your shell"
            log "(See 'None of the above' in the README)." ;;
    esac
}

# ---- Per-user / per-machine config -----------------------------------------------------------------------------------

add_dialout() {
    [ "$WANT_DIALOUT" = 1 ] || return 0   # bare `return` would propagate the failed test (1) and trip `set -e`
    log "Adding $USER to dialout group (USB/serial device access)"
    sudo usermod -aG dialout "$USER"
}

setup_github() {
    log "GitHub setup"
    gh auth login --git-protocol ssh --web

    if ! git config --global user.name >/dev/null 2>&1; then
        git config --global user.name "$(gh api user --jq '.name // .login')"
    fi

    if ! git config --global user.email >/dev/null 2>&1; then
        local email
        email="$(gh api user --jq '.email // empty' 2>/dev/null || true)"
        [ -n "$email" ] || email="$(gh api user --jq '"\(.id)+\(.login)@users.noreply.github.com"')"
        git config --global user.email "$email"
    fi
}

# ---- Main ------------------------------------------------------------------------------------------------------------

main() {
    log "ARV host bootstrap - platform: $(detect_platform)"
    ensure_prereqs
    install_pixi
    install_tools
    install_vscode
    configure_direnv
    configure_shell
    add_dialout
    setup_github
    log "Host bootstrap complete. Open a new terminal for the changes to take effect."
}

main "$@"
