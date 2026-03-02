#!/usr/bin/env bash
# DimOS Uninstaller — reverses what install.sh did, for testing iteration.
#
# Usage:
#   bash scripts/uninstall.sh            # interactive — asks what to remove
#   bash scripts/uninstall.sh --all      # remove everything
#   bash scripts/uninstall.sh --dry-run  # show what would be removed
#
set -euo pipefail

CYAN=$'\033[38;5;44m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; RED=$'\033[31m'
BOLD=$'\033[1m'; DIM=$'\033[2m'; RESET=$'\033[0m'

info()  { printf "%s▸%s %s\n" "$CYAN" "$RESET" "$*"; }
ok()    { printf "%s✓%s %s\n" "$GREEN" "$RESET" "$*"; }
warn()  { printf "%s⚠%s %s\n" "$YELLOW" "$RESET" "$*"; }
err()   { printf "%s✗%s %s\n" "$RED" "$RESET" "$*" >&2; }
dim()   { printf "%s%s%s\n" "$DIM" "$*" "$RESET"; }

DRY_RUN=0
ALL=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --all)     ALL=1; shift ;;
        --help|-h)
            echo "Usage: $0 [--all] [--dry-run] [--help]"
            echo "  --all       Remove everything without prompting"
            echo "  --dry-run   Show what would be removed"
            exit 0 ;;
        *) shift ;;
    esac
done

do_rm() {
    if [[ "$DRY_RUN" == "1" ]]; then
        dim "[dry-run] rm -rf $1"
    else
        rm -rf "$1"
        ok "removed $1"
    fi
}

should_remove() {
    local msg="$1"
    if [[ "$ALL" == "1" ]]; then return 0; fi
    if [[ "$DRY_RUN" == "1" ]]; then return 0; fi
    local yn
    printf "%s [y/N] " "$msg"
    read -r yn
    [[ "$yn" =~ ^[Yy] ]]
}

printf "\n%s%sDimOS Uninstaller%s\n\n" "$BOLD" "$CYAN" "$RESET"

# ─── DimOS project dirs ──────────────────────────────────────────────────────
for dir in "$HOME/dimos-project" "$HOME/dimos"; do
    if [[ -d "$dir" ]]; then
        local_size="$(du -sh "$dir" 2>/dev/null | cut -f1 || echo "?")"
        if should_remove "Remove ${dir}/ (${local_size})?"; then
            do_rm "$dir"
        fi
    fi
done

# ─── System packages (apt) ───────────────────────────────────────────────────
if command -v apt-get &>/dev/null; then
    PKGS="portaudio19-dev git-lfs libturbojpeg python3-dev pre-commit"
    installed=""
    for pkg in $PKGS; do
        dpkg -l "$pkg" &>/dev/null && installed+=" $pkg"
    done
    if [[ -n "$installed" ]]; then
        if should_remove "Remove system packages:${installed}?"; then
            if [[ "$DRY_RUN" == "1" ]]; then
                dim "[dry-run] sudo apt-get remove -y$installed"
            else
                sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get remove -y $installed
                ok "system packages removed"
            fi
        fi
    else
        dim "  no DimOS system packages found"
    fi
fi

# ─── uv ──────────────────────────────────────────────────────────────────────
if command -v uv &>/dev/null; then
    if should_remove "Remove uv ($(uv --version 2>/dev/null))?"; then
        if [[ "$DRY_RUN" == "1" ]]; then
            dim "[dry-run] rm ~/.local/bin/uv ~/.local/bin/uvx"
            dim "[dry-run] rm -rf ~/.cache/uv"
        else
            rm -f "$HOME/.local/bin/uv" "$HOME/.local/bin/uvx"
            rm -rf "$HOME/.cache/uv"
            ok "uv removed"
        fi
    fi
fi

# ─── sysctl ──────────────────────────────────────────────────────────────────
if [[ -f /etc/sysctl.d/99-dimos.conf ]]; then
    if should_remove "Remove LCM sysctl config (/etc/sysctl.d/99-dimos.conf)?"; then
        if [[ "$DRY_RUN" == "1" ]]; then
            dim "[dry-run] sudo rm /etc/sysctl.d/99-dimos.conf"
        else
            sudo rm -f /etc/sysctl.d/99-dimos.conf
            sudo sysctl -w net.core.rmem_max=212992 2>/dev/null || true
            sudo sysctl -w net.core.rmem_default=212992 2>/dev/null || true
            ok "sysctl config removed, buffers reset to defaults"
        fi
    fi
fi

# ─── Nix ─────────────────────────────────────────────────────────────────────
if [[ -d /nix ]]; then
    if should_remove "Remove Nix completely (/nix, daemon, build users)?"; then
        if [[ "$DRY_RUN" == "1" ]]; then
            dim "[dry-run] stop nix-daemon"
            dim "[dry-run] rm -rf /nix /etc/nix /etc/profile.d/nix*.sh"
            dim "[dry-run] remove nixbld users + group"
            dim "[dry-run] rm -rf ~/.nix-profile ~/.nix-defexpr ~/.nix-channels ~/.config/nix"
        else
            sudo systemctl stop nix-daemon.socket nix-daemon.service 2>/dev/null || true
            sudo systemctl disable nix-daemon.socket nix-daemon.service 2>/dev/null || true
            sudo rm -f /etc/systemd/system/nix-daemon.service /etc/systemd/system/nix-daemon.socket
            sudo rm -f /etc/systemd/system/sockets.target.wants/nix-daemon.socket
            sudo systemctl daemon-reload 2>/dev/null || true
            sudo rm -rf /nix /etc/nix
            sudo rm -f /etc/profile.d/nix.sh /etc/profile.d/nix-daemon.sh
            rm -rf "$HOME/.nix-profile" "$HOME/.nix-defexpr" "$HOME/.nix-channels" "$HOME/.config/nix"
            # Restore shell rc backups that Nix created (prevents reinstall failures)
            for rc in /etc/bashrc /etc/bash.bashrc /etc/zshrc; do
                if [[ -f "${rc}.backup-before-nix" ]]; then
                    sudo mv "${rc}.backup-before-nix" "$rc"
                fi
            done
            for i in $(seq 1 32); do sudo userdel "nixbld$i" 2>/dev/null || true; done
            sudo groupdel nixbld 2>/dev/null || true
            ok "Nix removed (open a new shell to clear environment)"
        fi
    fi
fi

# ─── dimensional-applications (library install default dir) ───────────────────
if [[ -d "$HOME/dimensional-applications" ]]; then
    local_size="$(du -sh "$HOME/dimensional-applications" 2>/dev/null | cut -f1 || echo "?")"
    if should_remove "Remove ~/dimensional-applications/ (${local_size})?"; then
        do_rm "$HOME/dimensional-applications"
    fi
fi

# ─── gum (temp install from installer) ───────────────────────────────────────
for tmpgum in /tmp/gum-install.*/gum*; do
    if [[ -d "$(dirname "$tmpgum")" ]]; then
        do_rm "$(dirname "$tmpgum")"
        break
    fi
done

# ─── tmp installer files ─────────────────────────────────────────────────────
for tmp in /tmp/dimos-install.*.sh /tmp/dimos-replay-*.log /tmp/dimos-nix-* /tmp/dimos-test-*; do
    if [[ -e "$tmp" ]]; then
        do_rm "$tmp"
    fi
done

printf "\n%s✓ cleanup complete%s\n\n" "$GREEN" "$RESET"
[[ "$DRY_RUN" == "1" ]] && dim "  (dry-run — nothing was actually removed)"
