#!/bin/bash

# Shared helpers for locating and validating kernel build trees.

kcan_log_info() {
    if declare -F print_info >/dev/null; then
        print_info "$1"
    else
        echo "$1"
    fi
}

kcan_log_warning() {
    if declare -F print_warning >/dev/null; then
        print_warning "$1"
    else
        echo "$1" >&2
    fi
}

kcan_log_error() {
    if declare -F print_error >/dev/null; then
        print_error "$1"
    else
        echo "$1" >&2
    fi
}

kcan_reset_kernel_location_state() {
    KCAN_KERNEL_LOCATION_SOURCE=""
    KCAN_KERNEL_LOCATION_ERROR=""
    KCAN_KERNEL_LOCATION_ATTEMPTS=()
}

kcan_record_kernel_attempt() {
    local candidate="$1"
    [[ -n "$candidate" ]] || return 0
    KCAN_KERNEL_LOCATION_ATTEMPTS+=("$candidate")
}

kcan_is_kbuild_tree() {
    local dir="$1"
    [[ -n "$dir" ]] && [[ -f "$dir/Makefile" ]] && [[ -d "$dir/include" ]] && {
        [[ -d "$dir/scripts" ]] || [[ -d "$dir/arch" ]]
    }
}

kcan_resolve_dir() {
    local dir="$1"

    [[ -n "$dir" ]] || return 1
    [[ -e "$dir" ]] || return 1

    if command -v readlink >/dev/null 2>&1; then
        readlink -f "$dir" 2>/dev/null && return 0
    fi

    (
        cd "$dir" 2>/dev/null &&
        pwd -P
    )
}

kcan_validate_kernel_location() {
    local candidate="$1"
    local resolved=""

    if [[ -z "$candidate" ]]; then
        KCAN_KERNEL_LOCATION_ERROR="Kernel tree path is empty"
        return 1
    fi

    if [[ ! -e "$candidate" ]]; then
        KCAN_KERNEL_LOCATION_ERROR="Kernel tree path does not exist: $candidate"
        return 1
    fi

    resolved=$(kcan_resolve_dir "$candidate" 2>/dev/null || true)
    if [[ -z "$resolved" ]] || [[ ! -d "$resolved" ]]; then
        KCAN_KERNEL_LOCATION_ERROR="Kernel tree path is not a directory: $candidate"
        return 1
    fi

    if ! kcan_is_kbuild_tree "$resolved"; then
        KCAN_KERNEL_LOCATION_ERROR="Path is not a usable kernel build tree: $resolved"
        return 1
    fi

    echo "$resolved"
}

kcan_standard_kernel_locations() {
    local kernel_version="${1:-$(uname -r)}"
    printf '%s\n' \
        "/lib/modules/$kernel_version/build" \
        "/lib/modules/$kernel_version/source"
}

kcan_find_kernel_location() {
    local kernel_version="${1:-$(uname -r)}"
    local cli_kernel_location="${2:-${CLI_KERNEL_LOCATION:-}}"
    local resolved=""
    local candidate=""

    kcan_reset_kernel_location_state

    if [[ -n "$cli_kernel_location" ]]; then
        kcan_record_kernel_attempt "$cli_kernel_location (CLI)"
        if resolved=$(kcan_validate_kernel_location "$cli_kernel_location"); then
            KCAN_KERNEL_LOCATION_SOURCE="cli"
            echo "$resolved"
            return 0
        fi
        return 1
    fi

    if [[ -n "${KERNEL_LOCATION:-}" ]]; then
        kcan_record_kernel_attempt "$KERNEL_LOCATION (env)"
        if resolved=$(kcan_validate_kernel_location "$KERNEL_LOCATION"); then
            KCAN_KERNEL_LOCATION_SOURCE="env"
            echo "$resolved"
            return 0
        fi
        return 1
    fi

    while IFS= read -r candidate; do
        [[ -n "$candidate" ]] || continue
        kcan_record_kernel_attempt "$candidate"
        if resolved=$(kcan_validate_kernel_location "$candidate" 2>/dev/null); then
            KCAN_KERNEL_LOCATION_SOURCE="standard"
            echo "$resolved"
            return 0
        fi
    done < <(kcan_standard_kernel_locations "$kernel_version")

    KCAN_KERNEL_LOCATION_ERROR="Unable to locate a usable kernel build tree for $kernel_version"
    return 1
}

kcan_kernel_tree_is_prepared() {
    local kernel_location="$1"
    [[ -d "$kernel_location/include/generated" ]]
}

kcan_print_kernel_prepare_guidance() {
    local kernel_location="$1"
    kcan_log_error "Kernel build tree is not prepared: $kernel_location"
    kcan_log_info "Run: make -C \"$kernel_location\" modules_prepare"
    kcan_log_info "Or install prepared headers so /lib/modules/\$(uname -r)/build points to a ready tree"
}

kcan_require_prepared_kernel_tree() {
    local kernel_location="$1"
    if kcan_kernel_tree_is_prepared "$kernel_location"; then
        return 0
    fi

    kcan_print_kernel_prepare_guidance "$kernel_location"
    return 1
}

kcan_print_kernel_location_guidance() {
    local kernel_version="${1:-$(uname -r)}"

    kcan_log_error "Unable to locate a usable kernel build tree for $kernel_version"
    if [[ -n "$KCAN_KERNEL_LOCATION_ERROR" ]]; then
        kcan_log_info "$KCAN_KERNEL_LOCATION_ERROR"
    fi

    if [[ ${#KCAN_KERNEL_LOCATION_ATTEMPTS[@]} -gt 0 ]]; then
        kcan_log_info "Checked locations:"
        local attempt=""
        for attempt in "${KCAN_KERNEL_LOCATION_ATTEMPTS[@]}"; do
            kcan_log_info "  - $attempt"
        done
    fi

    kcan_log_info "Provide an explicit kernel tree with one of the following:"
    kcan_log_info "  --kernel-location /path/to/kernel/tree"
    kcan_log_info "  KERNEL_LOCATION=/path/to/kernel/tree"
    kcan_log_info "Standard distributions usually provide /lib/modules/$kernel_version/build"
}
