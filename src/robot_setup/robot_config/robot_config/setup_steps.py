"""The command line of the setup steps that live in this package: python3 -m robot_config.setup_steps <command> ...

  config  --config FILE [--entry E] [--yes] [--status]   create robot.local.yaml from the template / add the keys it lacks
  dyn     --config FILE --ws-root DIR [--yes] [--reset]   tool load identification
  status  --config FILE STEP                              does the step (cameras, ee, dyn) still need the operator
  check   FILE                                            the configuration summary

scripts/setup.sh runs these; each command is the main of the module that implements it."""
from __future__ import annotations

import sys
from typing import Optional, Sequence


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    commands = ("config", "dyn", "status", "check")
    if not args or args[0] not in commands:
        print(f"usage: python3 -m robot_config.setup_steps {{{'|'.join(commands)}}} ...", file=sys.stderr)
        return 2
    command, rest = args[0], args[1:]
    if command == "config":
        from . import sync
        return sync.main(rest)
    if command == "dyn":
        from . import load_identify
        return load_identify.main(rest)
    from . import config_status
    return config_status.main([command, *rest])


if __name__ == "__main__":
    sys.exit(main())
