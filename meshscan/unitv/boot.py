# boot.py - MeshScan Unit V launcher (MaixPy v0.5 compatible).
#
# Replaces the M5StickV factory demo boot.py (backed up as
# boot_factory_stickv.py). This build of MaixPy v0.5.0 resolves filesystem
# imports fine from the REPL but NOT during boot (open() works, import does
# not) — so we don't rely on the import system at all: each firmware module
# is exec-loaded from /flash into a namespace object and pre-registered in
# sys.modules, then main.py runs via exec. `import config` etc. inside the
# modules then resolves from sys.modules without touching the filesystem.
# Bonus: SD copies can never shadow firmware again.

import sys
import time


class _NS:
    pass


def _read_retry(path, tries=12):
    # /flash reads are flaky in the first moments after boot on this build
    # (REPL sees the file, boot-time open intermittently raises ENOENT).
    # Retry with a short delay until the FS settles.
    for i in range(tries):
        try:
            return open(path).read()
        except OSError as e:
            print("[boot] open", path, "try", i + 1, "failed:", repr(e))
            time.sleep_ms(150)
    raise OSError("could not read " + path)


def _load(name, path):
    g = {"__name__": name}
    src = _read_retry(path)
    exec(src, g)
    ns = _NS()
    for k in g:
        setattr(ns, k, g[k])
    sys.modules[name] = ns
    print("[boot] loaded", name)
    return ns


try:
    # dependency order: config first, then the modules that import it
    _load("config", "/flash/config.py")
    _load("uart_protocol", "/flash/uart_protocol.py")
    _load("detector", "/flash/detector.py")
    print("[boot] modules loaded, starting main")
    exec(_read_retry("/flash/main.py"))
except KeyboardInterrupt:
    pass                      # Ctrl-C drops to the REPL
except Exception as e:
    print("[boot] main.py stopped:", repr(e))
    sys.print_exception(e)
