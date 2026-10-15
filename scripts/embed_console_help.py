"""
Embed docs/Console commands.txt into the firmware as a C string header.

Single source of truth for the console command reference: the text lives in
docs/Console commands.txt, and showHelp() prints the embedded copy, so what the
device prints on `help` is exactly the committed doc — they cannot drift.

This runs as a PlatformIO pre-build extra_script. It converts the .txt into
src/generated/console_help.h containing:

    static const char CONSOLE_HELP[] = R"EVDASH_HELP(<file contents>)EVDASH_HELP";

The generated header is a build artifact (git-ignored). The header is only
rewritten when the content changes, so it does not trigger needless rebuilds.

Can also be run standalone for inspection:
    python scripts/embed_console_help.py
"""

import os

try:
    Import("env")  # type: ignore[name-defined]  # noqa: F821
    PROJECT_DIR = env["PROJECT_DIR"]  # type: ignore[name-defined]  # noqa: F821
except Exception:
    PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SOURCE = os.path.join(PROJECT_DIR, "docs", "Console commands.txt")
OUTPUT = os.path.join(PROJECT_DIR, "src", "generated", "console_help.h")

# Raw-string delimiter; must not appear in the text (it won't — it is an
# evDash-specific tag).
DELIM = "EVDASH_HELP"


def build_header(text):
    if DELIM + ")" in text or "(" + DELIM in text:
        raise RuntimeError("console help text contains the raw-string delimiter")
    return (
        "// AUTO-GENERATED from docs/Console commands.txt by\n"
        "// scripts/embed_console_help.py at build time. Do not edit by hand or\n"
        "// commit: edit the .txt and rebuild. (git-ignored)\n"
        "#pragma once\n"
        "\n"
        "static const char CONSOLE_HELP[] = R\"" + DELIM + "(" + text + ")" + DELIM + "\";\n"
    )


def main():
    if not os.path.exists(SOURCE):
        raise RuntimeError("missing " + SOURCE)
    with open(SOURCE, "r", encoding="utf-8") as f:
        text = f.read()

    header = build_header(text)

    existing = None
    if os.path.exists(OUTPUT):
        with open(OUTPUT, "r", encoding="utf-8") as f:
            existing = f.read()

    if existing != header:
        os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
        with open(OUTPUT, "w", encoding="utf-8") as f:
            f.write(header)
        print("embed_console_help: regenerated src/generated/console_help.h")
    else:
        print("embed_console_help: src/generated/console_help.h up to date")


main()
